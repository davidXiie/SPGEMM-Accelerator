package gcn.core

import chisel3._
import chisel3.util._
import vta.util.config._
import scala.math._
import ISA._
import gcn.core.util.MuxTree

class PECSRIO(implicit p: Parameters) extends Bundle{
  val cp = p(AccKey).coreParams
  val sramColVal = Input(UInt(C_SRAM_OFFSET_BITS.W))
  val sramPtr = Input(UInt(C_SRAM_OFFSET_BITS.W))
  val sramDen = Input(UInt(C_SRAM_OFFSET_BITS.W))
  val denXSize = Input(UInt(C_XSIZE_BITS.W))
  val spaYSize = Input(UInt(C_YSIZE_BITS.W))
  val rowIdx = Input(UInt(C_XSIZE_BITS.W))
}


// /** Processing Element.
//  *
//  * Takes instructions from fetch module. Schedules computation between PEs.
//  * Each PE instantiates each scratchpad buffer
//  */
class PECSR(debug: Boolean = false)(implicit p: Parameters) extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams
  val regBits = p(AccKey).crParams.regBits 
  val io = IO(new Bundle {
    val peReq = Flipped(Decoupled(new PECSRIO))
    val spWrite = Vec(cp.nScratchPadMem, Flipped(Decoupled(new SPWriteCmd)))
    val spOutWrite = Decoupled(new SPWriteCmd(scratchType = "Col")) 
    val ecnt = Vec(p(AccKey).crParams.nPEEventCtr, ValidIO(UInt(regBits.W)))
    val free = Output(Bool())
  })

  val writeEnVec = Wire(Vec(cp.nScratchPadMem, Bool()))
  val d1Time = RegInit(0.U(regBits.W))
  val d2Time = RegInit(0.U(regBits.W))
  val macTime = RegInit(0.U(regBits.W))
  val peTime = RegInit(0.U(regBits.W))

  for(i <- 0 until cp.nScratchPadMem){
    io.spWrite(i).ready := true.B
    writeEnVec(i) := io.spWrite(i).fire
  }

  // Scratchpad Instantiation
  val spVal = Module(new Scratchpad(scratchType = "Val"))
  val spCol = Module(new Scratchpad(scratchType = "Col"))
  val spPtr = Module(new Scratchpad(scratchType = "Ptr"))
  val spDen = Module(new Scratchpad(scratchType = "Den"))
  val out_q = Module(new Queue(new SPWriteCmd(scratchType = "Out"), cp.peOutputScratchQueueEntries))
  io.spWrite(0).bits <> spVal.io.spWrite
  writeEnVec(0)      <> spVal.io.writeEn
  io.spWrite(1).bits <> spDen.io.spWrite
  writeEnVec(1)      <> spDen.io.writeEn
  io.spWrite(2).bits <> spPtr.io.spWrite
  writeEnVec(2)      <> spPtr.io.writeEn
  io.spWrite(3).bits <> spCol.io.spWrite
  writeEnVec(3)      <> spCol.io.writeEn
  io.spOutWrite <> out_q.io.deq

  val blockSizeBytes = (cp.blockSize/8)

  // Registers 
  val colCurr_q = RegInit(0.U(32.W))
  val rowPtr1Data = RegInit(0.U(32.W))
  val rowPtr2Data = RegInit(0.U(32.W))
  val macCount = RegInit(0.U(32.W)) 
  val acc_q = RegInit(0.U(cp.blockSize.W))
  val denCol_q = RegInit(0.U(32.W))
  val ptrNext = RegInit(0.U(32.W))
  val rowNum_q = RegInit(0.U(32.W))
  val rowNumNext = rowNum_q + cp.nPE.U
  // state machine

  val sIdle :: sRowPtr1 :: sRowPtr2 :: sCol :: sMAC :: Nil = Enum(5)
  val state = RegInit(sIdle)
  val startInst = io.peReq.fire && (io.peReq.bits.rowIdx < io.peReq.bits.spaYSize)
  val peReq_q = dontTouch(RegEnable(io.peReq.bits, startInst))
  val colCurr = Mux(state === sCol, spCol.io.spReadData.data, colCurr_q)
  val endOfRow = ((state === sMAC) && (ptrNext === rowPtr2Data))
  val denCol = Mux(endOfRow, denCol_q + 1.U, denCol_q)
  val endOfCol = ((state === sMAC) && (denCol === peReq_q.denXSize))
  val nonZeroInRow = Mux((state === sRowPtr2), spPtr.io.spReadData.data - rowPtr1Data, rowPtr2Data - rowPtr1Data)
  val ptrCurr = Mux((endOfRow && !endOfCol) || (nonZeroInRow === 1.U), rowPtr1Data, ptrNext)
  val acc = acc_q + (spVal.io.spReadData.data * spDen.io.spReadData.data)
  val spOutWrite = endOfRow
  val spOutWriteData = acc
  val done = ((state === sRowPtr2) && (nonZeroInRow === 0.U)
             || (endOfRow && endOfCol))
  val rowNum =Mux(startInst, io.peReq.bits.rowIdx, Mux(done, rowNumNext,rowNum_q))
  val start = dontTouch((done && !((rowNumNext) >= peReq_q.spaYSize)))


  val rowPtrAddr = io.peReq.bits.sramPtr + ((rowNum) << log2Ceil(cp.blockSize/8))
  val rowPtrAddr_q = peReq_q.sramPtr + ((rowNum) << log2Ceil(cp.blockSize/8))
  val rowPtrSPAddr = Mux(start || startInst,rowPtrAddr, rowPtrAddr_q)
  val rowPtrSPAddrNext = rowPtrSPAddr + (cp.blockSize/8).U

  spPtr.io.spReadCmd.addr := Mux((state === sIdle) || done || (endOfRow && ! endOfCol), rowPtrSPAddr, rowPtrSPAddrNext)



  val colIdxAddr = peReq_q.sramColVal + (ptrCurr << log2Ceil(cp.blockSize/8))
  spCol.io.spReadCmd.addr := colIdxAddr


  val valAddr = io.peReq.bits.sramColVal + (ptrCurr << log2Ceil(cp.blockSize/8))
  spVal.io.spReadCmd.addr := valAddr
  val denAddr = io.peReq.bits.sramDen + ((colCurr << log2Ceil(cp.blockSize/8)) << Log2(peReq_q.denXSize)) + (denCol << log2Ceil(cp.blockSize/8))
  spDen.io.spReadCmd.addr := denAddr
  val outWriteEn = endOfRow
  out_q.io.enq.bits.addr := (((rowNum_q << log2Ceil(cp.blockSize/8))) << Log2(peReq_q.denXSize)) + (denCol_q << log2Ceil(cp.blockSize/8))
  out_q.io.enq.bits.data := acc
  out_q.io.enq.valid := outWriteEn

  io.peReq.ready := ((state === sIdle) || done) &&  (out_q.io.count === 0.U)
  val done_q = WireDefault(false.B)
  done_q := ((state === sIdle) && (RegNext(state) =/= sIdle))

  switch(state){
    is(sIdle){
      denCol_q := 0.U
      rowNum_q := io.peReq.bits.rowIdx
      when(startInst || start){
        state := sRowPtr1
      }
    }
    is(sRowPtr1){
      acc_q := 0.U
      state := sRowPtr2
      rowPtr1Data := spPtr.io.spReadData.data
      ptrNext := spPtr.io.spReadData.data
    }
    is(sRowPtr2){
      rowPtr2Data := spPtr.io.spReadData.data
      when(nonZeroInRow === 0.U){
        when(start){
          rowNum_q := rowNumNext
          acc_q := 0.U
          state := sRowPtr1
        }.otherwise{
          state := sIdle
        }
      }.otherwise{
        state := sCol
      }
    }
    is(sCol){
      state := sMAC
      colCurr_q := spCol.io.spReadData.data
      ptrNext := ptrCurr + 1.U
    }
    is(sMAC){
      when(endOfRow){
        acc_q := 0.U
        when(endOfCol){
          when(start){
            rowNum_q := rowNumNext
            state := sRowPtr1
          }.otherwise{
            state := sIdle
          }
          denCol_q := 0.U
        }.otherwise{
          denCol_q := denCol_q + 1.U
          state := sCol
          ptrNext := rowPtr1Data
        }
      }.otherwise{
        state := sCol
        acc_q :=  acc
      }
    }
  }

  when(state === sRowPtr1 || state === sRowPtr2){
    d1Time := d1Time + 1.U
  }.elsewhen(done_q){
    d1Time := 0.U
  }
  when(state === sCol){
    d2Time := d2Time + 1.U
  }.elsewhen(done_q){
    d2Time := 0.U
  }
  when(state === sMAC){
    macTime := macTime + 1.U
  }.elsewhen(done_q){
    macTime := 0.U
  }
  when(state =/= sIdle){
    peTime := peTime + 1.U
  }.elsewhen(done_q){
    peTime := 0.U
  }

  
  io.ecnt(0).bits := d1Time
  io.ecnt(1).bits := d2Time
  io.ecnt(2).bits := macTime
  io.ecnt(3).bits := peTime
  io.ecnt(0).valid := done_q
  io.ecnt(1).valid := done_q
  io.ecnt(2).valid := done_q
  io.ecnt(3).valid := done_q
  io.free := (state === sIdle)&&(out_q.io.count === 0.U)
}
