package gcn.core

import chisel3._
import chisel3.util._
import vta.util.config._
import scala.math._
import ISA._
import gcn.core.util.MuxTree

// /** Processing Element.
//  *
//  * Takes instructions from fetch module. Schedules computation between PEs.
//  * Each PE instantiates each scratchpad buffer
//  */
class PipelinedPECSR(debug: Boolean = false)(implicit p: Parameters) extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams
  val regBits = p(AccKey).crParams.regBits 
  val io = IO(new Bundle {
    val peReq = Flipped(Decoupled(new PECSRIO))
    val spWrite = Vec(cp.nScratchPadMem, Flipped(Decoupled(new SPWriteCmd)))
    val spOutWrite = Decoupled(new SPWriteCmd(scratchType = "Col")) 
    val ecnt = Output(Vec(p(AccKey).crParams.nPEEventCtr, UInt(regBits.W)))
    val free = Output(Bool())
  })

  val writeEnVec = Wire(Vec(cp.nScratchPadMem, Bool()))
  val d1Time = RegInit(0.U(regBits.W))
  val d2Time = RegInit(0.U(regBits.W))
  val mTime = RegInit(0.U(regBits.W))
  val drTime = RegInit(0.U(regBits.W))
  val peTime = RegInit(0.U(regBits.W))
  val drainTime = RegInit(0.U(regBits.W))

  for(i <- 0 until cp.nScratchPadMem){
    io.spWrite(i).ready := true.B
    writeEnVec(i) := io.spWrite(i).fire
  }

  // Scratchpad Instantiation
  val spVal = Module(new Scratchpad(scratchType = "Val"))
  val spCol = Module(new Scratchpad(scratchType = "Col"))
  val spPtr = Module(new Scratchpad(scratchType = "Ptr"))
  val spDen = Module(new Scratchpad(scratchType = "Den"))
  val spPsum = Module(new Scratchpad(scratchType = "Psum"))
  val out_q = Module(new Queue(new SPWriteCmd(scratchType = "Out"), cp.peOutputScratchQueueEntries))
  io.spWrite(0).bits <> spVal.io.spWrite
  writeEnVec(0)      <> spVal.io.writeEn
  io.spWrite(1).bits <> spDen.io.spWrite
  writeEnVec(1)      <> spDen.io.writeEn
  io.spWrite(2).bits <> spPtr.io.spWrite
  writeEnVec(2)      <> spPtr.io.writeEn
  io.spWrite(3).bits <> spCol.io.spWrite
  writeEnVec(3)      <> spCol.io.writeEn
  io.spWrite(4).bits <> spPsum.io.spWrite
  writeEnVec(4)      <> spPsum.io.writeEn
  io.spOutWrite <> out_q.io.deq

  val blockSizeBytes = (cp.blockSize/8)

  /* Pipeline Stage: D1
  Cycles = 2
  Inputs:
    1. Row assignment from Compute module via peReq
  Performs:
    1. Uses a statemachine to read two consecutive addresses in row_ptr starting from assigned row
    2. Sends rowPtrData1, rowPtrData2 to next pipeline stage D2
  Output:
    1. rowPtrData1, rowPtrData2, (currRowPtr = rowPtr1Data) and peReq goes to D2.
  */

  val d1_moving = Wire(Bool())
  val sIdle :: sRowPtr1 :: sRowPtr2 :: Nil = Enum(3)
  val d1_state_q = RegInit(sIdle)
  val d1_ready = (d1_state_q === sIdle) || ((d1_state_q === sRowPtr2) && d1_moving)
  val d1_isrowPtr1 = ((d1_state_q === sIdle) || ((d1_state_q === sRowPtr2) && d1_moving))
  val d1_rowPtrAddr1 = io.peReq.bits.sramPtr + (io.peReq.bits.rowIdx << log2Ceil(cp.blockSize/8))
  val d1_peReq_q = RegEnable(io.peReq.bits, io.peReq.fire)
  val d1_peReqNext_q = RegNext(d1_peReq_q)
  val d1_rowPtrAddr2 = d1_peReq_q.sramPtr + ((d1_peReq_q.rowIdx + 1.U) << log2Ceil(blockSizeBytes)) 
  io.peReq.ready := d1_ready

  switch(d1_state_q){
    is(sIdle){
      when(io.peReq.valid){
        d1_state_q := sRowPtr1
      }
    }
    is(sRowPtr1){
      d1_state_q := sRowPtr2
    }
    is(sRowPtr2){
      when(d1_moving){
        when(io.peReq.valid){
          d1_state_q := sRowPtr1
        }.otherwise{
          d1_state_q := sIdle 
        }
      }
    }
  }
  when(d1_state_q =/= sIdle){
    d1Time := d1Time + 1.U
  }

  spPtr.io.spReadCmd.addr  := Mux(d1_isrowPtr1, d1_rowPtrAddr1, d1_rowPtrAddr2)
  val d1_rowPtr1Data_q = RegEnable(spPtr.io.spReadData.data, d1_state_q === sRowPtr1)
  val d1_rowPtr2Data_q = RegEnable(spPtr.io.spReadData.data, d1_state_q === sRowPtr2)
  val d1_rowPtr2Data = Mux(d1_state_q === sRowPtr2, spPtr.io.spReadData.data, d1_rowPtr2Data_q)
  val d1_isRowEmpty = (d1_rowPtr2Data - d1_rowPtr1Data_q) === 0.U
  val d1_valid = (d1_state_q === sRowPtr2) && !d1_isRowEmpty
  val d1_currRowPtr = d1_rowPtr1Data_q
  val d1_currDenCol = 0.U(cp.blockSize.W)

  /* Pipeline Stage: D2
  Cycles = 1
  Inputs:
    1. rowPtr1Data, rowPtr2Data and (currRowPtr = rowPtr1Data), peReq from D1 stage
    2. rowPtr1Data, rowPtr2Data, updated currRowPtr, peReq from D2 stage
  Performs:
    1. Arbitrates between requests from D1 stage and prev D2 stage. prev D2 stage is always given priority
    2. Reads the colIdx and sends the data to stage DR
  Output:
    1. colIdx and peReq goes to DR.
  */
  val d2_nextValid = Wire(Bool())
  val d2_nextValid_q = RegInit(false.B)
  val d2_nextRowPtr_q = Reg(chiselTypeOf(d1_currRowPtr))
  val d2_nextDenCol_q = Reg(chiselTypeOf(d1_currDenCol))
  val d2_valid_q = RegInit(false.B)
  val d2_rowPtr1Data_q = RegInit(0.U(cp.blockSize.W))
  val d2_rowPtr2Data_q = RegInit(0.U(cp.blockSize.W))
  val d2_currRowPtr_q = RegInit(0.U(cp.blockSize.W))
  val d2_currDenCol_q = RegInit(0.U(cp.blockSize.W))
  val d2_peReq_q = Reg(chiselTypeOf(d1_peReqNext_q))
  val d2_peReq = Mux(d2_nextValid_q, d2_peReq_q, d1_peReqNext_q)
  val d2_currRowPtr = Mux(d2_nextValid_q, d2_nextRowPtr_q, d1_currRowPtr)
  val d2_currDenCol = Mux(d2_nextValid_q, d2_nextDenCol_q, d1_currDenCol)
  val d2_rowPtr1Data = Mux(d2_nextValid_q, d2_rowPtr1Data_q, d1_rowPtr1Data_q)
  val d2_rowPtr2Data = Mux(d2_nextValid_q, d2_rowPtr2Data_q, d1_rowPtr2Data_q)
  val d2_endOfRow_q = RegInit(false.B)
  val d2_endOfCol_q = RegInit(false.B)
  val d2_isNewOutput_q = RegNext(d2_currRowPtr === d2_rowPtr1Data)
  d1_moving := !d2_nextValid
  d2_valid_q := d2_nextValid || d1_valid
  
  d2_peReq_q := d2_peReq
  d2_rowPtr1Data_q := d2_rowPtr1Data
  d2_rowPtr2Data_q := d2_rowPtr2Data
  d2_currRowPtr_q := d2_currRowPtr
  d2_currDenCol_q := d2_currDenCol
  spCol.io.spReadCmd.addr := d2_peReq.sramColVal + (d2_currRowPtr << log2Ceil(blockSizeBytes))
  val d2_endOfCol = (d2_currDenCol === d2_peReq.denXSize - 1.U)
  val d2_endOfRow = (d2_currRowPtr === d2_rowPtr2Data - 1.U)
  d2_endOfRow_q := d2_endOfRow
  d2_endOfCol_q := d2_endOfCol
  val d2_endOfColRow = d2_endOfCol && d2_endOfRow
  d2_nextDenCol_q := Mux(d2_endOfRow, d2_currDenCol + 1.U, d2_currDenCol)
  d2_nextRowPtr_q := Mux(d2_endOfRow, d2_rowPtr1Data , d2_currRowPtr + 1.U)
  d2_nextValid := !d2_endOfColRow && d2_valid_q
  d2_nextValid_q := d2_nextValid

  when(d2_valid_q){
    d2Time := d2Time + 1.U
  }

  /* Pipeline Stage: DR (DataRead)
  Cycles = 1
  Inputs:
    1. colIdx, denCol  and peReq from D2 stage
    2. colIdx, denCol  and peReqfrom M stage
  Performs:
    1. Arbitrates between D1 stage and M stage. M stage is always given priority
    2. Reads the colIdx and sends the data to stage M
  Output:
    1. RowPtrData1, RowPtrData2 and goes to D2.
  */
  val dr_valid_q = RegNext(d2_valid_q)
  val dr_colIdx = spCol.io.spReadData.data
  val dr_peReqdenXSize_q = RegEnable(d2_peReq_q.denXSize, dr_valid_q)
  val dr_currSpaRow_q = RegEnable(d2_peReq_q.rowIdx, dr_valid_q)
  val dr_currDenCol_q = RegEnable(d2_currDenCol_q, dr_valid_q)
  val dr_outWrite_q = RegEnable(d2_endOfRow_q, dr_valid_q)
  val dr_isNewOutput_q = RegEnable(d2_isNewOutput_q, dr_valid_q)
  spVal.io.spReadCmd.addr := d2_peReq_q.sramColVal + (d2_currRowPtr_q << log2Ceil(blockSizeBytes))
  spDen.io.spReadCmd.addr := d2_peReq_q.sramDen + ((dr_colIdx << log2Ceil(blockSizeBytes)) << Log2(d2_peReq_q.denXSize)) + (d2_currDenCol_q << log2Ceil(blockSizeBytes))
  spPsum.io.spReadCmd.addr := (((d2_peReq_q.rowIdx << log2Ceil(cp.blockSize/8))) << Log2(d2_peReq_q.denXSize)) + (d2_currDenCol_q << log2Ceil(cp.blockSize/8))
  when(dr_valid_q){
    drTime := drTime + 1.U
  }
  /* Pipeline Stage: M (MAC)
  Cycles = 1
  */
  val m_isNewRow = dr_isNewOutput_q
  val m_acc_q = RegInit(0.U(cp.blockSize.W))
  val m_valid_q = RegNext(dr_valid_q)
  val m_dense = spDen.io.spReadData.data
  val m_sparse = spVal.io.spReadData.data
  val m_multiply = (m_dense * m_sparse)
  val m_mac = Mux(dr_isNewOutput_q, spPsum.io.spReadData.data + m_multiply , m_acc_q + m_multiply) 
  val m_outWrite = dr_outWrite_q
  out_q.io.enq.bits.addr := (((dr_currSpaRow_q << log2Ceil(cp.blockSize/8))) << Log2(dr_peReqdenXSize_q)) + (dr_currDenCol_q << log2Ceil(cp.blockSize/8))
  out_q.io.enq.valid := m_outWrite && m_valid_q
  out_q.io.enq.bits.data :=  m_mac
  when(m_valid_q){
    m_acc_q := m_mac
  }.otherwise{
    m_acc_q := m_acc_q
  }
  when(m_valid_q){
    mTime := mTime + 1.U
  }
  val pipeEmpty = d1_ready && !dr_valid_q && !d2_valid_q && !m_valid_q
  when(!pipeEmpty){
    peTime := peTime + 1.U
  }
  io.free := pipeEmpty
  when(pipeEmpty && (out_q.io.count =/= 0.U)){
    drainTime := drainTime + 1.U
  }


  io.ecnt(0) := d1Time
  io.ecnt(1) := d2Time
  io.ecnt(2) := drTime
  io.ecnt(3) := mTime
  io.ecnt(4) := peTime
  io.ecnt(5) := drainTime
}
