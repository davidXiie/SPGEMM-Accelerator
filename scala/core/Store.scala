package gcn.core

import chisel3._
import chisel3.util._
import vta.util.config._
import scala.math._

/** Store.
 *
 * Store data from output scratchpad to DRAM
 */
class Store(debug: Boolean = false)(implicit p: Parameters) extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams
  val regBits = p(AccKey).crParams.regBits
  val io = IO(new Bundle {
    val inst = Flipped(Decoupled(UInt(INST_BITS.W)))
    val spReadCmd = Output(new SPReadCmd())
    val spReadData = Input(new SPReadData(scratchType = "Out"))
    val me_wr = new MEWriteMaster
    val valid = Input(Bool())
    val done = Output(Bool())
    // val ecnt = Output(UInt(regBits.W))
  })

  // Module instantiation
  val inst_q = Module(new Queue(UInt(INST_BITS.W), cp.loadInstQueueEntries))
  val dec = Module(new StoreDecode)
  val storeTime = RegInit(0.U(regBits.W))
  // state machine

  val sIdle :: sWriteCmd :: sWriteData :: sReadMem :: sWriteAck :: Nil = Enum(5)
  val state = RegInit(sIdle)
  val start = inst_q.io.deq.fire
  val done = RegInit(false.B)
  io.done := done
  val inst = RegEnable(inst_q.io.deq.bits, start)
  val maxTransferPerReq = (1 << mp.lenBits).U
  val waddr = Reg(chiselTypeOf(io.me_wr.cmd.bits.addr))
  val wlen = Reg(chiselTypeOf(io.me_wr.cmd.bits.len))
  val transferMaxSizeBytes = (mp.lenBits + 1) << log2Ceil(mp.dataBits / 8)
  val saddr = Reg(UInt(M_SRAM_OFFSET_BITS.W))
  val isStride = (dec.io.ySize =/= 0.U)
  val wcnt =  Reg(chiselTypeOf(io.me_wr.cmd.bits.len))
  val nBlockPerTransfer = mp.dataBits / cp.blockSize
  val transferTotal = WireDefault((dec.io.xSize)-1.U >> log2Ceil(nBlockPerTransfer)) + 1.U
  val transferRem = Reg(chiselTypeOf(dec.io.xSize))
  val totalBytes = WireDefault((dec.io.xSize) << log2Ceil(cp.blockSize/8)) 
  val totalBytesWritten = Reg(chiselTypeOf(totalBytes))
  val totalBytesRem = totalBytes - totalBytesWritten
  val currBytes = Mux(totalBytesRem >= (mp.dataBits/8).U, (mp.dataBits/8).U, totalBytesRem)

  when(state === sIdle){
    totalBytesWritten := 0.U
  }.elsewhen((state === sWriteData) && (io.me_wr.data.ready)){
    totalBytesWritten := totalBytesWritten + currBytes
  }

  // instruction queue
  dec.io.inst := Mux(start, inst_q.io.deq.bits, inst)

  switch(state){
    is(sIdle){
      done := false.B
      when(start){
        when(dec.io.xSize === 0.U){
          done := true.B
        }.otherwise{
          when(!isStride){
            waddr := dec.io.dramOffset
            saddr := dec.io.sramOffset
            state := sWriteCmd
            when(transferTotal < maxTransferPerReq){
              wlen := transferTotal - 1.U
              transferRem := 0.U
            }.otherwise{
              wlen := maxTransferPerReq - 1.U
              transferRem := transferTotal - (maxTransferPerReq)
            }
          }
        }
      }
    }
    is(sWriteCmd){
      when(io.me_wr.cmd.ready) {
        state := sReadMem
      }
    }
    is(sWriteData){
      when(io.me_wr.data.ready){
        when(wcnt === wlen){
          state := sWriteAck
        }.otherwise{
          state := sReadMem
        }
      }
    }
    is(sReadMem) {
      state := sWriteData
    }
    is(sWriteAck){
      when(io.me_wr.ack) {
        when(transferRem === 0.U){
          done := true.B
          state := sIdle
        }.otherwise{
          state := sWriteCmd
          waddr := waddr + transferMaxSizeBytes.U
          saddr := waddr + transferMaxSizeBytes.U
          when(transferRem < maxTransferPerReq){
            wlen := transferRem - 1.U
            transferRem := 0.U
          }.otherwise{
            wlen := maxTransferPerReq - 1.U
            transferRem := transferRem - maxTransferPerReq
          }
        }
      }
    }
  }


  when(state === sWriteCmd) {
    wcnt := 0.U
  }.elsewhen(io.me_wr.data.fire) {
    wcnt := wcnt + 1.U
  }

  // instructions
  inst_q.io.enq <> io.inst
  inst_q.io.deq.ready := (state === sIdle) && io.valid
 
  //sram read
  io.spReadCmd.addr := saddr
  
  // dram read
  io.me_wr.cmd.bits.len := wlen
  io.me_wr.cmd.bits.tag := dec.io.sramOffset
  io.me_wr.cmd.bits.addr := waddr
  io.me_wr.cmd.valid := (state === sWriteCmd)


  io.me_wr.data.valid := state === sWriteData
  io.me_wr.data.bits.data := io.spReadData.data
  // io.me_wr.data.bits.strb := Fill(io.me_wr.data.bits.strb.getWidth, true.B)
  io.me_wr.data.bits.strb := (for(i <- 0 until (mp.dataBits/8))yield{
    (i.U < totalBytesRem).asUInt
  }).reverse.reduce(Cat(_,_))

  // Store execution time
  // when(done){
  //   storeTime := 0.U
  // }.elsewhen(start || storeTime =/= 0.U){
  //   storeTime := storeTime + 1.U
  // }

  // io.ecnt := storeTime

}
