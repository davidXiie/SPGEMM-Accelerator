package gcn.core

import chisel3._
import chisel3.util._
import vta.util.config._
import scala.math._

/** Load.
 *
 * Load inputs and weights from memory (DRAM) into scratchpads (SRAMs).
 * This module instantiate the TensorLoad unit which is in charge of
 * loading 1D and 2D tensors to scratchpads, so it can be used by
 * other modules such as Compute.
 */
class Load(debug: Boolean = false)(implicit p: Parameters) extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams
  val regBits = p(AccKey).crParams.regBits
  val io = IO(new Bundle {
    val inst = Flipped(Decoupled(UInt(INST_BITS.W)))
    val me_rd = new MEReadMaster
    val valid = Input(Bool())
    val done = Output(Bool())
    val spWrite = Decoupled(new SPWriteCmd(scratchType = "Global"))
    val ecnt = Output(UInt(regBits.W))
  })
  // Module instantiation
  val inst_q = Module(new Queue(UInt(INST_BITS.W), cp.loadInstQueueEntries))
  val data_qEntries = (1 << mp.lenBits)
  val data_q = Module(new Queue(new SPWriteCmd(scratchType = "Global"), data_qEntries))
  val dec = Module(new LoadDecode)
  val loadTime = RegInit(0.U(regBits.W))
  
  // state machine
  val sIdle :: sStride :: sSeq :: sSeqCmd :: sSeqReadData :: sDelay :: Nil = Enum(6)
  val state = RegInit(sIdle)
  val start = inst_q.io.deq.fire
  val done = RegInit(false.B)
  io.done := done
  val inst = RegEnable(inst_q.io.deq.bits, start)
  val nBlockPerTransfer = mp.dataBits / cp.blockSize
  val transferTotal = WireDefault((dec.io.xSize)-1.U >> log2Ceil(nBlockPerTransfer)) + 1.U
  val transferRem = Reg(chiselTypeOf(dec.io.xSize))
  val maxTransferPerReq = (1 << mp.lenBits).U
  val raddr = Reg(chiselTypeOf(io.me_rd.cmd.bits.addr))
  val rlen = Reg(chiselTypeOf(io.me_rd.cmd.bits.len))
  val rlenRem = Reg(chiselTypeOf(io.me_rd.cmd.bits.len))
  val transferMaxSizeBytes = (mp.lenBits + 1) << log2Ceil(mp.dataBits / 8)
  val saddr = Reg(UInt(M_SRAM_OFFSET_BITS.W))
  val mask = UInt((mp.dataBits/cp.blockSize).W)
  val delayCtr = RegInit(0.U(5.W))

  // instruction queue
  dec.io.inst := Mux(start, inst_q.io.deq.bits, inst)

  val scratchSel = Cat(dec.io.isPsum, dec.io.isCol, dec.io.isPtr, !dec.io.isSeq, dec.io.isVal) // col,ptr,den,val

  // control
  switch(state) {
    is(sIdle) {
      done := false.B
      when(start) {
        when(dec.io.isSeq || !dec.io.isSeq){
          state := sSeqCmd
          raddr := dec.io.dramOffset
          saddr := dec.io.sramOffset
          when(dec.io.xSize === 0.U){
            state := sDelay
            delayCtr := 1.U
          }.otherwise{
            when(transferTotal < maxTransferPerReq){
              rlen := transferTotal - 1.U
              rlenRem := transferTotal - 1.U
              transferRem := 0.U
            }.otherwise{
                rlen := maxTransferPerReq - 1.U
                rlenRem := maxTransferPerReq - 1.U
                transferRem := transferTotal - (maxTransferPerReq)
            }
          }
        }.otherwise{
          state := sStride
        }
      }
    }
    is(sSeqCmd){
      when(data_q.io.count === 0.U){
        when(io.me_rd.cmd.ready){
          state := sSeqReadData 
       }
      }
    }
    is(sSeqReadData){
      when(io.me_rd.data.valid){
        saddr := saddr + (mp.dataBits/8).U
          when(rlenRem === 0.U){
            when(transferRem === 0.U){
              state := sDelay
            }.otherwise{
              state := sSeqCmd
              raddr := raddr + transferMaxSizeBytes.U
              when(transferRem < maxTransferPerReq){
                rlen := transferRem - 1.U
                rlenRem := transferRem - 1.U
                transferRem := 0.U
              }.otherwise{
                rlen := maxTransferPerReq - 1.U
                rlenRem := maxTransferPerReq - 1.U
                transferRem := transferRem - maxTransferPerReq
              }
            }
          }.otherwise{
            rlenRem := rlenRem - 1.U
          }
      }
    }
    is(sStride) {
      done := true.B
      state := sIdle
    }
    is(sDelay){
      when(data_q.io.count === 0.U){
        done := true.B
        state := sIdle
      }
    }
  }


  // instructions
  inst_q.io.enq <> io.inst
  inst_q.io.deq.ready := (state === sIdle) && io.valid

  // data queue
  data_q.io.enq.bits.data := io.me_rd.data.bits.data
  data_q.io.enq.bits.addr := saddr
  data_q.io.enq.valid := (state === sSeqReadData) && io.me_rd.data.valid && !dec.io.isPsum
  
  // dram read
  io.me_rd.cmd.bits.len := rlen
  io.me_rd.cmd.bits.tag := dec.io.sramOffset
  io.me_rd.cmd.bits.addr := raddr
  io.me_rd.cmd.valid := (state === sSeqCmd) && (data_q.io.count === 0.U)
  io.me_rd.data.ready := true.B

  // Data Write Queue to multiple scratchpad
  io.spWrite <> data_q.io.deq
  
// Load execution time
when(done){
  loadTime := 0.U
}.elsewhen(start || loadTime =/= 0.U){
  loadTime := loadTime + 1.U
}

io.ecnt := loadTime
}
