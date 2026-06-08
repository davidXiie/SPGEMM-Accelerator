package gcn.core

import chisel3._
import chisel3.util._
import vta.util.config._
import vta.util.genericbundle._
import gcn.core.util._
import scala.math._

/** CRBase. Parametrize base class. */
abstract class CRBase(implicit p: Parameters) extends GenericParameterizedBundle(p)

/** CRMaster.
 *
 * This is the master interface used by CR in the Accelerator to control
 * the Core unit.
 */
class CRMaster(implicit p: Parameters) extends CRBase {
  val cp = p(AccKey).crParams
  val mp = p(AccKey).memParams
  val launch = Output(Bool())
  val finish = Input(Bool())
  val ecnt = Input(Vec(cp.nEventCtr, UInt(cp.regBits.W)))
  val vals = Output(Vec(cp.nMmapReg, UInt(cp.regBits.W)))
}

/** CRClient.
 *
 * This is the slave interface used by the Core module to communicate
 * to the CR in the Accelerator.
 */
class CRClient(implicit p: Parameters) extends CRBase {
  val cp = p(AccKey).crParams
  val mp = p(AccKey).memParams
  val launch = Input(Bool())
  val finish = Output(Bool())
  val ecnt = Output(Vec(cp.nEventCtr, UInt(cp.regBits.W)))
  val vals = Input(Vec(cp.nMmapReg, UInt(cp.regBits.W)))
}
/** Control Registers (CR).
 *
 * This unit provides control registers (32 bits) to be used by a control
 * unit. These registers are read-only by the core.
 * ****************** TO-DO *************************
 * Add event counter registers to CR
 */
class CR(implicit p: Parameters) extends Module {
  val crParams = p(AccKey).crParams
  val regBits = crParams.regBits
  val io = IO(new Bundle {
    val host = new AXILiteClient(p(AccKey).hostParams)
    val cr = new CRMaster
  })
 /*
  ******* Slave Registers *********

  0 - 0x0 - start
  1 - 0x4 - instruction base address
  2 - 0x8 - instruction count
  
  ******* Event Counters *********
  
  3 - 0xc - finish
  4 - 0x10 - total time
  5 - 0x14 - load time
  6 - 0x18 - compute time
  7 - 0x20 - store time
  (8,9,10,11) - D1, D2, MAC, PE time 
  .
  .
  .
 */
  val nSlaveReg = crParams.nSlaveReg
  val slaveReg  = RegInit(VecInit(Seq.fill(nSlaveReg)(0.U(regBits.W))))
  val rdata  = WireDefault(slaveReg(0))
  val slaveRegReadSelect = (io.host.ar.bits.addr >> 2)(log2Ceil(nSlaveReg)-1,0)
  val totalTime = RegInit(0.U(regBits.W))
  // CR IO 
  val pulse = Wire(Bool())
  pulse := slaveReg(0)(0).asBool && !RegNext(slaveReg(0)(0), init  = false.B)
  io.cr.launch := pulse
  for {i <- 0 until crParams.nMmapReg}{
    io.cr.vals(i) := slaveReg(i+1)
  }

  // read control (AR, R)
  val sReadAddress :: sReadData :: Nil = Enum(2)
  val rstate = RegInit(sReadAddress)

  switch(rstate) {
    is(sReadAddress) {
      when(io.host.ar.valid) {
        rstate := sReadData
      }
    }
    is(sReadData) {
      when(io.host.r.ready) {
        rstate := sReadAddress
      }
    }
  }

  when(io.host.r.fire){rdata := MuxTree(slaveRegReadSelect, slaveReg)}

  io.host.ar.ready := rstate === sReadAddress
  io.host.r.valid := rstate === sReadData
  io.host.r.bits.data := rdata
  io.host.r.bits.resp := 0.U

  // Write control (AW, W, B)
  val waddr = RegInit("h_ffff".U(regBits.W)) // init with invalid address
  val wdata = io.host.w.bits.data
  val sWriteAddress :: sWriteData :: sWriteResponse :: Nil = Enum(3)
  val wstate = RegInit(sWriteAddress)
  val slaveRegWriteSelect = (waddr >> 2)(log2Ceil(nSlaveReg)-1,0)

  switch(wstate) {
    is(sWriteAddress) {
      when(io.host.aw.valid) {
        wstate := sWriteData
      }
    }
    is(sWriteData) {
      when(io.host.w.valid) {
        wstate := sWriteResponse
      }
    }
    is(sWriteResponse) {
      when(io.host.b.ready) {
        wstate := sWriteAddress
      }
    }
  }

  for(i <- 0 until nSlaveReg){
    when(io.host.w.fire){
      when(i.U===slaveRegWriteSelect){
        slaveReg(i) := wdata
      }
    }
  }

  when(io.host.aw.fire) { waddr := io.host.aw.bits.addr }

  // Total execution time
  when(io.cr.finish){
    slaveReg(4) := totalTime
    slaveReg(3) := 1.U
    totalTime := 0.U
  }.elsewhen(pulse || totalTime =/= 0.U){
     totalTime := totalTime + 1.U
  }
  io.cr.ecnt.zip(slaveReg.slice(5, slaveReg.length)).foreach{
    case (a,b) =>  b := a 
  }
  

  io.host.aw.ready := wstate === sWriteAddress
  io.host.w.ready := wstate === sWriteData
  io.host.b.valid := wstate === sWriteResponse
  io.host.b.bits.resp := 0.U
}
