package gcn.core

import chisel3._
import chisel3.util._
import vta.util.config._

// /** Simple Processing Element.
//  *
//  * MAC
//  */
class SimplePE(debug: Boolean = false)(implicit p: Parameters) extends Module with ISAConstants{
  val mp = p(AccKey).memParams
  val cp = p(AccKey).coreParams
  val regBits = p(AccKey).crParams.regBits 
  val io = IO(new Bundle {
    val a = Input(UInt(cp.blockSize.W))
    val b = Input(UInt(cp.blockSize.W))
    val c = Input(UInt(cp.blockSize.W))
    val out = Output(UInt(cp.blockSize.W))
  })
  io.out := (io.a * io.b) + io.c
}
