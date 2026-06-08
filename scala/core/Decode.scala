package gcn.core

import chisel3._
import chisel3.util._

import  ISA._

/** MemDecode.
 *
 * Decode memory instructions with a Bundle. This is similar to an union,
 * therefore order matters when declaring fields. These are the instructions
 * decoded with this bundle:
 *   - LUOP
 *   - LWGT
 *   - LINP
 *   - LACC
 *   - SOUT
 */
class MemDecode extends Bundle{
  val empty = UInt(123.W)
  val ysize = UInt(M_YSIZE_BITS.W)
  val xsize = UInt(M_XSIZE_BITS.W)
  val sram_offset = UInt(M_SRAM_OFFSET_BITS.W)
  val dram_offset = UInt(M_DRAM_OFFSET_BITS.W)
  val id = UInt(M_ID_BITS.W)
  val op = UInt(OP_BITS.W)
}

/** SpMMDecode.
 *
 * Decode GEMM instruction with a Bundle. This is similar to an union,
 * therefore order matters when declaring fields.
 */
class SpMMDecode extends Bundle {

  val empty = UInt(41.W)
  val pr_valid = UInt(C_PR_BITS.W)
  val row_size = UInt(C_YSIZE_BITS.W)
  val col_size = UInt(C_YSIZE_BITS.W)
  val den_size = UInt(C_XSIZE_BITS.W)
  val sram_offset_val = UInt(C_SRAM_OFFSET_BITS.W)
  val sram_offset_den = UInt(C_SRAM_OFFSET_BITS.W)
  val sram_offset_ptr = UInt(C_SRAM_OFFSET_BITS.W)
  val sram_offset_col = UInt(C_SRAM_OFFSET_BITS.W)
  val op = UInt(OP_BITS.W)
}

// /** AluDecode.
//  *
//  * Decode ALU instructions with a Bundle. This is similar to an union,
//  * therefore order matters when declaring fields. These are the instructions
//  * decoded with this bundle:
//  *   - VMIN
//  *   - VMAX
//  *   - VADD
//  *   - VSHX
//  */
// class AluDecode extends Bundle {
//   val alu_imm = UInt(C_ALU_IMM_BITS.W)
//   val alu_use_imm = Bool()
//   val alu_op = UInt(C_ALU_OP_BITS.W)
//   val src_1 = UInt(C_AIDX_BITS.W)
//   val src_0 = UInt(C_AIDX_BITS.W)
//   val dst_1 = UInt(C_AIDX_BITS.W)
//   val dst_0 = UInt(C_AIDX_BITS.W)
//   val empty_0 = Bool()
//   val lp_1 = UInt(C_ITER_BITS.W)
//   val lp_0 = UInt(C_ITER_BITS.W)
//   val uop_end = UInt(C_UOP_END_BITS.W)
//   val uop_begin = UInt(C_UOP_BGN_BITS.W)
//   val reset = Bool()
//   val push_next = Bool()
//   val push_prev = Bool()
//   val pop_next = Bool()
//   val pop_prev = Bool()
//   val op = UInt(OP_BITS.W)
// }

// /** UopDecode.
//  *
//  * Decode micro-ops (uops).
//  */
// class UopDecode extends Bundle {
//   val u2 = UInt(10.W)
//   val u1 = UInt(11.W)
//   val u0 = UInt(11.W)
// }

/** FetchDecode.
 *
 * Partial decoding for dispatching instructions to Load, Compute, and Store.
 */
class FetchDecode extends Module with ISAConstants{
  val io = IO(new Bundle {
    val inst = Input(UInt(INST_BITS.W))
    val isLoad = Output(Bool())
    val isCompute = Output(Bool())
    val isStore = Output(Bool())
  })
  val csignals =
    ListLookup(
      io.inst,
      List(N, OP_X),
      Array(
        LCOL -> List(Y, OP_L),
        LPTR -> List(Y, OP_L),
        LVAL -> List(Y, OP_L),
        LDEN -> List(Y, OP_L),
        LPSUM -> List(Y, OP_L),
        SPMM -> List(Y, OP_C),
        SOUT -> List(Y, OP_S)
      )
    )

  val (cs_val_inst: Bool) :: cs_op_type :: Nil = csignals

  io.isLoad := (cs_val_inst && cs_op_type === OP_L)
  io.isCompute := (cs_val_inst & cs_op_type === OP_C)
  io.isStore := (cs_val_inst & cs_op_type === OP_S)
}

/** LoadDecode.
 *
 * Decode dependencies, type and sync for Load module.
 */
class LoadDecode extends Module with ISAConstants{
  val io = IO(new Bundle {
    val inst = Input(UInt(INST_BITS.W))
    val isSeq = Output(Bool())
    val isVal = Output(Bool())
    val isCol = Output(Bool())
    val isPtr = Output(Bool())
    val isPsum = Output(Bool())
    val xSize = Output(UInt(M_XSIZE_BITS.W))
    val ySize = Output(UInt(M_YSIZE_BITS.W))
    val dramOffset = Output(UInt(M_DRAM_OFFSET_BITS.W))
    val sramOffset = Output(UInt(M_SRAM_OFFSET_BITS.W))
  })
  val dec = io.inst.asTypeOf(new MemDecode)
  io.isSeq := io.isVal || io.isCol || io.isPtr || io.isPsum
  io.isVal := io.inst === LVAL
  io.isCol := io.inst === LCOL
  io.isPtr := io.inst === LPTR
  io.isPsum := io.inst === LPSUM
  io.xSize := dec.xsize
  io.ySize := dec.ysize
  io.sramOffset := dec.sram_offset
  io.dramOffset := dec.dram_offset
}

/** ComputeDecode.
 *
 * Decode dependencies, type and sync for Compute module.
 */
class ComputeDecode extends Module with ISAConstants{
  val io = IO(new Bundle {
    val inst = Input(UInt(INST_BITS.W))
    val sramVal = Output(UInt(C_SRAM_OFFSET_BITS.W))
    val sramCol = Output(UInt(C_SRAM_OFFSET_BITS.W))
    val sramPtr = Output(UInt(C_SRAM_OFFSET_BITS.W))
    val sramDen = Output(UInt(C_SRAM_OFFSET_BITS.W))
    val denSize = Output(UInt(C_XSIZE_BITS.W))
    val colSize = Output(UInt(C_YSIZE_BITS.W))
    val rowSize = Output(UInt(C_YSIZE_BITS.W))
    val prStart = Output(Bool())
    val prEnd = Output(Bool())
  })
  val dec = io.inst.asTypeOf(new SpMMDecode)
  io.sramVal := dec.sram_offset_val
  io.sramCol := dec.sram_offset_col
  io.sramPtr := dec.sram_offset_ptr
  io.sramDen := dec.sram_offset_den
  io.denSize := dec.den_size
  io.colSize := dec.col_size
  io.rowSize := dec.row_size
  io.prStart := dec.pr_valid(1)
  io.prEnd := dec.pr_valid(0)
}

// /** StoreDecode.
//  *
//  * Decode dependencies, type and sync for Store module.
//  */
// class StoreDecode extends Module {
//   val io = IO(new Bundle {
//     val inst = Input(UInt(INST_BITS.W))
//     val push_prev = Output(Bool())
//     val pop_prev = Output(Bool())
//     val isStore = Output(Bool())
//     val isSync = Output(Bool())
//   })
//   val dec = io.inst.asTypeOf(new MemDecode)
//   io.push_prev := dec.push_prev
//   io.pop_prev := dec.pop_prev
//   io.isStore := io.inst === SOUT & dec.xsize =/= 0.U
//   io.isSync := io.inst === SOUT & dec.xsize === 0.U
// }
class StoreDecode extends Module with ISAConstants{
  val io = IO(new Bundle {
    val inst = Input(UInt(INST_BITS.W))
    val xSize = Output(UInt(M_XSIZE_BITS.W))
    val ySize = Output(UInt(M_YSIZE_BITS.W))
    val dramOffset = Output(UInt(M_DRAM_OFFSET_BITS.W))
    val sramOffset = Output(UInt(M_SRAM_OFFSET_BITS.W))
  })
  val dec = io.inst.asTypeOf(new MemDecode)
  io.xSize := dec.xsize
  io.ySize := dec.ysize
  io.sramOffset := dec.sram_offset
  io.dramOffset := dec.dram_offset
}