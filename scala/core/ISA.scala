package gcn.core

import chisel3._                       // Chisel3 核心库，提供 UInt、Bool、BitPat 等类型
import chisel3.util._                  // Chisel3 工具库
import scala.collection.mutable.HashMap // 可变 HashMap，用于构建指令字段映射表

/**
 * ISAConstants: 指令集架构常量 (trait)
 *
 * 这是一个 Scala trait（类似 Java 的接口，但可以有具体实现），
 * 定义了指令解码时所需的所有位宽常量和操作码。
 * 下游模块通过 `with ISAConstants` 混入此 trait 来使用这些常量。
 *
 * 注意：此 trait 与下方的 `object ISA` 存在部分字段重复定义，
 * 两者提供相同的常量值供不同场景使用。
 */
trait ISAConstants {
  val INST_BITS = 256
    // 指令总位宽 = 256-bit
    // 每条指令占用 32 字节，从 DDR 中 Fetch 读取时需要 512-bit AXI 传输 0.5 beat

  val OP_BITS = 2
    // 操作码（Opcode）位宽 = 2-bit
    // 可编码 4 种操作：Load(0)、Store(1)、Compute(2)、Reserved(3)

  val M_ID_BITS = 3
    // 内存类型 ID 位宽 = 3-bit
    // 用于区分 Load/Store 指令操作的内存区域：
    //   000=Col(列索引), 001=Ptr(行指针), 010=Val(稀疏值),
    //   011=Den(稠密矩阵), 100=Out(输出), 101=Psum(部分和)

  val M_DRAM_OFFSET_BITS = 64
    // DDR 地址偏移量位宽 = 64-bit
    // Load/Store 指令中，DDR 端的基地址偏移（相对于指令基址寄存器）

  val M_SRAM_OFFSET_BITS = 32
    // SRAM 地址偏移量位宽 = 32-bit
    // Load/Store 指令中，片上 SRAM 的地址偏移量

  val M_XSIZE_BITS = 32
    // Load/Store 指令中 X 维度尺寸位宽 = 32-bit
    // 表示一次 Load/Store 操作要搬运的数据量

  val M_YSIZE_BITS = 0
    // Load/Store 指令中 Y 维度尺寸位宽 = 0（当前未使用）
    // 保留字段，当前所有 load/store 都是一维操作

  val C_SRAM_OFFSET_BITS = 32
    // Compute 指令中 SRAM 偏移量位宽 = 32-bit
    // SpMM 计算指令中，各暂存器的起始地址偏移量

  val C_XSIZE_BITS = 32
    // Compute 指令中 X 维度尺寸位宽 = 32-bit
    // SpMM 指令中稀疏矩阵的行数（或非零元素数）

  val C_YSIZE_BITS = 32
    // Compute 指令中 Y 维度尺寸位宽 = 32-bit
    // SpMM 指令中稠密矩阵的列数

  val C_PSUM_BITS = 2
    // Compute 指令中部分和（Psum）标识位宽 = 2-bit
    // 用于标识是否需要加载之前的部分和（VR跨行聚合场景）

  val Y = true.B
    // 布尔常量 True，用于表示"是"（如 pr_valid=Y 表示需要跨组聚合）

  val N = false.B
    // 布尔常量 False，用于表示"否"

  val OP_L = 0.asUInt(OP_BITS.W)
    // 操作码：Load = 0（从 DDR 加载数据到 SRAM）

  val OP_S = 1.asUInt(OP_BITS.W)
    // 操作码：Store = 1（将 SRAM 数据写回 DDR）

  val OP_C = 2.asUInt(OP_BITS.W)
    // 操作码：Compute = 2（执行 SpMM 稀疏矩阵乘法）

  val OP_X = 3.asUInt(OP_BITS.W)
    // 操作码：Reserved = 3（保留/未使用，可扩展）
}


/**
 * ISA: 指令集架构定义 (object)
 *
 * 这是一个 Scala 单例对象，是整个加速器自定义指令集的唯一定义处。
 * 主要功能：
 *   1. 定义指令的 256-bit 编码格式（BitPat 模式匹配）
 *   2. 提供 Load/Store/SpMM 三种指令的位域布局
 *   3. 生成 BitPat 常量供 Decode 模块做模式匹配解码
 *
 * 指令编码总览（256-bit，从 LSB 到 MSB）：
 *   ┌──────────────┬────────────┬────────────┐
 *   │  Don't Care  │  Mem ID   │  Task ID   │
 *   │   (高位)     │  (3-bit)  │  (2-bit)   │
 *   └──────────────┴────────────┴────────────┘
 *    bit[255:5]     bit[4:2]    bit[1:0]
 *
 *  Task ID (bit[1:0])：
 *    00 = Load, 01 = Store, 10 = SpMM, 11 = Finish
 *
 *  Mem ID (bit[4:2])，仅 Load/Store 指令有效：
 *    000 = Col(列索引), 001 = Ptr(行指针), 010 = Val(稀疏值)
 *    011 = Den(稠密矩阵), 100 = Out(输出), 101 = Psum(部分和)
 */
object ISA {
  // ── 指令位宽常量（与 trait ISAConstants 功能重复，供 object 内部独立使用）──

  val INST_BITS = 256
    // 指令总位宽 = 256-bit

  val OP_BITS = 2
    // 操作码位宽 = 2-bit

  val M_ID_BITS = 3
    // 内存类型 ID 位宽 = 3-bit（六种内存类型：col/ptr/val/den/out/psum）

  val M_DRAM_OFFSET_BITS = 64
    // DDR 偏移量位宽 = 64-bit

  val M_SRAM_OFFSET_BITS = 32
    // SRAM 偏移量位宽 = 32-bit

  val M_XSIZE_BITS = 32
    // Load/Store 数据量 (X 维度) 位宽 = 32-bit

  val M_YSIZE_BITS = 0
    // Load/Store 数据量 (Y 维度) 位宽 = 0（保留未使用）

  val C_SRAM_OFFSET_BITS = 32
    // Compute 指令 SRAM 偏移量位宽 = 32-bit

  val C_XSIZE_BITS = 32
    // Compute 指令 X 维度尺寸位宽 = 32-bit

  val C_YSIZE_BITS = 32
    // Compute 指令 Y 维度尺寸位宽 = 32-bit

  val C_PR_BITS = 2
    // Compute 指令"前一行有效"标识位宽 = 2-bit
    // 用于跨 Group 的 VR（Virtual Row）聚合判断

  val Y = true.B
    // 布尔常量 True（用于 pr_valid 等标志位）

  val N = false.B
    // 布尔常量 False

  val OP_L = 0.asUInt(OP_BITS.W)
    // 操作码 Load  = 0 (二进制 00)

  val OP_S = 1.asUInt(OP_BITS.W)
    // 操作码 Store = 1 (二进制 01)

  val OP_C = 2.asUInt(OP_BITS.W)
    // 操作码 SpMM  = 2 (二进制 10)

  val OP_X = 3.asUInt(OP_BITS.W)
    // 操作码保留 = 3 (二进制 11)

  // ── 私有辅助字段与方法 ──

  private val xLen = 256
    // 指令总位宽的内部别名，用于拼接 BitPat 字符串时计算 dontCare 位数

  private val idBits: HashMap[String, Int] =
    HashMap(("task", 2), ("mem", 2))
    // 指令低位标识字段的位宽表
    //   "task" → 2-bit（操作类型：load/store/spmm/finish）
    //   "mem"  → 2-bit（内存类型：col/ptr/val/den/out/psum）
    // ⚠ 此处声明 mem=2-bit，但 memId 实际使用 3-bit 编码（见下方），
    //   运行时通过 idBits("mem") 计算 dontCare 高位时可能产生偏移，需注意

  private val taskId: HashMap[String, String] =
    HashMap(
      ("load",   "00"),  // Load 指令   → 二进制编码 00
      ("store",  "01"),  // Store 指令  → 二进制编码 01
      ("spmm",   "10"),  // SpMM 指令   → 二进制编码 10
      ("finish", "11")   // Finish 指令 → 二进制编码 11（通知 Core 结束执行）
    )
    // 操作类型（Task）到 2-bit 二进制码的映射表

  private val memId: HashMap[String, String] =
    HashMap(
      ("col",  "000"),  // 列索引暂存器   (spCol)
      ("ptr",  "001"),  // 行指针暂存器   (spPtr)
      ("val",  "010"),  // 稀疏值暂存器   (spVal)
      ("den",  "011"),  // 稠密矩阵暂存器  (spDen)
      ("out",  "100"),  // 输出暂存器     (spOut)
      ("psum", "101")   // 部分和暂存器   (spPsum)
    )
    // 内存类型（Mem）到 3-bit 二进制码的映射表
    // 与 M_ID_BITS=3 一致，但 idBits("mem") 声明为 2 需注意

  private def dontCare(bits: Int): String = "?" * bits
    // 生成指定数量的 "?" 字符串（Chisel BitPat 中的通配符）
    // 例如：dontCare(3) → "???"（匹配任意 3-bit 值）

  private def instPat(bin: String): BitPat = BitPat("b" + bin)
    // 将二进制字符串转换为 Chisel BitPat（位模式）对象
    // 例如：instPat("0??01") → BitPat("b0??01")
    // BitPat 用于 Decode 模块的指令模式匹配（类似 case 语句）

  private def load(id: String): BitPat = {
    val rem = xLen - idBits("mem") - idBits("task")
      // rem = 256 - 2 - 2 = 252
      // 高位 don't care 的位数（即指令中未参与解码的无关位）
    val inst = dontCare(rem) + memId(id) + taskId("load")
      // 拼接完整 256-bit 指令模式（字符串级别）
      // 格式：{252-bit 无关} + {3-bit Mem ID} + {2-bit 00(load)}
    instPat(inst)
      // 转为 Chisel BitPat 返回，供 Decode 模块做模式匹配
  }

  private def store(id: String): BitPat = {
    val rem = xLen - idBits("mem") - idBits("task")
      // rem = 256 - 2 - 2 = 252
    val inst = dontCare(rem) + memId(id) + taskId("store")
      // 格式：{252-bit 无关} + {3-bit Mem ID} + {2-bit 01(store)}
    instPat(inst)
  }

  private def spmm: BitPat = {
    val rem = xLen - idBits("task")
      // rem = 256 - 2 = 254
      // SpMM 指令不需要 memId 字段，所以只减去 task 的 2-bit
    val inst = dontCare(rem) + taskId("spmm")
      // 格式：{254-bit 无关} + {2-bit 10(spmm)}
    instPat(inst)
  }

  // ── 公开的指令 BitPat 常量 ──
  // Decode 模块通过 ListLookup 匹配这些 BitPat 来确定指令类型
  // 例如：Decode 模块中 ListLookup(io.inst, List(N, N, N), Array(LCOL -> ...))

  def LCOL = load("col")
    // Load Col 指令：MemID=000, Task=00 → bit[4:0]="00000"
    // 含义：从 DDR 加载列索引数据到 spCol 暂存器

  def LPTR = load("ptr")
    // Load Ptr 指令：MemID=001, Task=00 → bit[4:0]="00100"
    // 含义：从 DDR 加载 CSR 行指针数据到 spPtr 暂存器

  def LVAL = load("val")
    // Load Val 指令：MemID=010, Task=00 → bit[4:0]="01000"
    // 含义：从 DDR 加载稀疏值数据到 spVal 暂存器

  def LDEN = load("den")
    // Load Den 指令：MemID=011, Task=00 → bit[4:0]="01100"
    // 含义：从 DDR 加载稠密矩阵数据到 spDen 暂存器

  def LOUT = load("out")
    // Load Out 指令：MemID=100, Task=00 → bit[4:0]="10000"
    // 含义：从 DDR 加载之前的输出（用于跨层残差或初始化）

  def LPSUM = load("psum")
    // Load Psum 指令：MemID=101, Task=00 → bit[4:0]="10100"
    // 含义：从 DDR 加载上次计算的部分和（用于 VR 跨组聚合场景）

  def SPMM = spmm
    // SpMM 计算指令：Task=10 → bit[1:0]="10"
    // 含义：执行稀疏矩阵×稠密矩阵乘法（Sparse Matrix-Dense Matrix Multiply）

  def SOUT = store("out")
    // Store Out 指令：MemID=100, Task=01 → bit[4:0]="10001"
    // 含义：将输出暂存器结果写回 DDR

  // ── 暂存器 ID 映射 ──
  // 用于 Compute.scala 中按 ID 访问不同暂存器的数据
  val scratchID =
      HashMap(("Col", 8), ("Val", 1), ("Ptr", 4), ("Den", 2))
    // 四种 CSR 数据暂存器的 ID 编码：
    //   Col(列索引) → ID=8
    //   Val(稀疏值) → ID=1
    //   Ptr(行指针) → ID=4
    //   Den(稠密矩阵)→ ID=2
    // 这些 ID 用于 Load 阶段循环加载 4 种 CSR 数据时的分发路由

}
