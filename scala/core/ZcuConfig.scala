package gcn.core

import chisel3._                       // Chisel3 核心库，_ 表示导入所有成员
import chisel3.util._                  // Chisel3 工具库（Mux、Counter、Queue 等）
import vta.util.config._               // VTA 参数化配置框架（Field、Config、Parameters）
import scala.collection.mutable.HashMap  // 可变 HashMap，用于存储压缩格式↔暂存器数量的映射

/**
 * AccParams: 加速器顶层总参数容器
 * 将所有子模块的配置参数集中管理，通过 AccKey 在 Config 框架中注册
 */
case class AccParams(
    hostParams: AXIParams,   // CPU 控制端 AXI-Lite 总线参数（32-bit数据，16-bit地址）
    crParams: CRParams,      // 控制寄存器（CR）参数
    memParams: AXIParams,    // DDR 内存端 AXI-Full 总线参数（512-bit数据，64-bit地址）
    meParams: MEParams,      // 内存引擎（ME）参数：读客户端数、队列深度等
    coreParams: CoreParams   // 核心加速器参数：并行度、SRAM容量、Group数量等
)

/**
 * CRParams: 控制寄存器（Control Register）参数
 * CR 模块实现了 AXI-Lite 从设备，CPU 通过它配置/启动加速器并读取状态
 */
case class CRParams(
  val regBits : Int = 32,    // 单个寄存器的位宽，固定 32-bit
  val nMmapReg : Int = 2     // 内存映射寄存器数量（需要 CPU 在上电时初始化的寄存器）
                              // 这两个寄存器分别是：指令基地址、指令数量
) {
  val nPEEventCtr: Int = 6
    // 每个 PE 内部有 6 个事件计数器，用于性能剖析
    // 对应流水线各阶段：D1耗时、D2耗时、MAC耗时 等

  val nComputeEventCtr: Int = (CoreParams().nPE * nPEEventCtr) + 1
    // Compute 模块总事件计数器数 = 8个PE × 6个计数器/PE + 1个Compute总时间寄存器
    // 仅在性能调试模式下使用，当前被注释掉未启用以节省资源

  // val nEventCtr: Int = nComputeEventCtr + 2
  //  注释掉的完整事件计数：((D1,D2,MAC,Total_PE)*nPE + compute) + load + store
  //  即：PE内各阶段计数器 + Compute总时 + Load总时 + Store总时

  val nEventCtr: Int = 1
    // 当前实际启用的事件计数器数量 = 1
    // 仅保留总执行时间这一个计数器，简洁模式

  val nSlaveReg: Int = nEventCtr + nMmapReg + 3
    // AXI-Lite 从设备总寄存器数 = 1(事件计数) + 2(MMAP寄存器) + 3(额外寄存器)
    // = 6 个寄存器
    // 寄存器地址映射：
    //   0x00 ← 启动/控制寄存器（额外寄存器#1）
    //   0x04 ← 指令基地址（MMAP寄存器#1）
    //   0x08 ← 指令数量（MMAP寄存器#2）
    //   0x0C ← 完成标志（额外寄存器#2）
    //   0x10 ← 总执行时间（事件计数器）
    //   0x14 ← 保留（额外寄存器#3）

  require(nMmapReg < nSlaveReg,
    "memory mapped registers should be atleast 1 less than slave register")
    // 安全检查：MMAP寄存器数量必须 < 总寄存器数
    // 因为总寄存器还包含了启动/完成标志等非MMAP寄存器
}

/**
 * CoreParams: 核心加速器参数 — 定义了整个 GCN 加速器最关键的硬件配置
 * 这些参数直接影响片上 SRAM 大小、并行度和计算吞吐量
 */
case class CoreParams(
  val loadInstQueueEntries: Int = 1,
    // Load 指令队列深度 = 1
    // Fetch 模块将解码后的 Load 指令排入此队列，Load 模块逐条取出执行
    // 设为 1 意味着 Load 指令完全串行执行

  val computeInstQueueEntries: Int = 1,
    // Compute 指令队列深度 = 1
    // Fetch 模块将解码后的 Compute 指令排入此队列，Compute 模块逐条取出执行

  val peOutputScratchQueueEntries: Int = 10,
    // PE 输出暂存队列深度 = 10
    // Compute 模块中，各 Group 的计算结果在写回 OutputScratchpad 前
    // 先经过此 Queue 缓存，防止写端口阻塞导致流水线 stall

  val loadDataQueueEntries: Int = 10,
    // Load 数据队列深度 = 10
    // ME（内存引擎）读回的 DDR 数据先进入此 Queue 缓存
    // 再由 Load 模块写入 GlobalBuffer，解耦 DDR 读取和 SRAM 写入的速率差异

  val Compression: String = "CSR",
    // 稀疏矩阵压缩格式，当前仅支持 "CSR"（Compressed Sparse Row）
    // 决定了需要哪些暂存器（Val/Col/Ptr/Den/Out/Psum 共6个 → nScratchPadMem=5+1）
    // 如果设为 "None"，则只需 2 个暂存器（输入/输出）

  val scratchColSize: Int = 1024*8*10,
    // 列索引暂存器（spCol）容量 = 1024×8×10 = 81,920 个元素
    // 存储在 Scratchpad 中，用于存放 CSR 格式的 column index 数组

  val scratchDenSize: Int = 1024*8*1024,
    // 稠密矩阵暂存器（spDen）容量 = 1024×8×1024 = 8,388,608 个元素
    // 这是所有暂存器中最大的一块，存储稠密特征矩阵（每行 8×32=256bit）
    // 32 个 Group 共享 → 每个 Group 约 256K 元素

  val scratchValSize: Int = 1024*8*10,
    // 稀疏值暂存器（spVal）容量 = 1024×8×10 = 81,920 个元素
    // 存储 CSR 格式中非零元素的数值

  val scratchPtrSize: Int = 1024*8*10,
    // 行指针暂存器（spPtr）容量 = 1024×8×10 = 81,920 个元素
    // 存储 CSR 格式的 row pointer 数组（每行起始偏移）

  val globalBufferSize: Int = 1024*8*1024*128,
    // 全局缓冲区（GlobalBuffer）容量 = 1024×8×1024×128 ≈ 1,073M 个元素
    // 这是 Core 级的统一输入缓冲区，Load 模块从 DDR 加载数据写入此处
    // Compute 模块从此处读取 CSR 数据并分发给各 Group
    // 注意：这里的"元素"指 32-bit 单位，实际硬件中 SRAM 地址映射不同

  val nColInDense: Int = 8,
    // 稠密矩阵的列并行度 = 8
    // 每个 Group 在一个周期内同时处理稠密矩阵的 8 列，即 8 路 MAC 并行
    // 对应 bankBlockSize = 8×32 = 256bit

  val blockSize: Int = 32,
    // 数据块基本位宽 = 32-bit
    // 对应单精度浮点数（FP32）或 32-bit 定点数
    // 所有 SRAM 的读写都以 blockSize 为原子单位

  val nGroups: Int = 32
    // 并行 Group 数量 = 32
    // 32 个 Group 同时处理稀疏矩阵的不同行范围
    // 总 MAC 吞吐 = nGroups × nColInDense = 32×8 = 256 MAC/周期
) {
  // ── 以下为推导参数（derived parameters），根据上述配置自动计算 ──

  val nPE: Int = nColInDense
    // PE（Processing Element）数量 = 列并行度 = 8
    // 每个 Group 内有 8 个并行 PE，对应 nColInDense 路 MAC
    // 用于事件计数器计算

  val bankBlockSize: Int = nColInDense * blockSize
    // Bank 块大小 = 8 × 32 = 256-bit
    // Scratchpad 的每个 bank 一次读写 256-bit
    // 这正好是一个 Group 的稠密矩阵一行中 8 列的数据

  private val ScratchPadMap: HashMap[String, Int] =
    HashMap(("CSR", 5), ("None", 2))
    // 压缩格式 → 所需暂存器数量 的映射表
    // CSR 格式需要 5 种基础暂存器：Val、Col、Ptr、Den、Out（再加 Psum 共 6 个）
    // None（无压缩）只需要 2 种：输入、输出

  var scratchSizeMap: HashMap[String, Int] = HashMap(("None", 0))
    // 每种暂存器类型 → 容量大小 的映射表，默认无压缩为空

  if (Compression == "CSR") {
    // 当采用 CSR 格式时，配置 6 种暂存器及其容量
    scratchSizeMap =
      HashMap(
        ("Col", scratchColSize),   // 列索引暂存器容量
        ("Val", scratchValSize),   // 稀疏值暂存器容量
        ("Ptr", scratchPtrSize),   // 行指针暂存器容量
        ("Den", scratchDenSize),   // 稠密特征矩阵暂存器容量
        ("Out", scratchDenSize),   // 输出暂存器容量（与 Den 相同）
        ("Psum", scratchDenSize)   // 部分和暂存器容量（与 Den 相同）
      )
  }

  val nScratchPadMem = ScratchPadMap(Compression)
    // 当前压缩格式所需的暂存器种类数（CSR → 5 种基础类型）
}

/**
 * MEParams: 内存引擎（Memory Engine）参数
 * ME 模块是多路复用器，让 Core 的多个读写客户端共享一条 AXI-Full 总线访问 DDR
 */
case class MEParams
  (val nReadClients: Int = 2,
    // AXI 读客户端数量 = 2
    // 分别对应：Fetch（读取指令）和 Load（读取 CSR 数据）
    // 它们通过优先级仲裁器时分复用同一 AXI 读通道

    val nWriteClients: Int = 1,
    // AXI 写客户端数量 = 1（仅 Store 模块写回结果）
    // 一次只有一个模块写 DDR，不需要仲裁

    val clientBits: Int = 3,
    // 客户端 ID 的位宽 = 3-bit，可区分最多 8 个客户端

    val RequestQueueDepth: Int = 16,
    // AXI 请求队列深度 = 16
    // ME 内部缓存最多 16 个 outstanding AXI 读请求

    val meParams: Int = 18,
    // ME 内部参数，用于 tag 匹配逻辑的位宽常数

    val clientCmdQueueDepth: Int = 1,
    // 每个客户端的命令队列深度 = 1
    // Fetch/Load 向 ME 发送读请求的队列，深度为 1 意味着背靠背请求会阻塞

    val clientTagBitWidth: Int = 21,
    // Tag 位宽 = 21-bit
    // 每个 AXI 读请求携带一个 tag，ME 用 tag 匹配返回数据与原始请求
    // 21-bit 可区分 2M 个 outstanding 请求

    val clientDataQueueDepth: Int = 16
    // 客户端数据返回队列深度 = 16
    // AXI 读数据返回后暂存在此队列，再由客户端（Fetch/Load）取走
    // 深度 16 与 RequestQueueDepth 匹配，确保数据不会丢失
  ) {

  val RequestQueueMaskBits: Int = RequestQueueDepth.toInt
    // 请求队列掩码位宽 = 16（与队列深度相同，用于取模/索引计算）

  require(nReadClients > 0,
    "nReadClients must be larger than 0")
    // 安全检查：至少需要 1 个读客户端（否则无法从 DDR 读取指令和数据）

  require(nWriteClients == 1,
    "nWriteClients must be 1, only one-write-client support")
    // 安全检查：当前实现仅支持单写客户端
    // 多写客户端需要额外的写仲裁逻辑，本设计中 Store 是唯一的写源
}

/**
 * AccKey: 加速器参数在 Config 框架中的注册键（Key）
 * 这是一个单例对象，类型为 Field[AccParams]
 * 其他模块通过 `p(AccKey)` 查找此键来获取 AccParams 实例
 */
case object AccKey extends Field[AccParams]

/**
 * ZcuConfig: Xilinx UltraScale+ zcu106 开发板的硬件配置类
 * 继承自 VTA Config 框架，使用偏函数（PartialFunction）方式注册参数
 *
 * 偏函数参数说明：
 *   - site: 请求参数的调用位置（View）
 *   - here: 当前 Config 层本身
 *   - up:   Config 链中的上一层（支持参数覆写）
 */
class ZcuConfig extends Config((site, here, up) => {
  // 当有人查找 AccKey 时，返回以下 AccParams 实例
  case AccKey =>
    AccParams(
      // ── CPU 控制接口（AXI-Lite，低带宽控制通路）──
      hostParams = AXIParams(
        coherent = false,   // 非一致性总线（不需要 cache 一致性协议）
        addrBits = 16,      // 地址位宽 = 16-bit，寻址空间 64KB（足够覆盖 CR 寄存器）
        dataBits = 32,      // 数据位宽 = 32-bit（与 CPU 寄存器位宽一致）
        lenBits  = 8,       // 突发长度位宽 = 8-bit（最大支持 256 beat 的突发传输）
        userBits = 1        // 用户自定义信号位宽 = 1-bit（AXI4 协议保留字段）
      ),

      // ── 控制寄存器参数（使用默认值）──
      crParams = CRParams(),
        // regBits=32, nMmapReg=2 → 6 个 AXI-Lite 寄存器

      // ── 核心加速器参数（使用默认值）──
      coreParams = CoreParams(),
        // nGroups=32, nColInDense=8, blockSize=32
        // → 256 MAC/周期峰值吞吐

      // ── DDR 内存接口（AXI-Full，高带宽数据通路）──
      memParams = AXIParams(
        coherent = false,   // 非一致性总线
        addrBits = 64,      // 地址位宽 = 64-bit，支持全 64-bit 物理地址空间
        dataBits = 512,     // 数据位宽 = 512-bit（zcu106 的 AXI HP 端口最大位宽）
                            // 单次传输可搬运 16 个 32-bit 元素，极大提升 DDR 带宽效率
        lenBits  = 8,       // 突发长度位宽 = 8-bit
        userBits = 1        // 用户自定义信号位宽 = 1-bit
      ),

      // ── 内存引擎参数（使用默认值）──
      meParams = MEParams()
        // nReadClients=2, nWriteClients=1, RequestQueueDepth=16
    )
})
