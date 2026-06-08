package gcn.core                                                              // 包名：GCN核心模块

import chisel3._                                                              // Chisel3硬件描述语言
import chisel3.util._                                                         // Chisel3工具包（MuxLookup/Cat/Enum等）
import vta.util.config._                                                      // VTA配置框架
import scala.math._                                                            // Scala数学函数库
import gcn.core.util._                                                        // GCN自定义工具包（含ISAConstants/MuxTree等）
// /** Compute.                                                               // 模块文档注释（被注释掉）
//  *                                                                         // 
//  * Takes instructions from fetch module.                                   // 从Fetch模块接收指令
//  * Schedules computation between PEs.                                      // 调度PE之间的计算
//  * Arbitrates communication betwen PE and scratchpads.                     // 仲裁PE与暂存器之间的通信
//  */
class VRTableEntry()(implicit p: Parameters) extends Bundle{                   // VR表条目：记录每个Group的行数和跨组依赖标志
  val mp = p(AccKey).memParams                                                // 内存参数（位宽/深度等）
  val cp = p(AccKey).coreParams                                               // 核心参数（nGroups/blockSize等）
  val nRows = UInt(32.W)                                                       // 该Group包含的行数
  val isVRWithPrevGroup = Bool()                                              // 是否与前一个Group的行有依赖（跨组溢出）
}
class VRTableEntryWithGroup()(implicit p: Parameters) extends Bundle{          // 带Group编号的VR表条目（用于Arbiter仲裁）
  val mp = p(AccKey).memParams                                                // 内存参数
  val cp = p(AccKey).coreParams                                               // 核心参数
  val VRTableEntry = new VRTableEntry                                          // VR表条目内容
  val group = UInt(cp.nGroups.W)                                              // 该条目所属的Group编号
}

class Compute(debug: Boolean = false)(implicit p: Parameters) extends Module with ISAConstants{ // Compute模块：核心计算调度器
  val mp = p(AccKey).memParams                                                // 获取内存参数配置
  val cp = p(AccKey).coreParams                                               // 获取核心参数配置
  val cr = p(AccKey).crParams                                                 // 获取控制寄存器参数
  val regBits = p(AccKey).crParams.regBits                                    // 寄存器位宽
  val io = IO(new Bundle {                                                    // Compute模块IO端口定义
    val inst = Flipped(Decoupled(UInt(INST_BITS.W)))                           // 从Fetch模块接收指令（Flipped：输入方向）
    val gbReadCmd = Output(new SPReadCmd)                                      // 向GlobalBuffer发送读命令
    val gbReadData = Input(new SPReadData(scratchType = "Global"))             // 从GlobalBuffer接收读回的数据
    val spOutWrite = Decoupled(new SPWriteCmd)                                 // 向OutputScratchpad写入最终结果
    val valid = Input(Bool())                                                  // 外部valid信号（控制模块启动）
    val done = Output(Bool())                                                  // 计算完成标志
  })
  val bankBlockSizeBytes = cp.bankBlockSize/8                                 // Bank块大小（从bit转Byte）
  val denseLoaded = RegInit(false.B)                                          // 稠密矩阵是否已加载标志（首次加载后置true）
  val computeTimeOut = RegInit(0.U(32.W))                                      // 首次计算的超时周期数（用于确定固定计算时间）
  val computeTimer = RegInit(0.U(32.W))                                        // 后续计算的实际计时器
  val computeSkipCount = RegInit(0.U(32.W))                                   // 计算超时时跳过的计算次数计数器
  dontTouch(computeSkipCount)                                                  // 标记computeSkipCount不被优化器移除（断言用）

  // Module instantiation
  val inst_q = Module(new Queue(UInt(INST_BITS.W), cp.computeInstQueueEntries)) // 指令FIFO队列（缓冲来自Fetch的指令）
  val dec = Module(new ComputeDecode)                                          // 指令解码器模块
  val vRTable = SyncReadMem(cp.nGroups, new VRTableEntry)                      // VR表：同步读存储器，存储nGroups个VRTableEntry
  val vRTableReadGroup_q = RegInit(0.U(log2Ceil(cp.nGroups).W))                // VR表读Group索引寄存器（当前读取哪个Group）
  val vRTableReadGroup = Wire(chiselTypeOf(vRTableReadGroup_q))                // VR表读Group索引组合逻辑线
  val vRTableReadData = vRTable.read(vRTableReadGroup, true.B)                 // 从VR表读取数据（true.B = 使能读）
  val groupArray = for(i <- 0 until cp.nGroups) yield {                        // 生成nGroups个Group计算模块实例
    Module(new Group(groupID = i))                                            // 每个Group模块有自己的ID
  }

val vrArbiter = Module(new Arbiter(new VRTableEntryWithGroup, cp.nGroups))    // VR表写入仲裁器：多个Group竞争写入VR表
vrArbiter.io.out.ready := true.B                                              // 仲裁器输出端始终ready（立即接收）
when(vrArbiter.io.out.valid){                                                 // 当仲裁器输出有效时
  vRTable.write(vrArbiter.io.out.bits.group,vrArbiter.io.out.bits.VRTableEntry) // 将Group编号和VR表条目写入VR表
}


// state machine
  val sIdle :: sDataMoveRow :: sDataMoveCol :: sDataMoveVal :: sDataMoveDen :: sCompute :: sCombineGroup :: sCombine :: sDone :: Nil = Enum(9) // 9状态枚举：空闲/搬运RowPtr/搬运ColIdx/搬运Val/搬运Den/计算/Combine预检/汇总/完成
  val state = RegInit(sIdle)                                                   // 当前状态寄存器，初始为空闲
  val start = inst_q.io.deq.fire                                              // 启动信号：指令队列出队成功（valid&ready同时为高）
  val inst = RegEnable(inst_q.io.deq.bits, start)                             // 锁存当前指令：启动时采样并保持
  dec.io.inst := Mux(start, inst_q.io.deq.bits, inst)                         // 解码器输入：启动时用新指令，否则用已锁存的指令
  inst_q.io.enq <> io.inst                                                    // 指令队列输入端直连外部inst接口（手牵手valid/ready）
  inst_q.io.deq.ready := (state === sIdle) && io.valid                        // 仅当空闲且外部valid时才从队列取指


  val groupSel = RegInit(0.U(cp.nGroups.W))                                    // 当前选中的Group编号寄存器
  val groupEnd = groupSel === (cp.nGroups - 1).U                               // 是否为最后一个Group（边界检测）
  val nNonZeroPrevTotal = RegInit(0.U(32.W))                                   // 之前累计的非零元素总数
  
  val nNonZeroPerGroup =  dec.io.colSize >> log2Ceil(cp.nGroups)              // 每个Group平均分配的非零元素数 = colSize / nGroups（右移=除）
  val gbAddr = RegInit(0.U(C_SRAM_OFFSET_BITS.W))                              // GlobalBuffer读地址寄存器
  val gbRdata = io.gbReadData.data                                            // GlobalBuffer读回的数据

  // Row Splitting（RowPtr拆分：将CSR的rowPtr数组按Group划分）
  val rowPtrFin = Wire(Bool())                                                 // 当前Group的RowPtr数据搬运完成标志（组合逻辑）
  val rowPtrDataBlock = for(i <- 0 until (cp.bankBlockSize/cp.blockSize))yield{ // 将一个Bank块按blockSize切分成多个数据段
    gbRdata((((i+1)*cp.blockSize) -1), i*cp.blockSize)                        // 提取第i段：位宽为blockSize
  }
  val rowPtrAddr = RegInit(0.U(32.W))                                          // RowPtr读取地址寄存器
  val rowPtrIdxInBlock = rowPtrAddr(log2Ceil(bankBlockSizeBytes)-1,log2Ceil(cp.blockSize/8)) // 从读地址中提取块内索引（Bank块内第几个blockSize条目）
  val rowPtrData = MuxTree(rowPtrIdxInBlock, rowPtrDataBlock)                  // 多路选择：根据块内索引选出对应的rowPtr数据
  val rowPtrReadAddr = Mux(start, dec.io.sramPtr, Mux(rowPtrFin, rowPtrAddr, rowPtrAddr + (cp.blockSize/8).U)) // RowPtr读地址：启动时指向指令指定的起始地址，未完成时递增，完成时保持
  rowPtrFin := rowPtrData >= (nNonZeroPrevTotal + (( groupSel + 1.U) << Log2(nNonZeroPerGroup))) // 完成条件：当前rowPtr值 ≥ 当前Group的非零累积边界
  val rowPtrWriteAddr = RegInit(0.U(C_SRAM_OFFSET_BITS.W))                     // RowPtr写入到Group scratchpad的目标地址寄存器
  when(state === sDataMoveRow){                                                // 仅在Row搬运状态才更新写地址
    when(rowPtrFin){                                                           // 当前Group搬运完成
      rowPtrWriteAddr := 0.U                                                   // 写地址归零（准备下一Group）
    }.otherwise{                                                               // 当前Group搬运未完成
      rowPtrWriteAddr := rowPtrWriteAddr + (cp.blockSize/8).U                  // 写地址递增一个blockSize字节
    }
  }
  // val rowPtrWriteMask = UIntToOH(rowPtrIdxInBlock)                          // 注释掉：未使用的写掩码（一对一映射写法）
  val rowPtrWriteEn = !rowPtrFin && (state === sDataMoveRow)                   // RowPtr写使能：未完成 且 处于Row搬运状态
  val nRowWritten_q = RegInit(0.U(32.W))                                       // 已写入的行数寄存器
  val nRowWritten =  nRowWritten_q + !rowPtrFin                                // 已写入行数 = 寄存器值 + 当前周期是否完成一个rowPtr块
  val nRowWrittenValid = Wire(Bool())                                          // nRowWritten值有效的标志线
  
  // Col Splitting（ColIdx拆分：将CSR的colIdx数组按Group划分）
  val colReadAddr = RegInit(0.U(32.W))                                         // ColIdx从GlobalBuffer的读地址寄存器
  val colWriteAddr = RegInit(0.U(32.W))                                        // ColIdx写入Group scratchpad的目标地址
  val colReadBlockNum = RegInit(cp.nColInDense.U(32.W))                         // 累计读取的Col数据块编号寄存器
  val colFin = (colReadBlockNum >= ((groupSel + 1.U) << Log2(nNonZeroPerGroup))) // Col搬运完成条件：累计块数 ≥ 当前Group的非零边界
  when(state === sDataMoveCol){                                                // 仅在Col搬运状态才更新
    colReadBlockNum := colReadBlockNum + cp.nColInDense.U                      // 每读一个Bank块，累计块数+nColInDense
  }.otherwise{                                                                 // 非Col搬运状态
    colReadBlockNum := cp.nColInDense.U                                        // 重置为单个Bank块对应的Col数
  }
  when((state === sDataMoveCol)){                                              // Col搬运状态下的写地址更新
    when(colFin){                                                              // 当前Group搬运完成
      colWriteAddr := 0.U                                                      // 写地址归零
    }.otherwise{                                                               // 未完成
      colWriteAddr := colWriteAddr + bankBlockSizeBytes.U                       // 写地址递增一个Bank块字节数
    }
  }

  // Val Splitting（Val拆分：将CSR的value数组按Group划分）
  val valReadAddr = RegInit(0.U(32.W))                                         // Val从GlobalBuffer的读地址寄存器
  val valWriteAddr = RegInit(0.U(32.W))                                        // Val写入Group scratchpad的目标地址
  val valReadBlockNum = RegInit(cp.nColInDense.U(32.W))                         // 累计读取的Val数据块编号寄存器
  val valFin = (valReadBlockNum >= ((groupSel + 1.U) << Log2(nNonZeroPerGroup))) // Val搬运完成条件
  when(state === sDataMoveVal){                                                // 仅在Val搬运状态才更新
    valReadBlockNum := valReadBlockNum + cp.nColInDense.U                      // 读取块数递增
  }.otherwise{                                                                 // 非Val搬运状态
    valReadBlockNum := cp.nColInDense.U                                        // 重置
  }
  when((state === sDataMoveVal)){                                              // Val搬运状态下的写地址更新
    when(valFin){                                                              // 当前Group搬运完成
      valWriteAddr := 0.U                                                      // 写地址归零
    }.otherwise{                                                               // 未完成
      valWriteAddr := valWriteAddr + bankBlockSizeBytes.U                       // 写地址递增
    }
  }

  // Den Splitting（Dense拆分：将稠密矩阵按Group拆分）
  val denReadAddr = RegInit(0.U(32.W))                                         // Dense从GlobalBuffer的读地址寄存器
  val denWriteAddr = RegInit(0.U(32.W))                                        // Dense写入Group scratchpad的目标地址
  val denReadBlockNum = RegInit(cp.nColInDense.U(32.W))                         // 累计读取的Dense数据块编号寄存器
  val denFin = (denReadBlockNum >= dec.io.denSize)                              // Dense搬运完成条件：累计块数 ≥ Dense总大小
  when(state === sDataMoveDen){                                                // 仅在Dense搬运状态才更新
    denReadBlockNum := denReadBlockNum + cp.nColInDense.U                      // 读取块数递增
  }.otherwise{                                                                 // 非Dense搬运状态
    denReadBlockNum := cp.nColInDense.U                                        // 重置
  }
  when((state === sDataMoveDen)){                                              // Dense搬运状态下的写地址更新
    when(denFin){                                                              // 搬运完成
      denWriteAddr := 0.U                                                      // 写地址归零
    }.otherwise{                                                               // 未完成
      denWriteAddr := denWriteAddr + bankBlockSizeBytes.U                       // 写地址递增
    }
  }

// group select（Group选择与切换逻辑）
  when((((state === sDataMoveRow) && rowPtrFin)||(state === sDataMoveCol) && colFin)||((state === sDataMoveVal) && valFin)){ // 当Row/Col/Val任一完成当前Group时
    when(groupEnd){                                                            // 已经是最后一个Group
      groupSel := 0.U                                                          // 回到Group0（循环）
      nRowWritten_q := 0.U                                                     // 重置已写行数
    }.otherwise{                                                               // 不是最后一个Group
      groupSel := groupSel + 1.U                                              // 切换到下一个Group
      nRowWritten_q := 0.U                                                     // 重置已写行数
    }
  }.elsewhen((state === sDataMoveRow)){                                        // Row搬运状态但尚未完成
    nRowWritten_q := nRowWritten                                              // 持续更新已写行数
  }
  nRowWrittenValid := ((state === sDataMoveRow) && rowPtrFin)                  // nRowWritten有效：Row搬运状态且完成
  
  io.gbReadCmd.addr := MuxLookup(true.B,                                       // GlobalBuffer读地址多路选择器
      rowPtrReadAddr, // default                                               // 默认：RowPtr读地址
      Array(
        ((state === sIdle) && start)-> rowPtrReadAddr,                         // 空闲且启动→读RowPtr
        ((state === sDataMoveRow) && (!(rowPtrFin && groupEnd))) -> rowPtrReadAddr, // Row搬运未全部完成→继续读RowPtr
        ((state === sDataMoveRow) && (rowPtrFin && groupEnd)) -> colReadAddr,  // Row完成且最后一个Group→切换到读Col
        ((state === sDataMoveCol) && !(colFin && groupEnd)) -> colReadAddr,    // Col搬运未全部完成→继续读Col
        ((state === sDataMoveCol) && (colFin && groupEnd)) -> valReadAddr,     // Col完成且最后一个Group→切换到读Val
        ((state === sDataMoveVal) && !(valFin && groupEnd)) -> valReadAddr,    // Val搬运未全部完成→继续读Val
        ((state === sDataMoveVal) && (valFin && groupEnd)) -> denReadAddr,     // Val完成且最后一个Group→切换到读Dense
        ((state === sDataMoveDen)) -> denReadAddr                              // Dense搬运→读Dense
      ))


// Partial outputs aggregate（部分输出聚合：将各Group计算结果汇总）
val outRowCount_q = RegInit(0.U(32.W))                                         // 已输出的总行数寄存器
val aggDone = vRTableReadGroup_q === (cp.nGroups - 1).U                        // 聚合完成：所有Group都已处理完毕
val currRowInGroup_q = RegInit(0.U(32.W))                                      // 当前Group内已处理的行数寄存器
val currRowInGroup = Wire(chiselTypeOf(currRowInGroup_q))                      // 当前Group内行数的组合逻辑线（用于下一周期预计算）
val nRowInGroup = vRTableReadData.nRows                                        // 从VR表读取当前Group的总行数
val isPR = dec.io.prStart && (outRowCount_q === 0.U)                           // 是否为PR（PageRank）首行：prStart=1且是第一行
val isVR = vRTableReadData.isVRWithPrevGroup && !isPR                          // 是否为VR（跨组溢出行）：VR表标记且非PR行
val groupOutAddr = currRowInGroup << log2Ceil((cp.blockSize * cp.nColInDense)/8) // Group输出地址 = 组内行号 × 每行字节数
val groupOutData = Wire(chiselTypeOf(groupArray(0).io.outReadData))            // 从当前Group读取的输出数据
val groupOutDataPrev = RegEnable(groupOutData, state === sCombine)             // 上一行的Group输出数据（用于跨行聚合）
val aggWithPrevGroup = ((currRowInGroup_q === 0.U) && isVR)                    // 是否需要与前一个Group聚合：当前行=组首行 且 是VR溢出行
val outDataAgg = groupOutData.map(_.data).zip(groupOutDataPrev.map(_.data)).map{case(d,dP) => d+dP}.reverse.reduce{Cat(_,_)} // 跨Group聚合：当前行数据 + 上一行数据
val prData_q = Reg(chiselTypeOf(outDataAgg))                                   // PR部分和寄存器（PageRank累加值）
val prSplitData = for(i <- 0 until cp.nColInDense)yield{                       // 将PR累加值按blockSize拆分为nColInDense个元素
  prData_q(((i+1)*cp.blockSize) -1, i*cp.blockSize)                            // 提取第i个blockSize大小的元素
}
val prStartRow = (state === sCombine) && (currRowInGroup_q === 0.U) && (vRTableReadGroup_q === 0.U) // PR起始行：Combine状态 + Group0的首行
val prRowAgg = dec.io.prStart && prStartRow                                    // PR行聚合使能：指令prStart=1 且 是PR起始行
val outDataPrAgg = groupOutData.map(_.data).zip(prSplitData).map{case(d,dP) => d+dP}.reverse.reduce{Cat(_,_)} // PR聚合：当前行 + PR累加值
val outDataNoAgg = groupOutData.map(_.data).reverse.reduce(Cat(_,_))            // 无聚合情况：直接拼接当前行数据
val outData = Mux(aggWithPrevGroup, outDataAgg, Mux(prRowAgg, outDataPrAgg, outDataNoAgg)) // 输出数据三选一：跨组聚合 / PR聚合 / 无聚合

val outvRCount_q = RegInit(0.U(32.W))                                          // 已输出的VR行数寄存器
val outvRCount = Mux(state === sCombine && (RegNext(state)===sCombineGroup), outvRCount_q + isVR.asUInt, outvRCount_q) // VR计数：若下一周期要从Combine切回CombineGroup，累加当前是否为VR行
when(state === sCombine && (RegNext(state)===sCombineGroup)){                  // 当Combine完成且下一周期将回到CombineGroup时
  outvRCount_q := outvRCount                                                  // 更新VR行计数
}.elsewhen(state === sIdle){                                                   // 空闲状态
  outvRCount_q := 0.U                                                          // 重置VR行计数
}
when(state === sCombine){                                                      // Combine状态
  outRowCount_q := outRowCount_q + 1.U                                         // 输出总行数+1
}.elsewhen(state === sIdle){                                                   // 空闲状态
  outRowCount_q := 0.U                                                         // 重置输出总行数
}
val outWriteAddr = (outRowCount_q - outvRCount) << log2Ceil((cp.blockSize * cp.nColInDense)/8) // 输出写入地址 = (总行数 - VR行数) × 每行字节数（跳过VR行占位）
currRowInGroup := Mux(state === sCombine, currRowInGroup_q + 1.U, currRowInGroup_q) // 组内行索引：Combine状态递增，否则保持
val outWriteEn = state === sCombine                                            // 输出写使能：仅在Combine状态写入
when((state === sCompute)){                                                    // 进入Compute状态时
  vRTableReadGroup_q := 0.U                                                    // 初始化VR表读索引为0
}.elsewhen(((state === sCombine) || (state === sCombineGroup)) && (currRowInGroup === nRowInGroup)){ // 在Combine/CombineGroup且当前Group行已处理完时
  vRTableReadGroup_q := vRTableReadGroup_q + 1.U                               // 切换到下一个Group
}

when(state === sCombine){                                                      // Combine状态
  when(currRowInGroup === (nRowInGroup)){                                      // 当前Group所有行处理完毕
    currRowInGroup_q := 0.U                                                    // 组内行索引归零
  }.otherwise{                                                                 // 行未处理完
    currRowInGroup_q := currRowInGroup                                         // 更新为递增后的值
  }
}
vRTableReadGroup := Mux(((state === sCombine) || (state === sCombineGroup)) && (currRowInGroup === nRowInGroup),vRTableReadGroup_q + 1.U,vRTableReadGroup_q) // VR表读索引：Group处理完切换到下一Group，否则保持

io.spOutWrite.bits.addr := outWriteAddr                                        // 输出Scratchpad写地址
io.spOutWrite.bits.data := outData                                             // 输出Scratchpad写数据（已经过聚合处理）
io.spOutWrite.valid := outWriteEn                                              // 输出Scratchpad写使能

// pr partial sum io（PageRank部分和IO：将最后一行的结果暂存用于下一轮累加）
val prEndRow = (state === sCombine) && (currRowInGroup_q === (nRowInGroup - 1.U)) && (vRTableReadGroup_q === (cp.nGroups - 1).U) // PR末尾行：Combine + 最后一个Group的倒数第二行
val prRowWrite = dec.io.prEnd && prEndRow                                      // PR行写使能：指令prEnd=1 且 是PR末尾行
val prData = outData                                                           // 要保存的PR数据 = 当前输出数据
when(prRowWrite){                                                              // 当时机满足时
  prData_q := prData                                                           // 将当前输出数据存入prData_q寄存器
}



// group io（批量连接nGroups个Group模块的IO）
  for(i <- 0 until cp.nGroups){                                                // 遍历所有Group
    groupArray(i).io.outReadCmd.map(_.addr := groupOutAddr)                    // 设置每个Group的输出读地址
    groupOutData := MuxTree(vRTableReadGroup_q, groupArray.map(_.io.outReadData)) // 根据vRTableReadGroup_q选择对应Group的输出数据
    groupArray(i).io.nRowPtrInGroup.bits := nRowWritten                        // 告诉Group已写入的行指针数
    groupArray(i).io.nRowPtrInGroup.valid := (nRowWrittenValid && groupSel === i.U) // 仅当nRowWritten有效且当前选中该Group时valid
    vrArbiter.io.in(i).bits.VRTableEntry := groupArray(i).io.vrEntry.bits      // 连接Group的VR条目输出到仲裁器输入
    vrArbiter.io.in(i).bits.group := i.U                                       // 设置仲裁器输入对应的Group编号
    vrArbiter.io.in(i).valid := groupArray(i).io.vrEntry.valid                 // 连接valid信号
    vrArbiter.io.in(i).ready <> groupArray(i).io.vrEntry.ready                 // 连接ready信号（双向握手）
    groupArray(i).io.nNonZero.bits := nNonZeroPerGroup                         // 告诉Group每组的非零元素数
    groupArray(i).io.nNonZero.valid := start                                   // instruction start时valid
    groupArray(i).io.start := (state === sCompute)                             // Group启动信号：进入sCompute状态
    groupArray(i).io.ptrSpWrite.bits.addr :=  rowPtrWriteAddr                  // RowPtr写入Group scratchpad的地址
    groupArray(i).io.ptrSpWrite.valid :=  rowPtrWriteEn && (groupSel === i.U)  // RowPtr写valid：写使能且当前Group被选中
    groupArray(i).io.ptrSpWrite.bits.data := rowPtrData                        // RowPtr写入的数据
    groupArray(i).io.spWrite.bits.spSel :=                                     // Group scratchpad子模块选择信号
      MuxLookup(state,                                                         // 根据当前状态选择写入哪个scratchpad
      0.U, // default
      Array(
        sDataMoveVal -> 0.U,                                                   // Val搬运→scratchpad[0]（Val暂存区）
        sDataMoveRow -> 2.U,                                                   // Row搬运→scratchpad[2]（RowPtr暂存区）
        sDataMoveCol -> 3.U,                                                   // Col搬运→scratchpad[3]（ColIdx暂存区）
        sDataMoveDen -> 1.U                                                    // Dense搬运→scratchpad[1]（Dense暂存区）
      ))
    groupArray(i).io.spWrite.bits.spWriteCmd.addr :=                           // Group scratchpad写地址
      MuxLookup(state,                                                         // 根据状态选择
      0.U, // default
      Array(
        sDataMoveRow -> rowPtrWriteAddr,                                       // Row搬运→RowPtr写地址
        sDataMoveCol -> colWriteAddr,                                          // Col搬运→Col写地址
        sDataMoveVal -> valWriteAddr,                                          // Val搬运→Val写地址
        sDataMoveDen -> denWriteAddr                                           // Dense搬运→Dense写地址
      ))
    groupArray(i).io.spWrite.bits.spWriteCmd.data :=                           // Group scratchpad写数据
      MuxLookup(state,                                                         // 根据状态选择数据源
      0.U, // default
      Array(
        sDataMoveRow -> (rowPtrDataBlock.reverse.reduce(Cat(_,_))),            // Row搬运→RowPtr数据块拼接
        sDataMoveCol -> io.gbReadData.data,                                    // Col搬运→GlobalBuffer读回的数据
        sDataMoveVal -> io.gbReadData.data,                                    // Val搬运→GlobalBuffer读回的数据
        sDataMoveDen -> io.gbReadData.data                                     // Dense搬运→GlobalBuffer读回的数据
      ))
    groupArray(i).io.spWrite.valid := Mux(state === sDataMoveDen, true.B,      // Group scratchpad写valid：Dense状态始终写，否则...
      MuxLookup(state,
      0.U, // default
      Array(
        sDataMoveRow -> rowPtrWriteEn,                                         // Row搬运：仅写使能时才valid
        sDataMoveCol -> true.B,                                                // Col搬运：始终valid
        sDataMoveVal -> true.B,                                                // Val搬运：始终valid
        sDataMoveDen -> true.B                                                 // Dense搬运：始终valid
      )).asBool  && (groupSel === i.U))                                        // 叠加条件：必须是当前选中的Group
  }
val computeDone = groupArray.map(_.io.done).reduce(_&&_)                       // 所有Group都done的AND：全部计算完成
io.done := (state === sIdle) && !start                                         // 对外done信号：空闲且未启动

//state machine（主状态机）
  switch(state){                                                               // 根据当前状态跳转
    is(sIdle){                                                                 // === 空闲状态 ===
      when(start){                                                              // 收到启动信号
        state := sDataMoveRow                                                   // 进入RowPtr搬运状态
        colReadAddr := dec.io.sramCol                                           // 初始化Col读地址（来自指令解码）
        valReadAddr := dec.io.sramVal                                           // 初始化Val读地址
        denReadAddr := dec.io.sramDen                                           // 初始化Dense读地址
        rowPtrAddr := dec.io.sramPtr                                            // 初始化RowPtr读地址
      }
    }
    is(sDataMoveRow){                                                          // === RowPtr搬运状态 ===
      when(rowPtrFin){                                                          // 当前Group的RowPtr搬运完成
        when(groupEnd){                                                         // 已经是最后一个Group
          state := sDataMoveCol                                                 // 进入Col搬运状态
          colReadAddr := colReadAddr + bankBlockSizeBytes.U                     // Col读地址跳过已读部分
          nNonZeroPrevTotal := nNonZeroPrevTotal + dec.io.colSize               // 更新累计非零总数
        }
      }.otherwise{                                                              // RowPtr搬运未完成
        rowPtrAddr := rowPtrAddr + (cp.blockSize/8).U                           // RowPtr读地址递增
      }
    }
    is(sDataMoveCol){                                                          // === ColIdx搬运状态 ===
      when(colFin){                                                             // 当前Group的Col搬运完成
        when(groupEnd){                                                         // 最后一个Group
          state := sDataMoveVal                                                 // 进入Val搬运状态
          valReadAddr := valReadAddr + bankBlockSizeBytes.U                     // Val读地址跳过已读部分
        }
        colReadAddr := colReadAddr + bankBlockSizeBytes.U                       // Col读地址递增（不管是否GroupEnd）
      }.otherwise{                                                              // 未完成
        colReadAddr := colReadAddr + bankBlockSizeBytes.U                       // Col读地址继续递增
      }
    }
    is(sDataMoveVal){                                                          // === Val搬运状态 ===
      when(valFin){                                                             // 当前Group的Val搬运完成
        when(groupEnd){                                                         // 最后一个Group
          when(denseLoaded){                                                    // 如果Dense已经加载过
            state := sCompute                                                   // 直接进入计算状态
          }.otherwise{                                                          // Dense未加载
            state := sDataMoveDen                                               // 进入Dense搬运状态
          }
          denReadAddr := denReadAddr + bankBlockSizeBytes.U                     // Dense读地址跳过已读部分
        }
        valReadAddr := valReadAddr + bankBlockSizeBytes.U                       // Val读地址递增
      }.otherwise{                                                              // 未完成
        valReadAddr := valReadAddr + bankBlockSizeBytes.U                       // Val读地址继续递增
      }
    }
    is(sDataMoveDen){                                                          // === Dense搬运状态 ===
      denReadAddr := denReadAddr + bankBlockSizeBytes.U                        // Dense读地址递增
      when(denFin){                                                             // Dense搬运完成
        state := sCompute                                                       // 进入计算状态
      }
    }
    is(sCompute){                                                              // === 计算状态 ===
      when(!denseLoaded){                                                       // 首次计算（训练超时基准）
        computeTimeOut := computeTimeOut + 1.U                                  // 记录超时周期（作为后续计算的时间上限）
      }.otherwise{                                                              // 后续计算
        computeTimer := computeTimer + 1.U                                      // 计时器递增
      }
      when(computeDone){                                                        // 所有Group计算完成
        state := sCombineGroup                                                  // 进入Combine预检状态
        denseLoaded := true.B                                                   // 标记Dense已加载
      }.elsewhen(denseLoaded){                                                 // Dense已加载但计算未完成
        when(computeTimer === computeTimeOut){                                   // 计时器达到超时上限
          state := sIdle                                                        // 回到空闲（跳过本次计算结果）
          computeTimer := 0.U                                                   // 重置计时器
          computeSkipCount := computeSkipCount + 1.U                             // 跳过计数+1
        }
      }
    }
    is(sCombineGroup){                                                         // === Combine预检状态 ===
      when(nRowInGroup === 0.U){                                                // 当前Group有0行
        state := sCombineGroup                                                  // 保持自身（循环检查直到找到有行的Group）
      }.otherwise{                                                              // 有行
        state := sCombine                                                       // 进入Combine汇总状态
      }
    }
    is(sCombine){                                                              // === Combine汇总状态 ===
      when(currRowInGroup_q === (nRowInGroup - 1.U)){                           // 当前Group的倒数第一行处理完毕
        when(aggDone){                                                          // 所有Group聚合完成
          state := sIdle                                                        // 回到空闲
        }.otherwise{                                                            // 还有Group未处理
          state := sCombineGroup                                                // 回到Combine预检（处理下一个Group）
        }
      }
    }
  }
  assert(computeSkipCount =/= 1000.U)                                           // 断言：跳过的计算次数不应达到1000（防止死循环）
}
