# SPGEMM-Accelerator

基于 Verilog 的稀疏矩阵乘法加速器 (SpGEMM: Sparse × Sparse)。

## 项目结构

```
SPGEMM-Accelerator/
├── rtl/
│   ├── include/
│   │   ├── defines.vh          # 全局参数定义 (PE数量、MAC数量、位宽等)
│   │   └── isa.vh              # ISA指令格式定义 (256-bit指令编码)
│   ├── infrastructure/         # 基础设施模块 (从旧SPMM加速器复用)
│   │   ├── axi_interface.v     # AXI总线接口: CR从设备、读MUX、写通道
│   │   ├── decode.v            # 指令解码: FetchDecode/LoadDecode/SpGEMMDecode
│   │   ├── fetch.v             # 取指模块: DDR→指令队列→多通道分发
│   │   ├── load.v              # 加载模块: DRAM→GlobalBuffer burst传输
│   │   ├── store.v             # 存储模块: OutputScratchpad→DRAM写回
│   │   └── scratchpad.v        # 片上SRAM: GlobalBuffer/BankedScratchpad/OutputScratchpad/FIFO
│   ├── core/                   # 核心加速模块 (全新设计)
│   │   ├── core_top.v          # 顶层状态机: Idle→Load→Schedule→Compute→WriteCSR→Store→Finish
│   │   ├── scheduler.v         # 调度器: B Row Length Gen + Workload Analyzer + Partitioner
│   │   ├── pe_top.v            # PE顶层: 集成Decompress+MUL+Aggregation+SPA
│   │   ├── pe_decompress.v     # 解压单元: CSR→MAC乘法流 (A/B Row Controller + Streamer)
│   │   ├── pe_mul_array.v      # 乘法阵列: N_MAC个并行乘法器 (3级流水)
│   │   ├── pe_aggregation.v    # 聚合+累加+SPA: Banked SPA, 列索引聚合, touched_cols
│   │   └── c_csr_writer.v      # CSR写回: 行结果收集→前缀和→CSR格式生成
│   ├── wrapper.v               # 顶层封装: CR + Core + AXI接口
│   ├── sim/
│   │   └── tb_core_top.v       # Testbench: 时钟生成, AXI内存模型, 基本测试
│   └── filelist.f              # 文件列表 (仿真用)
├── note/
│   └── 架构设计.md              # 架构设计文档
└── scala/                      # 原始Chisel代码 (参考)
```

## 架构特点

| 特性 | 说明 |
|------|------|
| 输入格式 | A: CSR, B: CSR |
| 输出格式 | C: CSR |
| 最大维度 | 512 × 512 × 512 |
| PE数量 | 8 (可配) |
| MAC/PE  | 4 (可配) |
| 总MAC数 | 32 |
| 数据宽度 | 16-bit (FP16/IEEE 754 half) |
| AXI数据宽度 | 512-bit |
| 调度算法 | 动态剩余目标 + 最近边界切分 |
| SPA实现 | Banked Partial Row Buffer + touched_cols FIFO |

## 数据流

```
Host → CR(AXI-Lite) → Fetch(DDR) → Decode → Load → GlobalBuffer
                                                         ↓
                                                     Scheduler
                                                   (任务分析与分配)
                                                         ↓
                                                    PE Array × 8
                                              (Decompress→MUL→Aggregation→SPA)
                                                         ↓
                                                    Output Buffer
                                                         ↓
                                                   C CSR Writer
                                               (Row Collect→Prefix Sum→CSR)
                                                         ↓
                                                   Store → DRAM
```

## 快速开始

### 仿真 (Icarus Verilog)
```bash
cd rtl
iverilog -g2012 -o sim.vvp -f filelist.f sim/tb_core_top.v
vvp sim.vvp
```

### 仿真 (Verilator)
```bash
verilator --cc -f rtl/filelist.f rtl/sim/tb_core_top.v --top-module tb_core_top
cd obj_dir && make -f Vtb_core_top.mk && ./Vtb_core_top
```

## 配置参数

在 `rtl/include/defines.vh` 中修改:
- `N_PE`: PE数量 (默认8)
- `N_MAC`: 每个PE的MAC数 (默认4)
- `MAX_M/K/N`: 最大维度 (默认512)
- `DATA_WIDTH`: 数据位宽 (默认32)

## 关键算法

### Scheduler: Row-Block 行块调度

**工作量估计（Row-Block 公式）：**
```
row_eff[i] = sum(b_row_nnz[ A_col_idx[p] ])     for all A(i,k) in row i
row_cyc[i] = ceil(row_eff[i] / N_MAC)            整个行只做一次 ceil
```

相比旧公式 `sum(ceil(b_row_nnz/N_MAC))`，消除逐元素末周期浪费。

**行分配：动态剩余目标 + 最近边界切分：**
```
dynamic_target = ceil(remaining_work / remaining_pe)
当 cur_load + w >= dynamic_target 时:
  比较 err_before = dynamic_target - cur_load
        err_after  = new_load - dynamic_target
  如果 err_after <= err_before 或 cur_load == 0: 当前行归属当前PE
  否则: 当前行归属下一个PE
```

### PE: SPA (Sparse Accumulator)

- acc_val[512]: 当前C行各列的累加值
- acc_valid[512]: 标记列是否被写入
- touched_cols FIFO: 记录当前行被触发的列
- Banked: 按 j % N_MAC 分bank，减少冲突
