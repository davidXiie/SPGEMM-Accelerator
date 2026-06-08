#  项目总览

rtl/
├── include/                    # 全局参数 + ISA定义
│   ├── defines.vh              # PE=8, MAC/PE=4, MAX_DIM=512, AXI=512bit
│   └── isa.vh                  # 256-bit指令格式 (Load/Store/SpGEMM/Sched)
│
├── infrastructure/             # 从旧SPMM加速器复用改造
│   ├── axi_interface.v         # AXI-Lite CR从设备 + AXI-Full读MUX/写通道
│   ├── decode.v                # 三级解码: FetchDecode/LoadDecode/SpGEMMDecode
│   ├── fetch.v                 # DDR取指→指令队列→4通道分发(Load/SpGEMM/Store/Sched)
│   ├── load.v                  # AXI burst读→GlobalBuffer写入
│   ├── store.v                 # OutputScratchpad→AXI burst写回DRAM
│   └── scratchpad.v            # GlobalBuffer/BankedScratchpad/OutputScratchpad/FIFO
│
├── core/                       # 全新设计的SpGEMM核心
│   ├── core_top.v              # 主状态机: Idle→Load_A→Load_B→Schedule→Compute→WriteCSR→Store→Finish
│   ├── scheduler.v             # B Row Length Gen + A Workload Analyzer + 动态剩余目标切分
│   ├── pe_top.v                # PE集成: Decompress+MUL×4+Aggregation+SPA
│   ├── pe_decompress.v         # CSR解压: A Row Ctrl→A Elem Fetch→B Row Ctrl→B Stream→MAC Dispatch
│   ├── pe_mul_array.v          # N_MAC并行乘法器 (3级流水DSP48)
│   ├── pe_aggregation.v        # Banked SPA累加: acc_val[512]+acc_valid[512]+touched_cols FIFO
│   └── c_csr_writer.v          # 三阶段CSR生成: Collect→PrefixSum→Write
│
├── wrapper.v                   # 顶层封装: CR_slave + Core_top
├── sim/tb_core_top.v           # Testbench (AXI内存模型+基本激励)
├── filelist.f                  # 仿真文件列表
└── README.md                   # 完整使用说明
