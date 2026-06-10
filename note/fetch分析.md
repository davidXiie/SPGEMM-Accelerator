下面对 `fetch.v` 模块的**寄存器、连线和工作逻辑**进行全面梳理。该模块负责从 DRAM 通过 AXI 总线读取指令流，解码后分发到 4 个不同类型的执行单元（Load、SpGEMM、Store、Scheduler）。

---

## 一、模块接口与参数

### 参数（Parameter）
| 参数名 | 值 | 含义 |
|--------|----|------|
| `INST_QUEUE_DEPTH` | 16 | 指令队列深度（按 AXI beat 存储） |
| `INST_QUEUE_DEPTH_LOG` | 4 | 队列深度的 log2 值（用于指针位宽） |

### 输入 / 输出信号

| 类别 | 信号名 | 方向 | 描述 |
|------|--------|------|------|
| **控制** | `launch` | 输入 | 启动指令读取的脉冲（上升沿检测） |
| | `ins_baddr[AXI_ADDR_WIDTH-1:0]` | 输入 | 指令在 DRAM 中的基地址 |
| | `ins_count[15:0]` | 输入 | 需要读取的指令总数 |
| **AXI 读主端口** | `m_axi_arvalid`, `m_axi_araddr`, `m_axi_arlen` | 输出 | 读地址通道 |
| | `m_axi_rvalid`, `m_axi_rdata`, `m_axi_rlast` | 输入 | 读数据通道 |
| | `m_axi_rready` | 输出 | 读数据握手 |
| **指令分发** | `ld_inst_valid/ready/data` | 输出/输入/输出 | Load 单元队列 |
| | `sp_inst_valid/ready/data` | 输出/输入/输出 | SpGEMM 单元队列 |
| | `st_inst_valid/ready/data` | 输出/输入/输出 | Store 单元队列 |
| | `sch_inst_valid/ready/data` | 输出/输入/输出 | Scheduler 单元队列 |
| **时钟复位** | `aclk`, `aresetn` | 输入 | 时钟及异步复位（低有效） |

> 关键常量：  
> `INS_PER_TRANSFER = AXI_DATA_WIDTH / INST_WIDTH`（本例为 512/256 = 2）  
> 每个 AXI beat 包含 **2 条指令**。

---

## 二、内部寄存器及其用途

| 寄存器名 | 位宽 | 描述 |
|----------|------|------|
| `inst_queue` | 深度 16 × `AXI_DATA_WIDTH` | 存储 AXI 读回的原始数据（每个 entry 为 1 个 beat = 2 条指令） |
| `wr_ptr` | `INST_QUEUE_DEPTH_LOG:0` | 写指针（指向下一个要写入的队列 entry） |
| `rd_ptr` | 同上 | 读指针（指向当前待拆分的 beat） |
| `state`, `state_next` | 3 位 | 状态机当前状态与下一状态 |
| `raddr` | `AXI_ADDR_WIDTH` | 当前正在读取的 DRAM 地址 |
| `rlen` | 8 位 | 当前 AXI 突发长度（`arlen` 值，0 表示 1 个 beat） |
| `ilen` | 8 位 | 当前阶段期望接收的 beat 数量（用于判断何时停止读） |
| `xrem` | 16 位 | 剩余尚未读取的“指令批次”数量（1 批次 = 1 个 beat） |
| `xsize` | 16 位 | 总需要读取的指令批次数（`ins_count` 换算而来） |
| `xmax` | 16 位 | 单次 AXI 突发允许的最大 beat 数（`1 << AXI_LEN_WIDTH`） |
| `pack_sel` | `INS_PER_TRANSFER_LOG:0` | 当前 beat 内选择第几个指令（0 或 1） |
| `launch_r` | 1 位 | 对 `launch` 的延迟采样，用于产生上升沿脉冲 |

---

## 三、重要组合逻辑连线

| 连线名 | 产生方式 | 作用 |
|--------|----------|------|
| `queue_count` | 计算 `wr_ptr - rd_ptr`（模深度） | 队列中已存储的 beat 数量 |
| `queue_empty` | `queue_count == 0` | 队列空标志 |
| `queue_full` | `queue_count >= DEPTH-1` | 队列几乎满（用于反压） |
| `launch_pulse` | `launch && !launch_r` | 启动脉冲（单时钟周期） |
| `inst_pack` | `inst_queue[rd_ptr]` | 当前正在拆分的原始 beat |
| `cur_inst` | 从 `inst_pack` 中按 `pack_sel` 提取 | 当前解码的指令（256 位） |
| `inst_opcode` | `cur_inst[2:0]` | 指令操作码 |
| `is_load / is_store / is_spgemm / is_sched / is_finish` | 与 `define.vh` 中的操作码比较 | 指令类型判断 |
| `split_valid` | `state == S_SPLIT && !is_finish` | 当前处于拆分状态且非 FINISH 指令 |

---

## 四、工作逻辑详解

### 1. 启动与地址计算
- 检测到 `launch_pulse` 后，计算总指令批次数：  
  `xsize = (ins_count >> 1) - 1`  
  > 注：此处 `-1` 是为了将长度转换为 AXI `arlen` 格式（0 表示 1 个 beat）。  
  > 该计算隐含要求 `ins_count` 为偶数，且至少为 2，否则可能溢出。用户需保证指令数为偶数。
- 根据 `xmax`（最大突发长度限制）分段：
  - 若总批次数小于 `xmax`，则一次性读完：`rlen = ilen = xsize`，`xrem = 0`
  - 否则先读 `xmax` 个 beat，`xrem` 记录剩余批次数
- 设置读起始地址 `raddr = ins_baddr`

### 2. 状态机流转

```
        ┌─────┐
        │IDLE │
        └──┬──┘
           │ launch_pulse
           ▼
      ┌───────┐
      │READ_CMD│ ── arready ──▶ ┌─────────┐
      └───────┘                  │READ_DATA│
                                 └────┬────┘
                                      │ queue_count >= ilen
                                      ▼
            ┌─────────┐           ┌───────┐
            │  IDLE   │◀── xrem=0 │ DRAIN │
            └─────────┘           └───┬───┘
                 ▲                     │ queue非空且非FINISH
                 │                     ▼
                 │                ┌─────────┐
                 └── xrem≠0 ──────┤  SPLIT  │
                    重新读         └─────────┘
```

- **S_IDLE**：空闲，等待启动脉冲。
- **S_READ_CMD**：发送 AXI 读请求（`arvalid=1`），等待 `arready`。一旦握手，下一周期进入 **S_READ_DATA**。
- **S_READ_DATA**：接收数据。每个有效 `rvalid` 周期，若队列未满，将 `m_axi_rdata` 写入 `inst_queue`，`wr_ptr++`。当接收的 beat 数达到 `ilen` 时，进入 **S_DRAIN**。
- **S_DRAIN**：排空队列。
  - 若队列为空：
    - 如果 `xrem == 0`：所有指令处理完毕 → **S_IDLE**
    - 否则，发起下一段读取（更新 `raddr` 和 `rlen`）→ **S_READ_CMD**
  - 若队列非空：
    - 如果当前 beat 的第一个指令（由 `rd_ptr` 指向）不是 `OP_FINISH`，则进入 **S_SPLIT** 开始拆分。
    - ⚠️ 如果第一个指令是 `OP_FINISH`，则状态机**卡在 S_DRAIN**，不处理。这可能是设计缺陷，建议避免使用 `OP_FINISH` 或修改逻辑。
- **S_SPLIT**：拆分并分发指令。
  - 每次分发一个指令到对应通道（`ld_inst_valid` 等根据 opcode 置高）。
  - 当下游 `ready` 信号有效时：
    - 若 `pack_sel == INS_PER_TRANSFER-1`（当前 beat 的最后一个指令），则消费完整个 beat：`rd_ptr++`，下一状态回到 **S_DRAIN**。
    - 否则，`pack_sel++`，下一状态仍为 **S_SPLIT**，继续分发同一 beat 中的下一条指令。

### 3. 队列读写控制
- **写队列**：仅在 `state == S_READ_DATA` 且 `m_axi_rvalid & m_axi_rready & !queue_full` 时写。
- **读队列**：仅在 `state == S_SPLIT` 且最后一个指令被消费时，将 `rd_ptr_inc` 置 1，使 `rd_ptr` 递增。  
  因此 `rd_ptr` 始终指向当前正在拆分的 beat。

### 4. AXI 读地址更新
- 在 `launch_pulse` 或新一段读取开始时（`state==DRAIN && queue_empty && xrem!=0`），`raddr` 增加 `xmax_val * (AXI_DATA_WIDTH/8)` 字节。

### 5. 指令分发
- 分发有效条件：`split_valid = (state == S_SPLIT) && !is_finish`  
  （FINISH 指令不会被分发，且会导致上面提到的卡死）
- 各通道的 `_valid` 等于 `split_valid` 且对应的 opcode 匹配。
- 所有通道的 `_inst` 均直接连接 `cur_inst`（256 位原始指令）。

---

## 五、注意事项与潜在问题

1. **指令计数计算**  
   `xsize_comb = (ins_count >> 1) - 1`  
   - 当 `ins_count` 为 2 时，`xsize_comb = 0`，`rlen=0` → 读取 1 个 beat（2 条指令），正确。  
   - 当 `ins_count` 为 1 时，右移得 0，减 1 得 -1（无符号溢出为 65535），导致错误读取大量指令。  
   **建议**：保证 `ins_count` 为偶数，或修改为向上取整再减 1。

2. **FINISH 指令处理不完整**  
   代码中 `is_finish` 既阻止进入 S_SPLIT，又不消费队列，导致一旦队列头部是 `OP_FINISH`，系统死锁。  
   若设计意图是让 FINISH 作为终止指令，则应将其也视为普通指令分发（或直接在 DRAIN 中丢弃并递增 rd_ptr）。

3. **队列满时的反压**  
   `m_axi_rready` 在 `state==S_READ_DATA && !queue_full` 时有效，若队列满则暂停接收，防止溢出。这是合理的流控。

4. **复位行为**  
   异步复位（`aresetn` 低有效）将所有状态寄存器清零，队列内容无效。

5. **AXI 突发长度限制**  
   `xmax_val = (1 << `AXI_LEN_WIDTH)`，`AXI_LEN_WIDTH` 通常为 8（最大 256 beats），但代码中 rlen 为 8 位，符合 AXI4 规范。

---

## 六、模块数据流总结

```
[DRAM] → AXI Read → inst_queue (beat粒度) → pack_sel拆分 → cur_inst → decode → 4个分发端口
           ↑                                                       ↓
       状态机控制                                           下游ready握手
```

该模块是一个典型的指令预取与分发器，通过内部缓冲队列解耦 AXI 读取延迟和下游执行单元的消费速率，支持大块指令传输（分段突发），适合加速器场景。使用时需注意指令数对齐和 FINISH 指令的处理。