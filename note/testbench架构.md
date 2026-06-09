
## cocotb Testbench 搭建概要

### 文件结构（4个文件）

```
test/
├── Makefile       ← cocotb 仿真配置
├── gen_data.py    ← 数据生成脚本（独立运行，生成仿真输入文件）
├── test_dut.py    ← cocotb testbench（@cocotb.test()）
└── data.txt       ← gen_data.py 生成的二进制数据文件
```

### 各文件职责

**1. `Makefile`** — 告诉 cocotb 用什么仿真器、顶层模块名、Python 测试模块名、硬件源文件路径。

**2. `gen_data.py`** — 独立脚本，只做一件事：把测试数据（指令、矩阵权重、配置参数等）按 DUT 的接口格式编码为二进制串，写入 `data.txt`。不依赖 cocotb。

**3. `test_dut.py`** — cocotb testbench，核心模式：

```python
class TB:
    def __init__(self, dut):
        # ① 创建 AXI Master（写寄存器控制 DUT）
        # ② 创建 AXI Slave（mmap 模拟 DRAM，DUT 可读写）
        # ③ 从 data.txt 读二进制数据写入 mmap
        # ④ 启动时钟 + 复位

    async def launch(self, ...):
        # 通过 AXI Master 写配置寄存器 + 写启动信号

@cocotb.test()
async def test_case(dut):
    tb = TB(dut)
    await tb.launch(...)
    await Timer(10, units='us')   # 等待 DUT 执行完成
```

**4. `data.txt`** — `gen_data.py` 的输出，testbench 初始化时读入。

### 数据流

```
gen_data.py  ──→  data.txt  ──→  test_dut.py 读入 mmap  ──→  DUT 通过 AXI 读写 mmap
```

### 关键依赖

- `cocotb` + `cocotbext-axi`（如需 AXI 总线模型）
- Icarus Verilog 或 Verilator 作为仿真后端

---