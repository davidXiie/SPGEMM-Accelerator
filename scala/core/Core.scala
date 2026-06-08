package gcn.core                                          // 包声明

import chisel3._                                           // Chisel3 基础类型和模块
import chisel3.util._                                      // Chisel3 工具（Mux、Cat、Enum、switch等）
import vta.util.config._                                   // VTA 配置参数框架

/** Core.
 *
 * The core defines the current GCN Accelerator architecture by connecting memory and
 * compute modules together such as load/store and compute.
 *
 * Also, the core must be instantiated by a wrapper using the
 * Control Registers (CR) and the Memory Engine (ME) interfaces.
 */

class Core(implicit p: Parameters) extends Module {        // GCN加速器核心模块
  val io = IO(new Bundle {                                  // Core顶层IO
    val cr = new CRClient                                   // 控制寄存器客户端接口
    val me = new MEMaster                                    // 内存引擎主接口
  })
  val cp = p(AccKey).coreParams                              // Core参数
  val cr = p(AccKey).crParams                                // CR参数
  val fetch = Module(new Fetch)                              // 取指模块：从DRAM取指令到SRAM队列
  val load = Module(new Load)                                // 加载模块：从DRAM加载数据到scratchpad
  val globalBuffer = Module(new GlobalBuffer())              // 全局缓冲：存储输入特征/权重
  val outputScratchpad = Module(new OutputScratchpad())      // 输出暂存：缓存计算结果
  val compute = Module(new Compute)                          // 计算模块：执行矩阵乘法等GCN运算
  val store = Module(new Store)                              // 存储模块：回写结果到DRAM
  val start = Wire(Bool())                                   // 启动信号

  // ===== 数据流连接 =====
  load.io.spWrite.ready := true.B                            // Load写scratchpad始终就绪
  globalBuffer.io.spWrite <> load.io.spWrite.bits             // Load → GlobalBuffer 写入数据
  globalBuffer.io.writeEn := load.io.spWrite.fire             // Load写有效时使能GB写入
  globalBuffer.io.spReadCmd <> compute.io.gbReadCmd           // Compute → GB 读命令
  globalBuffer.io.spReadData <> compute.io.gbReadData         // GB → Compute 读数据
  compute.io.spOutWrite.bits <> outputScratchpad.io.spWrite   // Compute → OutputScratchpad 写结果
  compute.io.spOutWrite.ready := true.B                       // Compute写输出始终就绪
  outputScratchpad.io.writeEn := compute.io.spOutWrite.valid  // Compute写有效时使能OS写入
  io.cr.ecnt(0) := 0.U                                       // 事件计数器0清零
  // io.cr.ecnt(0) <> load.io.ecnt                            // (被注释) Load事件计数
  // io.cr.ecnt(1) <> compute.io.ecnt(0)                      // (被注释) Compute事件计数
  // io.cr.ecnt(2) <> store.io.ecnt                           // (被注释) Store事件计数
  // for(i <- 0 until cp.nPE){                                // (被注释) PE事件计数循环
  //   for(j <- 0 until cr.nPEEventCtr){
  //     io.cr.ecnt(3+(i*cr.nPEEventCtr) + j) <> compute.io.ecnt((i*cr.nPEEventCtr) + j + 1)
  //   }
  // }
 
  start := io.cr.launch                                      // 启动信号来自CR的launch寄存器

  // ===== 主状态机 =====
  val sIdle :: sLoad :: sCompute :: sStore :: sFinish :: Nil = Enum(5)  // 5个状态：空闲、加载、计算、存储、完成
  val state = RegInit(sIdle)                                 // 当前状态寄存器，初始为Idle
  val ctr = RegInit(0.U(4.W))                                // 循环计数器（用于Load 4轮子循环）
  compute.io.valid := (state === sCompute)                   // 仅在计算状态使能Compute
  load.io.valid := (state === sLoad) && !load.io.done        // 仅在加载状态且未完成时使能Load
  store.io.valid := (state === sStore) && !store.io.done     // 仅在存储状态且未完成时使能Store

  // ===== 取指 =====
  fetch.io.launch := io.cr.launch                            // 取指启动信号
  fetch.io.ins_baddr := Cat(io.cr.vals(0),io.cr.vals(0))     // 指令基地址（拼接两次得到64位地址）
  fetch.io.ins_count := io.cr.vals(1)                        // 指令总数
  val insCountTotal = fetch.io.ins_count                     // 指令总数（别名）
  val insCountCurr_q = RegInit(0.U(32.W))                    // 当前已执行指令数寄存器
  val insCountCurr = Mux(state === sStore, insCountCurr_q + 6.U, insCountCurr_q)  // 存储时指令计数+6（每条GCN算子6条指令）

  // ===== 指令分发 =====
  load.io.inst <> fetch.io.inst.ld                            // Fetch → Load 加载指令
  compute.io.inst <> fetch.io.inst.co                         // Fetch → Compute 计算指令
  store.io.inst <> fetch.io.inst.st                           // Fetch → Store 存储指令
  store.io.spReadCmd <> outputScratchpad.io.spReadCmd         // Store ← OutputScratchpad 读命令
  store.io.spReadData <> outputScratchpad.io.spReadData       // Store ← OutputScratchpad 读数据

  // ===== 内存访问接口 =====
  io.cr.finish := (state === sFinish)                        // 完成信号：状态为sFinish
  io.me.rd(0) <> fetch.io.me_rd                              // ME读通道0 → Fetch
  io.me.rd(1) <> load.io.me_rd                               // ME读通道1 → Load
  io.me.wr(0) <> store.io.me_wr                              // ME写通道0 → Store


  switch(state){                                              // 状态机跳转逻辑
    is(sIdle){                                                // 空闲状态
        when(start){                                          // 收到启动信号
            state := sLoad                                    // 进入加载状态
            ctr := ctr + 1.U                                  // 加载计数器+1
        }
    }
    is(sLoad){                                                // 加载状态
      when(load.io.done){                                     // 当前加载完成
        when(ctr === 4.U){                                    // 已完成4轮加载
          state := sCompute                                   // 进入计算状态
        }.otherwise{                                          // 未完成4轮
          state := sLoad                                      // 保持加载状态
          ctr := ctr + 1.U                                    // 加载计数器+1
        }
      }
    }
    is(sCompute){                                             // 计算状态
      when(compute.io.done){                                  // 计算完成
        state := sStore                                       // 进入存储状态
        ctr := 0.U                                            // 计数器清零
      }
    }
    is(sStore){                                               // 存储状态
      when(store.io.done){                                    // 当前存储完成
        insCountCurr_q := insCountCurr_q + 6.U                // 更新已执行指令数
        when(insCountCurr === insCountTotal){                  // 所有指令执行完毕
          state := sFinish                                    // 进入完成状态
        }.otherwise{                                          // 还有剩余指令
          state := sLoad                                      // 回到加载状态，处理下一算子
          ctr := 1.U                                          // 加载计数器从1开始
        }
      }
    }
  }
  load.io.spWrite.ready := true.B                             // Load写scratchpad始终就绪

}
