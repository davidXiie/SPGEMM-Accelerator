
package gcn.core                                          // 包声明

import scala.math.pow                                      // 幂运算（未直接使用）
import scala.math.sqrt                                     // 开方（未直接使用）
import chisel3._                                           // Chisel3 基础类型和模块
import chisel3.util._                                      // Chisel3 工具（Enum、Decoupled、RegInit等）
import vta.util.config._                                   // VTA 配置参数框架
import vta.util._                                          // VTA 工具（SyncQueue等）

/** Fetch.
 *
 * The fetch unit reads instructions (tasks) from memory (i.e. DRAM), using the
 * Memory Engine (ME), and push them into an instruction queue called
 * inst_q. Once the instruction queue is full, instructions are dispatched to
 * the Load, Compute and Store module queues based on the instruction opcode.
 * After draining the queue, the fetch unit checks if there are more instructions
 * via the ins_count register which is written by the host.
 *
 * Additionally, instructions are read into two chunks (see sReadLSB and sReadMSB)
 * because we are using a DRAM payload of 8-bytes or half of a VTA instruction.
 * This should be configurable for larger payloads, i.e. 64-bytes, which can load
 * more than one instruction at the time. Finally, the instruction queue is
 * sized (entries_q), depending on the maximum burst allowed in the memory.
 */
class Fetch(debug: Boolean = false)(implicit p: Parameters) extends Module with ISAConstants{  // 取指模块：从DDR读指令→解码→分发给Load/Compute/Store
  val vp = p(AccKey).crParams                               // CR参数（寄存器位宽等）
  val mp = p(AccKey).memParams                               // 内存参数（地址位宽、数据位宽、突发长度等）
  val io = IO(new Bundle {                                   // Fetch的IO接口
    val launch = Input(Bool())                               // 启动信号：CPU写CR后触发
    val ins_baddr = Input(UInt(mp.addrBits.W))               // 指令在DDR中的基地址
    val ins_count = Input(UInt(vp.regBits.W))                // 指令总条数
    val me_rd = new MEReadMaster                             // ME读接口：通过ME从DDR读指令
    val inst = new Bundle {                                  // 三条指令输出通道
      val ld = Decoupled(UInt(INST_BITS.W))                  // Load指令 → Load模块
      val co = Decoupled(UInt(INST_BITS.W))                  // Compute指令 → Compute模块
      val st = Decoupled(UInt(INST_BITS.W))                  // Store指令 → Store模块
    }
  })
//   val entries_q = 1 << (mp.lenBits - 1) // one-instr-every-two-me-word
  val insPerTransfer = (mp.dataBits/INST_BITS)               // 每次DDR传输包含的指令条数（512bit/128bit=4条）
  val entries_q = (1 << mp.lenBits)                          // 指令队列深度 = 最大突发长度
  val inst_q = Module(new SyncQueue(UInt(mp.dataBits.W), entries_q))  // 指令缓冲队列（宽度=mp.dataBits, 深度=entries_q）
  val dec = Module (new FetchDecode)                         // 取指解码器：判断指令属于Load/Compute/Store

  val s1_launch = RegNext(io.launch, init = false.B)         // launch延迟1拍：用于边缘检测
  val pulse = io.launch & ~s1_launch                         // launch上升沿脉冲：启动触发

  val raddr = Reg(chiselTypeOf(io.me_rd.cmd.bits.addr))      // 当前读地址寄存器
  val rlen = Reg(chiselTypeOf(io.me_rd.cmd.bits.len))        // ME读请求的突发长度
  val ilen = Reg(chiselTypeOf(io.me_rd.cmd.bits.len))        // 指令队列预期填充量（用于判断是否填满）

  val xrem = Reg(chiselTypeOf(io.ins_count))                 // 剩余待取指令数（换算为传输次数）
  val xsize = (io.ins_count >> log2Ceil(insPerTransfer)) - 1.U  // 需要读取的总传输次数 = 指令数/每次传输条数 - 1
  val xmax = (1 << mp.lenBits).U                             // 单次最大传输次数 = 2^lenBits
  val xmax_bytes = ((1 << mp.lenBits) * mp.dataBits / 8).U   // 单次最大传输字节数 = max传输次数 × dataBits/8

  val sIdle :: sReadCmd :: sRead :: sDrain :: sSplit :: Nil = Enum(5)  // 5个状态：空闲→发读命令→读数据→排空→分裂指令
  val state = RegInit(sIdle)                                 // 当前状态寄存器，初始Idle
  val packInst = Reg(chiselTypeOf(io.me_rd.data.bits.data))  // 打包指令寄存器（一次DDR读回包含多条指令）
  val packInstSelect = RegInit(0.U(log2Ceil(mp.dataBits).W)) // 指令选择偏移：当前在packInst中选择第几条指令
  val deqReady = Wire(Bool())                                // 出队就绪线网
  val inst = RegInit(0.U(mp.dataBits.W))                     // 当前正在解码的指令缓存（宽度=mp.dataBits）

  // control
  switch(state) {                                             // 状态机控制
    is(sIdle) {                                               // 空闲：等待launch脉冲
      when(pulse) {                                           // 检测到启动上升沿
        state := sReadCmd                                     // 跳转到发读命令
        when(xsize < xmax) {                                  // 所需传输量小于单次最大传输
          rlen := xsize                                       // 突发长度 = 全部所需
          ilen := xsize                                       // 预期填充量 = 全部所需
          xrem := 0.U                                         // 无剩余
        }.otherwise {                                         // 所需传输量超过单次最大值，需要分批
          rlen := xmax - 1.U                                  // 突发长度 = 最大-1
          ilen := xmax - 1.U                                  // 预期填充量 = 最大-1
          xrem := xsize - xmax                                // 剩余 = 总量 - 本轮
        }
      }
    }
    is(sReadCmd) {                                            // 发送ME读命令
      when(io.me_rd.cmd.ready) {                              // ME接受读命令
        state := sRead                                        // 进入读数据状态
      }
    }
    is(sRead) {                                               // 接收ME返回的数据，写入inst_q
      when(io.me_rd.data.valid) {                             // ME返回数据有效
        when(inst_q.io.count === ilen) {                      // 指令队列已填满预期量
          state := sDrain                                     // 进入排空状态
          packInstSelect := 0.U                               // 指令选择偏移归零
        }.otherwise {                                         // 未填满
          state := sRead                                      // 继续读
        }
      }
    }
    is(sDrain) {                                              // 排空指令队列
      when(inst_q.io.count === 0.U) {                         // 队列已空
        when(xrem === 0.U) {                                  // 无剩余指令
          state := sIdle                                      // 回到空闲
        }.elsewhen(xrem < xmax) {                             // 剩余量小于单次最大
          state := sReadCmd                                   // 发起新读
          rlen := xrem                                        // 突发长度 = 剩余量
          ilen := xrem                                        // 预期填充 = 剩余量
          xrem := 0.U                                         // 清零剩余
        }.otherwise {                                         // 仍有大批剩余
          state := sReadCmd                                   // 发起新读
          rlen := xmax - 1.U                                  // 突发长度 = 最大-1
          ilen := xmax - 1.U                                  // 预期填充 = 最大-1
          xrem := xrem - xmax                                 // 更新剩余
        }
      }.otherwise{                                            // 队列非空
        state := sSplit                                       // 进入分裂指令发送状态
      }
    }
    is(sSplit){                                               // 将packInst中的指令逐条发给目标模块
      when(io.inst.ld.fire || io.inst.co.fire || io.inst.st.fire){  // 任一指令通道握手成功
        when(packInstSelect === (mp.dataBits - INST_BITS).U){ // 当前packInst已发完最后一条指令
          packInstSelect := 0.U                               // 偏移归零
          state := sDrain                                     // 回到排空，准备出队下一个packInst
        }.otherwise{                                          // packInst中还有剩余指令
          packInstSelect := packInstSelect + INST_BITS.U      // 偏移前进一条指令的长度
        }
      }
    }
  }

  // read instructions from dram
  when(state === sIdle) {                                     // 空闲时初始化读地址
    raddr := io.ins_baddr                                     // 读地址 = 指令基地址
  }.elsewhen(state === sDrain && inst_q.io.count === 0.U && xrem =/= 0.U) {  // 排空完毕且还有剩余
    raddr := raddr + xmax_bytes                               // 读地址前进一个最大传输块
  }

  io.me_rd.cmd.valid := state === sReadCmd                    // 仅在sReadCmd发送读请求
  io.me_rd.cmd.bits.addr := raddr                             // 读地址
  io.me_rd.cmd.bits.len := rlen                               // 突发长度
  io.me_rd.cmd.bits.tag := 0.U // Cannot reorder requests as a queue is used  // 请求标签（固定0，不支持乱序）

  io.me_rd.data.ready := (state === sRead) && inst_q.io.enq.ready  // 仅在sRead状态且队列可入队时接收数据


  inst_q.io.enq.valid := io.me_rd.data.valid                  // ME返回数据有效 → 指令队列入队有效
  inst_q.io.enq.bits := io.me_rd.data.bits.data               // ME数据写入指令队列


  // instruction queues
  io.inst.ld.valid := dec.io.isLoad & io.inst.ld.ready & state === sSplit     // Load通道有效=解码为Load & Load就绪 & sSplit
  io.inst.co.valid := dec.io.isCompute & io.inst.co.ready & state === sSplit  // Compute通道有效=解码为Compute & Compute就绪 & sSplit
  io.inst.st.valid := dec.io.isStore & io.inst.st.ready & state === sSplit    // Store通道有效=解码为Store & Store就绪 & sSplit

  assert(!(inst_q.io.deq.valid & state === sDrain) || dec.io.isLoad || dec.io.isCompute || dec.io.isStore,  // 安全检查：出队指令必须是已知类型
    "-F- Fetch: Unknown instruction type")

  io.inst.ld.bits := (inst >> (packInstSelect))(INST_BITS - 1, 0)    // 从inst中选择偏移位置的指令发给Load
  io.inst.co.bits := (inst >> (packInstSelect))(INST_BITS - 1, 0)    // 从inst中选择偏移位置的指令发给Compute
  io.inst.st.bits := (inst >> (packInstSelect))(INST_BITS - 1, 0)    // 从inst中选择偏移位置的指令发给Store

  // check if selected queue is ready
  val deq_sel = Cat(dec.io.isCompute, dec.io.isStore, dec.io.isLoad).asUInt  // 编码当前指令类型：bit2=Compute, bit1=Store, bit0=Load
  val deq_ready =                                            // 查询对应目标通道是否就绪
    MuxLookup(deq_sel,                                       // 根据指令类型选择
      false.B, // default                                    // 默认不就绪
      Array(                                                  // 查表
        "h_01".U -> io.inst.ld.ready,                         // Load → 查Load通道就绪
        "h_02".U -> io.inst.st.ready,                         // Store → 查Store通道就绪
        "h_04".U -> io.inst.co.ready                          // Compute → 查Compute通道就绪
      ))

  // dequeue instruction
  inst_q.io.deq.ready := deq_ready & inst_q.io.deq.valid & state === sDrain  // 出队条件：目标就绪 & 数据有效 & sDrain（第一次赋值，被覆盖）

  deqReady := (state === sDrain)                             // deqReady = sDrain状态
  when(inst_q.io.deq.fire){ inst := inst_q.io.deq.bits}      // 出队握手成功 → 锁存当前指令到inst寄存器
  inst_q.io.deq.ready := deqReady & inst_q.io.deq.valid & state === sDrain  // 出队条件（最终赋值）


  // decode
  dec.io.inst := (inst >> (packInstSelect))(INST_BITS - 1, 0)  // 从inst寄存器中提取当前偏移位置的指令送给解码器

  // debug
  if (debug) {                                                // 调试模式（默认关闭）
    when(state === sIdle && pulse) {                          // 空闲态收到启动脉冲
      printf("[Fetch] Launch\n")                              // 打印启动信息
    }
    // instruction
    when(inst_q.io.deq.fire) {                                // 指令出队时
      when(dec.io.isLoad) {                                   // 解码为Load
        printf("[Fetch] [instruction decode] [L] %x\n", inst_q.io.deq.bits)  // 打印Load指令
      }
      when(dec.io.isCompute) {                                // 解码为Compute
        printf("[Fetch] [instruction decode] [C] %x\n", inst_q.io.deq.bits)  // 打印Compute指令
      }
      when(dec.io.isStore) {                                  // 解码为Store
        printf("[Fetch] [instruction decode] [S] %x\n", inst_q.io.deq.bits)  // 打印Store指令
      }
    }
  }
}
