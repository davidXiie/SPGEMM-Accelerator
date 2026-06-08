package gcn.core

import chisel3._
import chisel3.stage._
import chisel3.util._
import vta.util.config._

class Wrapper(implicit p: Parameters) extends RawModule{

  val ap_clk = IO(Input(Clock()))     // 时钟
  val ap_rst_n = IO(Input(Bool()))    // 低有效复位
  val hp = p(AccKey).hostParams    
  val s_axi_control = IO(new XilinxAXILiteClient(hp))    // CPU 控制口 (32-bit)
  val m_axi_gmem = IO(new XilinxAXIMaster(p(AccKey).memParams))    // DDR 数据口 (512-bit)

  val cr = withClockAndReset(clock = ap_clk, reset = ~ap_rst_n) {       // 控制寄存器
    Module(new CR)                                
  }
  val core = withClockAndReset(clock = ap_clk, reset = ~ap_rst_n) {       // 核心加速器
    Module(new Core)
  }
  val me = withClockAndReset(clock = ap_clk, reset = ~ap_rst_n) {       // 内存引擎
    Module(new ME)
  }
  core.io.cr <> cr.io.cr                                          // CR 直接控制 Core 的启停
  me.io.me <> core.io.me                                          // Core 的所有内存读写都通过 ME


  // ========== AXI4 写地址通道 (AW) ==========
  m_axi_gmem.AWVALID := me.io.mem.aw.valid          // 写地址有效
  me.io.mem.aw.ready := m_axi_gmem.AWREADY           // 写地址就绪
  m_axi_gmem.AWADDR := me.io.mem.aw.bits.addr         // 写地址
  m_axi_gmem.AWID := me.io.mem.aw.bits.id             // 写事务ID
  m_axi_gmem.AWUSER := me.io.mem.aw.bits.user          // 写事务用户侧带信号
  m_axi_gmem.AWLEN := me.io.mem.aw.bits.len           // 突发长度
  m_axi_gmem.AWSIZE := me.io.mem.aw.bits.size         // 突发大小
  m_axi_gmem.AWBURST := me.io.mem.aw.bits.burst       // 突发类型
  m_axi_gmem.AWLOCK := me.io.mem.aw.bits.lock         // 锁类型
  m_axi_gmem.AWCACHE := me.io.mem.aw.bits.cache       // 缓存属性
  m_axi_gmem.AWPROT := me.io.mem.aw.bits.prot         // 保护类型
  m_axi_gmem.AWQOS := me.io.mem.aw.bits.qos           // QoS标识
  m_axi_gmem.AWREGION := me.io.mem.aw.bits.region     // 区域标识

  // ========== AXI4 写数据通道 (W) ==========
  m_axi_gmem.WVALID := me.io.mem.w.valid             // 写数据有效
  // me.io.mem.w.ready := m_axi_gmem.WREADY
  me.io.mem.w.ready := true.B                         // 写数据就绪（恒为1）
  m_axi_gmem.WDATA := me.io.mem.w.bits.data           // 写数据
  m_axi_gmem.WSTRB := me.io.mem.w.bits.strb           // 写字节使能
  m_axi_gmem.WLAST := me.io.mem.w.bits.last           // 写猝发最后一拍标志
  m_axi_gmem.WID := me.io.mem.w.bits.id               // 写数据ID
  m_axi_gmem.WUSER := me.io.mem.w.bits.user            // 写数据用户侧带信号

  // ========== AXI4 写响应通道 (B) ==========
  // me.io.mem.b.valid := m_axi_gmem.BVALID
  me.io.mem.b.valid := true.B                         // 写响应有效（恒为1）
  m_axi_gmem.BREADY := me.io.mem.b.valid              // 写响应就绪
  me.io.mem.b.bits.resp := m_axi_gmem.BRESP            // 写响应状态
  me.io.mem.b.bits.id := m_axi_gmem.BID                // 写响应ID
  me.io.mem.b.bits.user := m_axi_gmem.BUSER             // 写响应用户侧带信号

  // ========== AXI4 读地址通道 (AR) ==========
  m_axi_gmem.ARVALID := me.io.mem.ar.valid            // 读地址有效
  me.io.mem.ar.ready := m_axi_gmem.ARREADY            // 读地址就绪
  m_axi_gmem.ARADDR := me.io.mem.ar.bits.addr           // 读地址
  m_axi_gmem.ARID := me.io.mem.ar.bits.id              // 读事务ID
  m_axi_gmem.ARUSER := me.io.mem.ar.bits.user           // 读事务用户侧带信号
  m_axi_gmem.ARLEN := me.io.mem.ar.bits.len            // 突发长度
  m_axi_gmem.ARSIZE := me.io.mem.ar.bits.size          // 突发大小
  m_axi_gmem.ARBURST := me.io.mem.ar.bits.burst        // 突发类型
  m_axi_gmem.ARLOCK := me.io.mem.ar.bits.lock          // 锁类型
  m_axi_gmem.ARCACHE := me.io.mem.ar.bits.cache        // 缓存属性
  m_axi_gmem.ARPROT := me.io.mem.ar.bits.prot          // 保护类型
  m_axi_gmem.ARQOS := me.io.mem.ar.bits.qos            // QoS标识
  m_axi_gmem.ARREGION := me.io.mem.ar.bits.region      // 区域标识

  // ========== AXI4 读数据通道 (R) ==========
  me.io.mem.r.valid := m_axi_gmem.RVALID              // 读数据有效
  m_axi_gmem.RREADY := me.io.mem.r.ready               // 读数据就绪
  me.io.mem.r.bits.data := m_axi_gmem.RDATA            // 读数据
  me.io.mem.r.bits.resp := m_axi_gmem.RRESP            // 读响应状态
  me.io.mem.r.bits.last := m_axi_gmem.RLAST            // 读猝发最后一拍标志
  me.io.mem.r.bits.id := m_axi_gmem.RID                // 读数据ID
  me.io.mem.r.bits.user := m_axi_gmem.RUSER             // 读数据用户侧带信号

  // ========== AXI4-Lite 写地址通道 (AW) — CPU控制口 ==========
  cr.io.host.aw.valid := s_axi_control.AWVALID         // 写地址有效
  s_axi_control.AWREADY := cr.io.host.aw.ready          // 写地址就绪
  cr.io.host.aw.bits.addr := s_axi_control.AWADDR       // 写地址

  // ========== AXI4-Lite 写数据通道 (W) ==========
  cr.io.host.w.valid := s_axi_control.WVALID           // 写数据有效
  s_axi_control.WREADY := cr.io.host.w.ready            // 写数据就绪
  cr.io.host.w.bits.data := s_axi_control.WDATA         // 写数据
  cr.io.host.w.bits.strb := s_axi_control.WSTRB          // 写字节使能

  // ========== AXI4-Lite 写响应通道 (B) ==========
  s_axi_control.BVALID := cr.io.host.b.valid           // 写响应有效
  cr.io.host.b.ready := s_axi_control.BREADY            // 写响应就绪
  s_axi_control.BRESP := cr.io.host.b.bits.resp          // 写响应状态

  // ========== AXI4-Lite 读地址通道 (AR) ==========
  cr.io.host.ar.valid := s_axi_control.ARVALID         // 读地址有效
  s_axi_control.ARREADY := cr.io.host.ar.ready          // 读地址就绪
  cr.io.host.ar.bits.addr := s_axi_control.ARADDR       // 读地址

  // ========== AXI4-Lite 读数据通道 (R) ==========
  s_axi_control.RVALID := cr.io.host.r.valid           // 读数据有效
  cr.io.host.r.ready := s_axi_control.RREADY            // 读数据就绪
  s_axi_control.RDATA := cr.io.host.r.bits.data         // 读数据
  s_axi_control.RRESP := cr.io.host.r.bits.resp          // 读响应状态

}

// Executable object generate verilog
object DefaultTemplate extends App {
  implicit val p: Parameters = new ZcuConfig
  val chiselStage = new chisel3.stage.ChiselStage
  chiselStage.execute(
    Array(
      "-e", "mverilog", 
      "--target-dir", "verilog"), 
    Seq(ChiselGeneratorAnnotation(() => new Wrapper()))
  )
}