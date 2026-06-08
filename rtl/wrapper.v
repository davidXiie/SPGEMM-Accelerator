//=============================================================================
// File     : wrapper.v
// Project  : SPGEMM-Accelerator
// Brief    : Top-level wrapper - instantiates Core + CR (AXI-Lite slave) +
//           ME (Memory Engine) + AXI interfaces
//=============================================================================

`include "defines.vh"

module spgemm_accelerator (
    // AXI-Lite Slave: CPU control interface
    input  wire                      s_axi_aclk,
    input  wire                      s_axi_aresetn,
    // Write
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [15:0]               s_axi_awaddr,
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    input  wire [31:0]               s_axi_wdata,
    input  wire [3:0]                s_axi_wstrb,
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,
    output wire [1:0]                s_axi_bresp,
    // Read
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,
    input  wire [15:0]               s_axi_araddr,
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,
    output wire [31:0]               s_axi_rdata,
    output wire [1:0]                s_axi_rresp,

    // AXI-Full Master: DRAM interface
    // Read Address
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,
    output wire [`AXI_ID_WIDTH-1:0]  m_axi_arid,
    output wire [2:0]                m_axi_arsize,
    output wire [1:0]                m_axi_arburst,
    // Read Data
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [1:0]                m_axi_rresp,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,
    input  wire [`AXI_ID_WIDTH-1:0]  m_axi_rid,
    // Write Address
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,
    output wire [`AXI_ID_WIDTH-1:0]  m_axi_awid,
    // Write Data
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    output wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,
    // Write Response
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    input  wire [1:0]                m_axi_bresp,
    input  wire [`AXI_ID_WIDTH-1:0]  m_axi_bid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // Control Register signals
    //=========================================================================
    wire [191:0] cr_reg_out;  // 6 × 32-bit regs
    wire [191:0] cr_reg_in;
    wire [5:0]   cr_wr_pulse;
    wire [5:0]   cr_rd_valid;

    // Register map:
    //   Reg 0 (0x00): Launch command (write to start)
    //   Reg 1 (0x04): Instruction base address
    //   Reg 2 (0x08): Instruction count
    //   Reg 3 (0x0C): Status / Finish flag (read)
    //   Reg 4 (0x10): Cycle counter (read)
    //   Reg 5 (0x14): Reserved

    wire cr_launch   = cr_reg_out[31:0];
    wire [`AXI_ADDR_WIDTH-1:0] cr_ins_baddr;
    wire [31:0] cr_ins_count;

    assign cr_ins_baddr = {cr_reg_out[63:32], cr_reg_out[31:0]};
    assign cr_ins_count = cr_reg_out[95:64];

    wire core_finish;

    // CR Reg In: status
    assign cr_reg_in[31:0]   = cr_reg_out[31:0];  // echo launch
    assign cr_reg_in[63:32]  = cr_reg_out[63:32]; // echo ins_baddr
    assign cr_reg_in[95:64]  = cr_reg_out[95:64]; // echo ins_count
    assign cr_reg_in[127:96] = {31'b0, core_finish};
    assign cr_reg_in[159:128] = 32'd0;  // cycle counter placeholder
    assign cr_reg_in[191:160] = 32'd0;

    //=========================================================================
    // CR (AXI-Lite Slave)
    //=========================================================================
    cr_slave #(
        .REG_COUNT(6),
        .REG_BITS(32)
    ) u_cr_slave (
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_awaddr  (s_axi_awaddr),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready  (s_axi_bready),
        .s_axi_bresp   (s_axi_bresp),
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_araddr  (s_axi_araddr),
        .s_axi_rvalid  (s_axi_rvalid),
        .s_axi_rready  (s_axi_rready),
        .s_axi_rdata   (s_axi_rdata),
        .s_axi_rresp   (s_axi_rresp),
        .reg_out       (cr_reg_out),
        .reg_in        (cr_reg_in),
        .reg_wr_pulse  (cr_wr_pulse),
        .reg_rd_valid  (cr_rd_valid)
    );

    //=========================================================================
    // Core (SpGEMM Accelerator)
    //=========================================================================
    core_top u_core_top (
        .cr_launch     (cr_launch[0]),
        .ins_baddr     (cr_ins_baddr),
        .ins_count     (cr_ins_count),
        .cr_finish     (core_finish),
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_awvalid (m_axi_awvalid),
        .m_axi_awready (m_axi_awready),
        .m_axi_awaddr  (m_axi_awaddr),
        .m_axi_awlen   (m_axi_awlen),
        .m_axi_wvalid  (m_axi_wvalid),
        .m_axi_wready  (m_axi_wready),
        .m_axi_wdata   (m_axi_wdata),
        .m_axi_wstrb   (m_axi_wstrb),
        .m_axi_wlast   (m_axi_wlast),
        .m_axi_bvalid  (m_axi_bvalid),
        .m_axi_bready  (m_axi_bready),
        .m_axi_bresp   (m_axi_bresp),
        .cycle_counter (),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // AXI IDs
    assign m_axi_arid   = 4'b0;
    assign m_axi_arsize = 3'b110;  // 64 bytes
    assign m_axi_arburst = 2'b01;  // INCR
    assign m_axi_awid   = 4'b0;

endmodule
