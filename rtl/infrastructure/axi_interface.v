//=============================================================================
// File     : axi_interface.v
// Project  : SPGEMM-Accelerator
// Brief    : AXI-Full bus interface definitions and MUX for read/write clients
//           Reusable from old SPMM accelerator (remapped from Chisel)
//=============================================================================

`include "defines.vh"

//=============================================================================
// AXI-Lite Slave: Control Register interface (CPU side)
//=============================================================================
module cr_slave #(
    parameter integer REG_COUNT = 6,
    parameter integer REG_BITS  = 32
) (
    // AXI-Lite Write Address Channel
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [15:0]               s_axi_awaddr,

    // AXI-Lite Write Data Channel
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    input  wire [REG_BITS-1:0]       s_axi_wdata,
    input  wire [REG_BITS/8-1:0]     s_axi_wstrb,

    // AXI-Lite Write Response Channel
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,
    output wire [1:0]                s_axi_bresp,

    // AXI-Lite Read Address Channel
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,
    input  wire [15:0]               s_axi_araddr,

    // AXI-Lite Read Data Channel
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,
    output wire [REG_BITS-1:0]       s_axi_rdata,
    output wire [1:0]                s_axi_rresp,

    // Register read/write from accelerator core
    output wire [REG_COUNT*REG_BITS-1:0] reg_out,
    input  wire [REG_COUNT*REG_BITS-1:0] reg_in,
    output wire [REG_COUNT-1:0]          reg_wr_pulse,
    output wire [REG_COUNT-1:0]          reg_rd_valid
);

    localparam integer ADDR_BITS = $clog2(REG_COUNT) + 2; // byte address

    // Internal signals
    reg [1:0] write_state, read_state;
    localparam W_IDLE = 2'b00, W_DATA = 2'b01, W_RESP = 2'b10;
    localparam R_IDLE = 2'b00, R_DATA = 2'b01;

    reg [REG_BITS-1:0] waddr_reg;
    reg [REG_BITS-1:0] raddr_reg;
    reg [REG_COUNT-1:0] wr_pulse;
    reg [REG_COUNT-1:0] rd_valid;
    reg [`AXI_ADDR_WIDTH-1:0] raddr;

    // Register address decode
    wire [ADDR_BITS-1:0] waddr_word = s_axi_awaddr[ADDR_BITS-1:0] >> 2;
    wire [ADDR_BITS-1:0] raddr_word = s_axi_araddr[ADDR_BITS-1:0] >> 2;

    // Write FSM
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            write_state <= W_IDLE;
            waddr_reg <= 0;
            wr_pulse <= 0;
        end else begin
            wr_pulse <= 0;
            case (write_state)
                W_IDLE: begin
                    if (s_axi_awvalid && s_axi_awready) begin
                        waddr_reg <= s_axi_awaddr;
                        write_state <= W_DATA;
                    end
                end
                W_DATA: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        write_state <= W_RESP;
                        if (waddr_reg[ADDR_BITS-1:2] < REG_COUNT)
                            wr_pulse[waddr_reg[ADDR_BITS-1:2]] <= 1'b1;
                    end
                end
                W_RESP: begin
                    if (s_axi_bvalid && s_axi_bready)
                        write_state <= W_IDLE;
                end
            endcase
        end
    end

    assign s_axi_awready = (write_state == W_IDLE);
    assign s_axi_wready  = (write_state == W_DATA);
    assign s_axi_bvalid  = (write_state == W_RESP);
    assign s_axi_bresp   = 2'b00;

    // Read FSM
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            read_state <= R_IDLE;
            raddr_reg <= 0;
            rd_valid <= 0;
        end else begin
            rd_valid <= 0;
            case (read_state)
                R_IDLE: begin
                    if (s_axi_arvalid && s_axi_arready) begin
                        raddr_reg <= s_axi_araddr;
                        read_state <= R_DATA;
                        if (s_axi_araddr[ADDR_BITS-1:2] < REG_COUNT)
                            rd_valid[s_axi_araddr[ADDR_BITS-1:2]] <= 1'b1;
                    end
                end
                R_DATA: begin
                    if (s_axi_rvalid && s_axi_rready)
                        read_state <= R_IDLE;
                end
            endcase
        end
    end

    wire [REG_BITS-1:0] rdata_mux = reg_in[raddr_reg[ADDR_BITS-1:2]*REG_BITS +: REG_BITS];

    assign s_axi_arready = (read_state == R_IDLE);
    assign s_axi_rvalid  = (read_state == R_DATA);
    assign s_axi_rdata   = rdata_mux;
    assign s_axi_rresp   = 2'b00;

    assign reg_out       = {REG_COUNT*REG_BITS{1'b0}}; // write-through handled by wr_pulse
    assign reg_wr_pulse  = wr_pulse;
    assign reg_rd_valid  = rd_valid;

endmodule


//=============================================================================
// AXI-Full Read Master MUX: Share AXI read channel among multiple clients
//=============================================================================
module axi_read_mux #(
    parameter integer N_CLIENTS = 2
) (
    // AXI Read Address Channel (to MEM)
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,
    output wire [`AXI_ID_WIDTH-1:0]  m_axi_arid,
    output wire [2:0]                m_axi_arsize,
    output wire [1:0]                m_axi_arburst,

    // AXI Read Data Channel (from MEM)
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [1:0]                m_axi_rresp,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,
    input  wire [`AXI_ID_WIDTH-1:0]  m_axi_rid,

    // Client interfaces
    input  wire [N_CLIENTS-1:0]      s_axi_arvalid,
    output wire [N_CLIENTS-1:0]      s_axi_arready,
    input  wire [N_CLIENTS*`AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire [N_CLIENTS*8-1:0]    s_axi_arlen,

    output wire [N_CLIENTS-1:0]      s_axi_rvalid,
    input  wire [N_CLIENTS-1:0]      s_axi_rready,
    output wire [N_CLIENTS*`AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output wire [N_CLIENTS-1:0]      s_axi_rlast,

    input  wire                      aclk,
    input  wire                      aresetn
);

    genvar i;
    integer j;

    // Priority encoder: lower index = higher priority
    wire [N_CLIENTS-1:0] req_vec = s_axi_arvalid;
    reg  [N_CLIENTS-1:0] mask;

    // Demux tag
    reg [3:0] active_client;

    // Request state
    localparam ARB_IDLE = 1'b0, ARB_BUSY = 1'b1;
    reg arb_state;

    // Read request tracking
    reg [`AXI_ADDR_WIDTH-1:0] client_addr [0:N_CLIENTS-1];
    reg [7:0]                client_len  [0:N_CLIENTS-1];

    wire [N_CLIENTS-1:0] grant;

    // Simple round-robin / priority arbiter
    assign grant[0] = req_vec[0];
    generate
        for (i = 1; i < N_CLIENTS; i = i + 1) begin : gen_grant
            assign grant[i] = req_vec[i] && !(|req_vec[i-1:0]);
        end
    endgenerate

    // Find active client
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            active_client <= 0;
            arb_state <= ARB_IDLE;
        end else begin
            case (arb_state)
                ARB_IDLE: begin
                    if (|req_vec) begin
                        for (j = 0; j < N_CLIENTS; j = j + 1) begin
                            if (grant[j]) active_client <= j;
                        end
                        arb_state <= ARB_BUSY;
                    end
                end
                ARB_BUSY: begin
                    if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
                        arb_state <= ARB_IDLE;
                    end
                end
            endcase
        end
    end

    // Grant signals
    generate
        for (i = 0; i < N_CLIENTS; i = i + 1) begin : gen_grant_out
            assign s_axi_arready[i] = (arb_state == ARB_IDLE) && grant[i];
        end
    endgenerate

    // MUX AR channel
    wire [`AXI_ADDR_WIDTH-1:0] mux_addr;
    wire [7:0]                 mux_len;
    assign mux_addr = s_axi_araddr[active_client*`AXI_ADDR_WIDTH +: `AXI_ADDR_WIDTH];
    assign mux_len  = s_axi_arlen[active_client*8 +: 8];

    assign m_axi_arvalid = (arb_state == ARB_IDLE) && (|req_vec);
    assign m_axi_araddr  = mux_addr;
    assign m_axi_arlen   = mux_len;
    assign m_axi_arid    = {1'b0, active_client};
    assign m_axi_arsize  = 3'b110;  // 64 bytes per beat
    assign m_axi_arburst = 2'b01;   // INCR

    // DEMUX R channel
    generate
        for (i = 0; i < N_CLIENTS; i = i + 1) begin : gen_r_demux
            assign s_axi_rvalid[i] = m_axi_rvalid && (active_client == i);
            assign s_axi_rdata[i*`AXI_DATA_WIDTH +: `AXI_DATA_WIDTH] = m_axi_rdata;
            assign s_axi_rlast[i]  = m_axi_rlast;
        end
    endgenerate

    assign m_axi_rready = |s_axi_rready;

endmodule


//=============================================================================
// AXI-Full Write Master: Single write client
//=============================================================================
module axi_write_master (
    // AXI Write Address Channel
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,
    output wire [`AXI_ID_WIDTH-1:0]  m_axi_awid,

    // AXI Write Data Channel
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    output wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,

    // AXI Write Response Channel
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    input  wire [1:0]                m_axi_bresp,
    input  wire [`AXI_ID_WIDTH-1:0]  m_axi_bid,

    // Client interface
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [`AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire [7:0]                s_axi_awlen,

    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    input  wire [`AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input  wire [`AXI_STRB_WIDTH-1:0] s_axi_wstrb,
    input  wire                      s_axi_wlast,

    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,

    input  wire                      aclk,
    input  wire                      aresetn
);

    // Pass-through AXI write (unified client, no arbitration needed)
    assign m_axi_awvalid = s_axi_awvalid;
    assign s_axi_awready = m_axi_awready;
    assign m_axi_awaddr  = s_axi_awaddr;
    assign m_axi_awlen   = s_axi_awlen;
    assign m_axi_awid    = 4'b0;

    assign m_axi_wvalid  = s_axi_wvalid;
    assign s_axi_wready  = m_axi_wready;
    assign m_axi_wdata   = s_axi_wdata;
    assign m_axi_wstrb   = s_axi_wstrb;
    assign m_axi_wlast   = s_axi_wlast;

    assign s_axi_bvalid  = m_axi_bvalid;
    assign m_axi_bready  = s_axi_bready;

endmodule
