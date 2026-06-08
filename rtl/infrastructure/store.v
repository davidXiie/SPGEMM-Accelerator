//=============================================================================
// File     : store.v
// Project  : SPGEMM-Accelerator
// Brief    : Store module - write C CSR results from OutputScratchpad to DRAM.
//           Reusable from old SPMM accelerator (remapped from Store.scala)
//=============================================================================

`include "defines.vh"

module store #(
    parameter integer INST_QUEUE_DEPTH = 4
) (
    // Instruction input (from Fetch)
    input  wire                      inst_valid,
    output wire                      inst_ready,
    input  wire [`INST_WIDTH-1:0]    inst_data,

    // Control
    input  wire                      ext_valid,  // from core state machine
    output wire                      done,

    // Read from OutputScratchpad
    input  wire                      osp_rd_en,
    input  wire [`OUTBUF_DEPTH_LOG-1:0] osp_rd_addr,
    input  wire [`AXI_DATA_WIDTH-1:0]  osp_rd_data,
    input  wire                      osp_rd_valid,

    // AXI Write Master
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,

    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    output wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,

    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    input  wire [1:0]                m_axi_bresp,

    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam STATE_IDLE       = 3'd0;
    localparam STATE_WRITE_CMD  = 3'd1;
    localparam STATE_READ_MEM   = 3'd2;
    localparam STATE_WRITE_DATA = 3'd3;
    localparam STATE_WRITE_ACK  = 3'd4;

    reg [2:0] state;
    reg done_reg;

    // Instruction storage
    reg [`INST_WIDTH-1:0] stored_inst;
    wire [`AXI_ADDR_WIDTH-1:0] dram_offset;
    wire [31:0] sram_offset;
    wire [31:0] xsize;

    store_decode u_decode (
        .inst        (stored_inst),
        .dram_offset (dram_offset),
        .sram_offset (sram_offset),
        .xsize       (xsize),
        .mem_id      ()
    );

    // Transfer calculation
    wire [31:0] n_block_per_transfer = `AXI_DATA_WIDTH / `DATA_WIDTH;  // 512/16 = 32
    wire [31:0] n_block_per_transfer_log = 5;  // log2(32)
    wire [31:0] transfer_total = ((xsize - 1) >> n_block_per_transfer_log) + 1;

    reg [`AXI_ADDR_WIDTH-1:0] waddr;
    reg [7:0]  wlen;
    reg [7:0]  wcnt;
    reg [31:0] transfer_rem;
    reg [31:0] saddr;
    reg [31:0] max_transfer;
    reg [31:0] max_transfer_bytes;
    reg [31:0] total_bytes;
    reg [31:0] total_bytes_written;
    wire [31:0] total_bytes_rem;
    wire [31:0] curr_bytes;
    wire [31:0] curr_strb;

    // Instruction queue
    reg inst_q_valid;
    wire inst_q_ready;
    reg inst_start;

    assign total_bytes      = xsize << (`DATA_BITS_LOG2);
    assign total_bytes_rem   = total_bytes - total_bytes_written;
    assign curr_bytes        = (total_bytes_rem >= (`AXI_DATA_WIDTH/8)) ?
                               (`AXI_DATA_WIDTH/8) : total_bytes_rem;

    // Generate strobe
    assign curr_strb = (max_transfer_bytes == 0) ? 0 :
                       ({`AXI_STRB_WIDTH{1'b1}} >> (`AXI_STRB_WIDTH - curr_bytes));

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            inst_q_valid <= 1'b0;
        end else begin
            if (inst_valid && inst_ready)
                inst_q_valid <= 1'b1;
            else if (inst_start)
                inst_q_valid <= 1'b0;
        end
    end
    assign inst_ready = !inst_q_valid;
    assign inst_q_ready = (state == STATE_IDLE) && ext_valid;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            stored_inst <= 0;
        end else if (inst_valid && inst_ready) begin
            stored_inst <= inst_data;
        end
    end

    always @(posedge aclk) begin
        inst_start <= inst_q_valid && inst_q_ready;
    end

    // State machine
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state    <= STATE_IDLE;
            done_reg <= 1'b0;
            waddr    <= 0;
            wlen     <= 0;
            wcnt     <= 0;
            transfer_rem <= 0;
            saddr    <= 0;
            max_transfer <= 0;
            max_transfer_bytes <= 0;
            total_bytes_written <= 0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    done_reg <= 1'b0;
                    total_bytes_written <= 0;
                    if (inst_start) begin
                        max_transfer <= (1 << `AXI_LEN_WIDTH);
                        max_transfer_bytes <= max_transfer * (`AXI_DATA_WIDTH / 8);
                        if (xsize == 0) begin
                            done_reg <= 1'b1;
                        end else begin
                            waddr <= dram_offset;
                            saddr <= sram_offset;
                            if (transfer_total < max_transfer) begin
                                wlen <= transfer_total[7:0] - 1;
                                transfer_rem <= 0;
                            end else begin
                                wlen <= max_transfer[7:0] - 1;
                                transfer_rem <= transfer_total - max_transfer;
                            end
                            state <= STATE_WRITE_CMD;
                        end
                    end
                end
                STATE_WRITE_CMD: begin
                    if (m_axi_awready)
                        state <= STATE_READ_MEM;
                end
                STATE_READ_MEM: begin
                    state <= STATE_WRITE_DATA;
                end
                STATE_WRITE_DATA: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        wcnt <= wcnt + 1;
                        total_bytes_written <= total_bytes_written + curr_bytes;
                        if (wcnt == wlen)
                            state <= STATE_WRITE_ACK;
                        else
                            state <= STATE_READ_MEM;
                    end
                end
                STATE_WRITE_ACK: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        if (transfer_rem == 0) begin
                            done_reg <= 1'b1;
                            state <= STATE_IDLE;
                        end else begin
                            waddr <= waddr + max_transfer_bytes;
                            saddr <= saddr + max_transfer_bytes;
                            if (transfer_rem < max_transfer) begin
                                wlen <= transfer_rem[7:0] - 1;
                                transfer_rem <= 0;
                            end else begin
                                wlen <= max_transfer[7:0] - 1;
                                transfer_rem <= transfer_rem - max_transfer;
                            end
                            state <= STATE_WRITE_CMD;
                        end
                    end
                end
            endcase
        end
    end

    // AXI Write Address
    assign m_axi_awvalid = (state == STATE_WRITE_CMD);
    assign m_axi_awaddr  = waddr;
    assign m_axi_awlen   = wlen;

    // AXI Write Data
    assign m_axi_wvalid  = (state == STATE_WRITE_DATA) && osp_rd_valid;
    assign m_axi_wdata   = osp_rd_data;
    assign m_axi_wstrb   = {{(`AXI_STRB_WIDTH-curr_bytes){1'b0}}, {curr_bytes{1'b1}}};
    assign m_axi_wlast   = (wcnt == wlen);

    // AXI Write Response
    assign m_axi_bready  = (state == STATE_WRITE_ACK);

    // Read from output scratchpad
    assign osp_rd_en   = (state == STATE_READ_MEM) || (state == STATE_WRITE_DATA && m_axi_wready);
    assign osp_rd_addr = (state == STATE_WRITE_CMD) ? saddr[`OUTBUF_DEPTH_LOG-1:0] :
                         saddr[`OUTBUF_DEPTH_LOG-1:0];

    assign done = done_reg;

endmodule
