//=============================================================================
// File     : load.v
// Project  : SPGEMM-Accelerator
// Brief    : Load module - load A/B CSR data from DRAM to GlobalBuffer via AXI.
//           Reusable from old SPMM accelerator (remapped from Load.scala)
//=============================================================================

`include "defines.vh"

module load #(
    parameter integer INST_QUEUE_DEPTH = 4,
    parameter integer DATA_QUEUE_DEPTH = 16
) (
    // Instruction input (from Fetch)
    input  wire                      inst_valid,
    output wire                      inst_ready,
    input  wire [`INST_WIDTH-1:0]    inst_data,

    // Control
    input  wire                      ext_valid,  // from core state machine
    output wire                      done,

    // AXI Read Master
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,

    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,

    // Write to GlobalBuffer
    output wire                      gbuf_wr_en,
    output wire [`GBUF_DEPTH_LOG-1:0] gbuf_wr_addr,
    output wire [`AXI_DATA_WIDTH-1:0] gbuf_wr_data,

    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam STATE_IDLE          = 3'd0;
    localparam STATE_READ_CMD      = 3'd1;
    localparam STATE_READ_DATA     = 3'd2;
    localparam STATE_DELAY         = 3'd3;

    reg [2:0] state, state_next;
    reg done_reg;

    // Instruction storage
    reg [`INST_WIDTH-1:0] stored_inst;
    wire [`AXI_ADDR_WIDTH-1:0] dram_offset;
    wire [15:0] sram_offset;
    wire [15:0] xsize;

    load_decode u_decode (
        .inst        (stored_inst),
        .dram_offset (dram_offset),
        .sram_offset (sram_offset),
        .xsize       (xsize),
        .mem_id      ()
    );

    // Transfer calculation
    wire [15:0] n_block_per_transfer = `AXI_DATA_WIDTH / `DATA_WIDTH;  // 512/16 = 32 FP16 elements per beat
    wire [15:0] n_block_per_transfer_log = 5;  // log2(32)
    wire [15:0] transfer_total = ((xsize - 1) >> n_block_per_transfer_log) + 1;

    reg [`AXI_ADDR_WIDTH-1:0] raddr;
    reg [7:0]  rlen;
    reg [7:0]  rlen_rem;
    reg [15:0] transfer_rem;
    reg [15:0] saddr;
    reg [15:0] max_transfer;

    // Data queue
    reg [DATA_QUEUE_DEPTH-1:0][`AXI_DATA_WIDTH-1:0] data_q;
    reg [DATA_QUEUE_DEPTH-1:0][`GBUF_DEPTH_LOG-1:0] addr_q;
    reg [4:0] data_wr_ptr, data_rd_ptr;
    wire data_q_empty, data_q_full;
    wire [4:0] data_q_count;

    assign data_q_count = (data_wr_ptr >= data_rd_ptr) ?
                          (data_wr_ptr - data_rd_ptr) :
                          (DATA_QUEUE_DEPTH + data_wr_ptr - data_rd_ptr);
    assign data_q_empty = (data_q_count == 0);
    assign data_q_full  = (data_q_count >= DATA_QUEUE_DEPTH - 1);

    // Instruction queue
    reg inst_q_valid;
    wire inst_q_ready;
    reg inst_start;

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

    // Start
    always @(posedge aclk) begin
        inst_start <= inst_q_valid && inst_q_ready;
    end

    // State machine
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state   <= STATE_IDLE;
            done_reg <= 1'b0;
            raddr   <= 0;
            saddr   <= 0;
            rlen    <= 0;
            rlen_rem <= 0;
            transfer_rem <= 0;
            max_transfer <= 0;
        end else begin
            state <= state_next;
            case (state)
                STATE_IDLE: begin
                    done_reg <= 1'b0;
                    if (inst_start) begin
                        max_transfer <= (1 << `AXI_LEN_WIDTH);
                        if (xsize == 0) begin
                            state <= STATE_DELAY;
                        end else begin
                            if (transfer_total < max_transfer) begin
                                rlen    <= transfer_total[7:0] - 1;
                                rlen_rem <= transfer_total[7:0] - 1;
                                transfer_rem <= 0;
                            end else begin
                                rlen    <= max_transfer[7:0] - 1;
                                rlen_rem <= max_transfer[7:0] - 1;
                                transfer_rem <= transfer_total - max_transfer;
                            end
                            raddr <= dram_offset;
                            saddr <= sram_offset;
                        end
                    end
                end
                STATE_READ_CMD: begin
                    // wait for arready
                end
                STATE_READ_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        saddr <= saddr + (`AXI_DATA_WIDTH / 8);
                        if (rlen_rem == 0) begin
                            if (transfer_rem == 0)
                                state <= STATE_DELAY;
                            else if (transfer_rem < max_transfer) begin
                                rlen <= transfer_rem[7:0] - 1;
                                rlen_rem <= transfer_rem[7:0] - 1;
                                transfer_rem <= 0;
                            end else begin
                                rlen <= max_transfer[7:0] - 1;
                                rlen_rem <= max_transfer[7:0] - 1;
                                transfer_rem <= transfer_rem - max_transfer;
                            end
                            raddr <= raddr + (max_transfer * (`AXI_DATA_WIDTH / 8));
                        end else begin
                            rlen_rem <= rlen_rem - 1;
                        end
                    end
                end
                STATE_DELAY: begin
                    if (data_q_empty) begin
                        done_reg <= 1'b1;
                        state <= STATE_IDLE;
                    end
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        state_next = state;
        case (state)
            STATE_IDLE: begin
                if (inst_start) begin
                    if (xsize == 0)
                        state_next = STATE_DELAY;
                    else
                        state_next = STATE_READ_CMD;
                end
            end
            STATE_READ_CMD: begin
                if (data_q_empty && m_axi_arready)
                    state_next = STATE_READ_DATA;
            end
            STATE_READ_DATA: begin
                // handled in sequential block
            end
            STATE_DELAY: begin
                if (data_q_empty && done_reg)
                    state_next = STATE_IDLE;
            end
        endcase
    end

    // AXI read command
    assign m_axi_arvalid = (state == STATE_READ_CMD) && data_q_empty;
    assign m_axi_araddr  = raddr;
    assign m_axi_arlen   = rlen;
    assign m_axi_rready  = (state == STATE_READ_DATA) && !data_q_full;

    // Data queue write (from AXI R channel)
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            data_wr_ptr <= 0;
        end else begin
            if (state == STATE_READ_DATA && m_axi_rvalid && m_axi_rready) begin
                data_q[data_wr_ptr[3:0]] <= m_axi_rdata;
                addr_q[data_wr_ptr[3:0]] <= saddr[`GBUF_DEPTH_LOG-1:0];
                data_wr_ptr <= data_wr_ptr + 1'b1;
            end
        end
    end

    // Data queue read (to GlobalBuffer)
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            data_rd_ptr <= 0;
        end else begin
            if (gbuf_wr_en)
                data_rd_ptr <= data_rd_ptr + 1'b1;
        end
    end

    assign gbuf_wr_en   = !data_q_empty;
    assign gbuf_wr_addr = addr_q[data_rd_ptr[3:0]];
    assign gbuf_wr_data = data_q[data_rd_ptr[3:0]];

    assign done = done_reg;

endmodule
