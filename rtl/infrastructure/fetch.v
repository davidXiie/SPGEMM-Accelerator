//=============================================================================
// File     : fetch.v
// Project  : SPGEMM-Accelerator
// Brief    : Instruction Fetch module - reads instructions from DRAM via AXI,
//           decodes opcode, dispatches to Load/Compute/Scheduler/Store queues.
//           Reusable from old SPMM accelerator (remapped from Fetch.scala)
//=============================================================================

`include "defines.vh"
`include "isa.vh"

module fetch #(
    parameter integer INST_QUEUE_DEPTH = 16,
    parameter integer INST_QUEUE_DEPTH_LOG = 4
) (
    // Control
    input  wire                      launch,
    input  wire [`AXI_ADDR_WIDTH-1:0] ins_baddr,
    input  wire [15:0]               ins_count,

    // AXI Read Master (to fetch instructions from DRAM)
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,

    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,

    // Dispatch: Load instruction
    output wire                      ld_inst_valid,
    input  wire                      ld_inst_ready,
    output wire [`INST_WIDTH-1:0]    ld_inst,

    // Dispatch: SpGEMM instruction
    output wire                      sp_inst_valid,
    input  wire                      sp_inst_ready,
    output wire [`INST_WIDTH-1:0]    sp_inst,

    // Dispatch: Store instruction
    output wire                      st_inst_valid,
    input  wire                      st_inst_ready,
    output wire [`INST_WIDTH-1:0]    st_inst,

    // Dispatch: Scheduler instruction (new for SPGEMM)
    output wire                      sch_inst_valid,
    input  wire                      sch_inst_ready,
    output wire [`INST_WIDTH-1:0]    sch_inst,

    input  wire                      aclk,
    input  wire                      aresetn
);

    // Constants
    localparam integer INS_PER_TRANSFER = `AXI_DATA_WIDTH / `INST_WIDTH;  // 512/256 = 2
    localparam integer INS_PER_TRANSFER_LOG = 1;  // log2(2)

    // Instruction queue: stores AXI beats (each containing 2 instructions)
    reg  [`AXI_DATA_WIDTH-1:0] inst_queue [0:INST_QUEUE_DEPTH-1];
    reg  [INST_QUEUE_DEPTH_LOG:0] wr_ptr, rd_ptr;
    wire [INST_QUEUE_DEPTH_LOG:0] queue_count;
    wire queue_empty, queue_full;

    // State machine
    localparam S_IDLE      = 3'd0;
    localparam S_READ_CMD  = 3'd1;
    localparam S_READ_DATA = 3'd2;
    localparam S_DRAIN     = 3'd3;
    localparam S_SPLIT     = 3'd4;

    reg [2:0] state, state_next;

    // Counters
    reg [`AXI_ADDR_WIDTH-1:0] raddr;
    reg [7:0]  rlen;
    reg [7:0]  ilen;
    reg [15:0] xrem;
    reg [15:0] xsize;
    reg [15:0] xmax;

    // Split state
    reg [INS_PER_TRANSFER_LOG:0] pack_sel;  // which instruction in the AXI beat

    // Instruction decode wires
    wire [2:0] inst_opcode;
    wire [2:0] inst_memid;
    wire is_load, is_store, is_spgemm, is_sched, is_finish;

    // Current instruction being split
    wire [`AXI_DATA_WIDTH-1:0] inst_pack;
    wire [`INST_WIDTH-1:0]    cur_inst;

    assign queue_count = (wr_ptr >= rd_ptr) ? (wr_ptr - rd_ptr) :
                         (INST_QUEUE_DEPTH + wr_ptr - rd_ptr);
    assign queue_empty = (queue_count == 0);
    assign queue_full  = (queue_count >= INST_QUEUE_DEPTH - 1);

    // Launch edge detection
    reg launch_r;
    wire launch_pulse;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) launch_r <= 1'b0;
        else          launch_r <= launch;
    end
    assign launch_pulse = launch && !launch_r;

    // Derived
    wire [15:0] xsize_comb = (ins_count >> INS_PER_TRANSFER_LOG) - 1;
    wire [15:0] xmax_val   = (1 << `AXI_LEN_WIDTH);

    // Instruction opcode decode
    assign inst_opcode = cur_inst[2:0];
    assign inst_memid  = cur_inst[5:3];
    assign is_load    = (inst_opcode == `OP_LOAD);
    assign is_store   = (inst_opcode == `OP_STORE);
    assign is_spgemm  = (inst_opcode == `OP_SPGEMM);
    assign is_sched   = (inst_opcode == `OP_SCHED);
    assign is_finish  = (inst_opcode == `OP_FINISH);

    // Current instruction
    assign inst_pack  = inst_queue[rd_ptr];
    assign cur_inst   = inst_pack[pack_sel*`INST_WIDTH +: `INST_WIDTH];

    // Queue write
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_ptr <= 0;
        end else if (state == S_READ_DATA && m_axi_rvalid && m_axi_rready && !queue_full) begin
            inst_queue[wr_ptr[INST_QUEUE_DEPTH_LOG-1:0]] <= m_axi_rdata;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    // Queue read
    reg rd_ptr_inc;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rd_ptr <= 0;
        end else if (rd_ptr_inc) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // State machine
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state  <= S_IDLE;
            raddr  <= 0;
            rlen   <= 0;
            ilen   <= 0;
            xrem   <= 0;
            xsize  <= 0;
            xmax   <= 0;
            pack_sel <= 0;
        end else begin
            state <= state_next;

            case (state)
                S_IDLE: begin
                    if (launch_pulse) begin
                        xsize <= xsize_comb;
                        xmax  <= xmax_val;
                        if (xsize_comb < xmax_val) begin
                            rlen <= xsize_comb[7:0];
                            ilen <= xsize_comb[7:0];
                            xrem <= 0;
                        end else begin
                            rlen <= xmax_val - 1;
                            ilen <= xmax_val - 1;
                            xrem <= xsize_comb - xmax_val;
                        end
                    end
                end
                S_READ_CMD: begin
                    // wait for arready
                end
                S_READ_DATA: begin
                    // increment handled in queue write
                end
                S_DRAIN: begin
                    if (queue_empty) begin
                        if (xrem == 0) begin
                            // done
                        end else if (xrem < xmax_val) begin
                            rlen <= xrem[7:0];
                            ilen <= xrem[7:0];
                            xrem <= 0;
                        end else begin
                            rlen <= xmax_val - 1;
                            ilen <= xmax_val - 1;
                            xrem <= xrem - xmax_val;
                        end
                    end else begin
                        pack_sel <= 0;
                    end
                end
                S_SPLIT: begin
                    if (ld_inst_valid && ld_inst_ready ||
                        sp_inst_valid && sp_inst_ready ||
                        st_inst_valid && st_inst_ready ||
                        sch_inst_valid && sch_inst_ready) begin
                        if (pack_sel == INS_PER_TRANSFER - 1) begin
                            pack_sel <= 0;
                        end else begin
                            pack_sel <= pack_sel + 1'b1;
                        end
                    end
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE: begin
                if (launch_pulse)
                    state_next = S_READ_CMD;
            end
            S_READ_CMD: begin
                if (m_axi_arready)
                    state_next = S_READ_DATA;
            end
            S_READ_DATA: begin
                if (queue_count >= ilen)
                    state_next = S_DRAIN;
            end
            S_DRAIN: begin
                if (queue_empty) begin
                    if (xrem == 0)
                        state_next = S_IDLE;
                    else
                        state_next = S_READ_CMD;
                end else if (!is_finish) begin
                    state_next = S_SPLIT;
                end
            end
            S_SPLIT: begin
                if ((ld_inst_valid && ld_inst_ready) ||
                    (sp_inst_valid && sp_inst_ready) ||
                    (st_inst_valid && st_inst_ready) ||
                    (sch_inst_valid && sch_inst_ready)) begin
                    if (pack_sel == INS_PER_TRANSFER - 1)
                        state_next = S_DRAIN;
                end
            end
        endcase
    end

    // Queue read increment
    always @(*) begin
        rd_ptr_inc = 1'b0;
        if (state == S_SPLIT && pack_sel == INS_PER_TRANSFER - 1) begin
            if ((ld_inst_valid && ld_inst_ready) ||
                (sp_inst_valid && sp_inst_ready) ||
                (st_inst_valid && st_inst_ready) ||
                (sch_inst_valid && sch_inst_ready))
                rd_ptr_inc = 1'b1;
        end
    end

    // AXI read command
    assign m_axi_arvalid = (state == S_READ_CMD);
    assign m_axi_araddr  = raddr;
    assign m_axi_arlen   = rlen;
    assign m_axi_rready  = (state == S_READ_DATA) && !queue_full;

    // Update raddr on batch boundaries
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            raddr <= 0;
        end else if (state == S_IDLE && launch_pulse) begin
            raddr <= ins_baddr;
        end else if (state == S_DRAIN && queue_empty && xrem != 0) begin
            raddr <= raddr + (xmax_val * (`AXI_DATA_WIDTH / 8));
        end
    end

    // Dispatch
    wire split_valid = (state == S_SPLIT) && !is_finish;

    assign ld_inst_valid  = split_valid && is_load;
    assign sp_inst_valid  = split_valid && is_spgemm;
    assign st_inst_valid  = split_valid && is_store;
    assign sch_inst_valid = split_valid && is_sched;

    assign ld_inst   = cur_inst;
    assign sp_inst   = cur_inst;
    assign st_inst   = cur_inst;
    assign sch_inst  = cur_inst;

endmodule
