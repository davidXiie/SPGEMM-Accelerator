//=============================================================================
// File     : load.v
// Project  : SPGEMM-Accelerator
// Brief    : Load module - load A/B CSR data from DRAM to GlobalBuffer via AXI.
//           Reusable from old SPMM accelerator (remapped from Load.scala)
//
// Key fix: AXI beat (512-bit) is expanded into 32 × 16-bit element writes.
// Each GlobalBuffer address stores exactly one 16-bit entry.
// This ensures scheduler/pe_decompress read B_row_ptr[k] at addr = base + k
// (instead of base + k/32 with the old 512-bit-wide write bug).
//=============================================================================

`include "defines.vh"

module load #(
    parameter integer INST_QUEUE_DEPTH = 4
) (
    // Instruction input (from Fetch)
    input  wire                      inst_valid,
    output wire                      inst_ready,
    input  wire [`INST_WIDTH-1:0]  inst_data,

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

    // Write to GlobalBuffer (one 16-bit element per address)
    output wire                      gbuf_wr_en,
    output wire [`GBUF_DEPTH_LOG-1:0] gbuf_wr_addr,
    output wire [`DATA_WIDTH-1:0]   gbuf_wr_data,

    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam STATE_IDLE       = 3'd0;
    localparam STATE_READ_CMD   = 3'd1;
    localparam STATE_READ_DATA  = 3'd2;
    localparam STATE_DRAIN_BEAT = 3'd3;  // expand one beat to 32 × 16-bit writes
    localparam STATE_DELAY      = 3'd4;

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
    // N_ELEM_PER_BEAT = 32: each AXI beat carries 32 × 16-bit elements
    wire [15:0] n_elem_per_beat      = `N_ELEM_PER_AXI_BEAT;           // 32
    wire [15:0] n_elem_per_beat_log = `DATA_BITS_LOG2 + `N_MAC_BITS;  // 2+3 = 5, log2(32)
    wire [15:0] n_beats_per_transfer  = ((xsize - 1) >> n_elem_per_beat_log) + 1;

    reg [`AXI_ADDR_WIDTH-1:0] raddr;
    reg [7:0]  rlen;
    reg [7:0]  rlen_rem;
    reg [15:0] transfer_rem;
    reg [15:0] saddr;          // current element address in GlobalBuffer (16-bit entry addr)
    localparam MAX_TRANSFER = (1 << `AXI_LEN_WIDTH);  // max beats per AXI burst (=256)

    //=========================================================================
    // Beat capture & element expansion
    //=========================================================================
    // Holding register: captures one full 512-bit AXI beat
    reg [`AXI_DATA_WIDTH-1:0] beat_reg;
    reg                       beat_valid_r;

    // Element drain counter: 0..31, expands one beat into 32 writes
    // n_elem_per_beat_log = 5, but iverilog doesn't support localparam as width, so hardcode
    reg [4:0] elem_idx;
    wire [`DATA_WIDTH-1:0] elem_data;

    // AXI-side backpressure: stall until current beat is fully drained
    reg axi_busy_r;

    // Element extracted from beat_reg: elem_idx=0 → lower 16 bits
    assign elem_data = beat_reg[elem_idx * `DATA_WIDTH +: `DATA_WIDTH];

    //=========================================================================
    // Instruction queue (minimal, one instruction deep)
    //=========================================================================
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

    always @(posedge aclk) begin
        inst_start <= inst_q_valid && inst_q_ready;
    end

    //=========================================================================
    // State machine
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state       <= STATE_IDLE;
            done_reg    <= 1'b0;
            raddr       <= 0;
            saddr       <= 0;
            rlen        <= 0;
            rlen_rem    <= 0;
            transfer_rem <= 0;
            beat_reg    <= 0;
            beat_valid_r <= 1'b0;
            elem_idx    <= 0;
            axi_busy_r  <= 1'b0;
        end else begin
            state <= state_next;

            case (state)
                STATE_IDLE: begin
                    done_reg     <= 1'b0;
                    axi_busy_r   <= 1'b0;
                    beat_valid_r <= 1'b0;
                    if (inst_start) begin
                        if (xsize == 0) begin
                            state <= STATE_DELAY;
                        end else begin
                            if (n_beats_per_transfer < MAX_TRANSFER) begin
                                rlen        <= n_beats_per_transfer[7:0] - 1;
                                rlen_rem    <= n_beats_per_transfer[7:0] - 1;
                                transfer_rem <= 0;
                            end else begin
                                rlen        <= MAX_TRANSFER[7:0] - 1;
                                rlen_rem    <= MAX_TRANSFER[7:0] - 1;
                                transfer_rem <= n_beats_per_transfer - MAX_TRANSFER;
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
                    // AXI handshake: capture beat, begin expansion
                    if (m_axi_rvalid && m_axi_rready) begin
                        beat_reg     <= m_axi_rdata;
                        beat_valid_r <= 1'b1;
                        axi_busy_r   <= 1'b1;
                        elem_idx     <= 0;
                        // Always drain this beat before accepting the next one.
                        // Next-burst preparation is deferred to STATE_DRAIN_BEAT.
                        state <= STATE_DRAIN_BEAT;
                    end
                end

                STATE_DRAIN_BEAT: begin
                    // Expand one beat (32 × 16-bit) to GlobalBuffer
                    if (gbuf_wr_en) begin
                        saddr   <= saddr + 1;              // each 16-bit element = one addr
                        elem_idx <= elem_idx + 1;
                        if (elem_idx == n_elem_per_beat - 1) begin
                            // Last element of this beat written: clear beat, decide next action
                            elem_idx     <= 0;
                            beat_valid_r <= 1'b0;
                            axi_busy_r   <= 1'b0;
                            if (rlen_rem > 0) begin
                                // More beats remain in current AXI burst
                                rlen_rem <= rlen_rem - 1;
                                state <= STATE_READ_DATA;
                            end else begin
                                // Current AXI burst finished
                                if (transfer_rem > 0) begin
                                    // Prepare next AXI burst
                                    if (transfer_rem < MAX_TRANSFER) begin
                                        rlen        <= transfer_rem[7:0] - 1;
                                        rlen_rem    <= transfer_rem[7:0] - 1;
                                        transfer_rem <= 0;
                                    end else begin
                                        rlen        <= MAX_TRANSFER[7:0] - 1;
                                        rlen_rem    <= MAX_TRANSFER[7:0] - 1;
                                        transfer_rem <= transfer_rem - MAX_TRANSFER;
                                    end
                                    raddr <= raddr + (MAX_TRANSFER * (`AXI_DATA_WIDTH / 8));
                                    state <= STATE_READ_CMD;
                                end else begin
                                    // All bursts done
                                    state <= STATE_DELAY;
                                end
                            end
                        end
                    end
                    // If gbuf_wr_en is deasserted (backpressure), stay and retry
                end

                STATE_DELAY: begin
                    if (!beat_valid_r) begin
                        done_reg <= 1'b1;
                        state    <= STATE_IDLE;
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
                if (m_axi_arready)
                    state_next = STATE_READ_DATA;
            end
            STATE_READ_DATA: begin
                // handled in sequential block
            end
            STATE_DRAIN_BEAT: begin
                // handled in sequential block
            end
            STATE_DELAY: begin
                if (!beat_valid_r)
                    state_next = STATE_IDLE;
            end
        endcase
    end

    //=========================================================================
    // AXI Read Command
    //=========================================================================
    assign m_axi_arvalid = (state == STATE_READ_CMD);
    assign m_axi_araddr  = raddr;
    assign m_axi_arlen   = rlen;
    // Stall AXI read until current beat is fully drained
    assign m_axi_rready  = (state == STATE_READ_DATA) && !axi_busy_r;

    //=========================================================================
    // GlobalBuffer Write (one 16-bit element per address)
    //=========================================================================
    assign gbuf_wr_en   = beat_valid_r && (state == STATE_DRAIN_BEAT);
    assign gbuf_wr_addr = saddr;
    assign gbuf_wr_data = elem_data;

    assign done = done_reg;

endmodule
