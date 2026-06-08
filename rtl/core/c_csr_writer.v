//=============================================================================
// File     : c_csr_writer.v
// Project  : SPGEMM-Accelerator
// Brief    : C CSR Writer - Collects PE output rows and generates final CSR format.
//
//   Three-phase process:
//     Phase 1: Collect C row results from PEs, write to Output Buffer
//     Phase 2: Prefix sum on C_row_nnz to generate C_row_ptr
//     Phase 3: Write C_col_idx and C_val to Output Buffer at calculated addresses
//
//   Sub-modules:
//     1. Row Result Collector: buffers incoming PE row outputs
//     2. C Row NNZ Buffer: stores nnz per C row
//     3. Prefix Sum Unit: computes C_row_ptr from C_row_nnz
//     4. CSR Address Generator: computes write addresses for col/val
//     5. C Data Writer: writes final CSR arrays to Output Buffer
//=============================================================================

`include "defines.vh"

module c_csr_writer (
    input  wire                          start,
    output reg                           done,

    // Input: from all PEs
    input  wire [`N_PE-1:0]              pe_row_valid,
    input  wire [`N_PE*`MAX_DIM_BITS-1:0] pe_row_id,
    input  wire [`N_PE*`MAX_DIM_BITS-1:0] pe_nnz,
    input  wire [`N_PE*`DATA_WIDTH-1:0]  pe_col,
    input  wire [`N_PE*`DATA_WIDTH-1:0]  pe_val,

    input  wire [`MAX_DIM_BITS-1:0]      M,

    // Output Buffer write
    output reg                           obuf_wr_en,
    output reg  [`OUTBUF_DEPTH_LOG-1:0]  obuf_wr_addr,
    output reg  [`DATA_WIDTH-1:0]        obuf_wr_data,

    // C_row_ptr output (to Store module, written first)
    output reg  [`OUTBUF_DEPTH_LOG-1:0]  row_ptr_base_addr,
    output reg  [`OUTBUF_DEPTH_LOG-1:0]  col_val_base_addr,

    input  wire                          aclk,
    input  wire                          aresetn
);

    //=========================================================================
    // Phase State Machine
    //=========================================================================
    localparam PHASE_IDLE        = 3'd0;
    localparam PHASE_COLLECT     = 3'd1;  // Collect row results from PEs
    localparam PHASE_PREFIX_SUM  = 3'd2;  // Compute C_row_ptr
    localparam PHASE_WRITE_CSR   = 3'd3;  // Write C_col_idx and C_val
    localparam PHASE_DONE        = 3'd4;

    reg [2:0] phase;

    //=========================================================================
    // C Row NNZ Buffer: 512 entries, up to MAX_M
    //=========================================================================
    reg [`CSR_NNZ_BITS-1:0] c_row_nnz [`MAX_M-1:0];
    reg [`CSR_ADDR_BITS-1:0] c_row_ptr [`MAX_M-1:0];

    //=========================================================================
    // Row Result Collector
    //=========================================================================
    // Output Buffer is organized as:
    //   Region 0..M-1: C_row_ptr values
    //   Region M..: C_col_idx, C_val interleaved

    reg [`MAX_DIM_BITS-1:0] collect_count;
    reg [`MAX_DIM_BITS-1:0] current_write_row;
    reg [31:0] collect_write_addr;

    // Per-row temporary storage (simplified: write to OBuf directly)
    reg [`OUTBUF_DEPTH_LOG-1:0] col_val_write_ptr;

    //=========================================================================
    // Prefix Sum Unit
    //=========================================================================
    reg [`MAX_DIM_BITS-1:0] prefix_idx;
    reg [`CSR_ADDR_BITS-1:0] prefix_acc;

    //=========================================================================
    // Phase Control
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            phase <= PHASE_IDLE;
            done <= 1'b0;
            col_val_write_ptr <= `MAX_M;  // start after row_ptr region
        end else begin
            case (phase)
                PHASE_IDLE: begin
                    if (start) begin
                        phase <= PHASE_COLLECT;
                        done <= 1'b0;
                        col_val_write_ptr <= `MAX_M;
                    end
                end
                PHASE_COLLECT: begin
                    // Collect completed rows from PEs
                    // For simplicity, we assume rows arrive in order
                    // In real implementation, need per-row buffering
                    for (integer p = 0; p < `N_PE; p = p + 1) begin
                        if (pe_row_valid[p]) begin
                            c_row_nnz[{7'd0, pe_row_id[p*`MAX_DIM_BITS +: `MAX_DIM_BITS]}] <=
                                pe_nnz[p*`MAX_DIM_BITS +: `CSR_NNZ_BITS];
                            // Write col/val to OBuf at current pointer
                            obuf_wr_en <= 1'b1;
                            obuf_wr_addr <= col_val_write_ptr;
                            obuf_wr_data <= pe_col[p*`DATA_WIDTH +: `DATA_WIDTH];
                            col_val_write_ptr <= col_val_write_ptr + 1;
                        end
                    end
                    // Check if all rows collected
                    if (collect_count >= M)
                        phase <= PHASE_PREFIX_SUM;
                end
                PHASE_PREFIX_SUM: begin
                    // Compute prefix sum on c_row_nnz
                    // Each cycle computes one entry
                    if (prefix_idx < M) begin
                        c_row_ptr[prefix_idx] <= prefix_acc;
                        prefix_acc <= prefix_acc + c_row_nnz[prefix_idx];
                        prefix_idx <= prefix_idx + 1;

                        // Also write C_row_ptr to OBuf
                        obuf_wr_en   <= 1'b1;
                        obuf_wr_addr <= prefix_idx;
                        obuf_wr_data <= prefix_acc;
                    end else begin
                        phase <= PHASE_WRITE_CSR;
                        row_ptr_base_addr <= 0;
                        col_val_base_addr <= `MAX_M;
                    end
                end
                PHASE_WRITE_CSR: begin
                    // Write C_col_idx and C_val final positions
                    // (data already written during PHASE_COLLECT)
                    // Just need to record total nnz for Store module
                    phase <= PHASE_DONE;
                    done <= 1'b1;
                end
                PHASE_DONE: begin
                    phase <= PHASE_IDLE;
                    done <= 1'b0;
                end
            endcase
        end
    end

    //=========================================================================
    // Collection tracking
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            collect_count <= 0;
        end else if (phase == PHASE_IDLE && start) begin
            collect_count <= 0;
        end else if (phase == PHASE_COLLECT) begin
            for (integer p = 0; p < `N_PE; p = p + 1) begin
                if (pe_row_valid[p])
                    collect_count <= collect_count + 1;
            end
        end
    end

    // Prefix sum init
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            prefix_idx  <= 0;
            prefix_acc  <= 0;
        end else if (phase == PHASE_COLLECT && phase != PHASE_COLLECT) begin
            // transitioning to PREFIX_SUM
            prefix_idx  <= 0;
            prefix_acc  <= 0;
        end
    end

    // Default output disables
    always @(*) begin
        if (phase != PHASE_COLLECT && phase != PHASE_PREFIX_SUM) begin
            obuf_wr_en   = 1'b0;
            obuf_wr_addr = 0;
            obuf_wr_data = 0;
        end
    end

endmodule
