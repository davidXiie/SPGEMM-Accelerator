//=============================================================================
// File     : pe_aggregation.v
// Project  : SPGEMM-Accelerator
// Brief    : FP16 Aggregation Unit + ADD Array + Partial Row Buffer (SPA)
//
//   Accepts FP16 partial products from MUL array, aggregates by column index j,
//   and accumulates into the Partial Row Buffer (SPA).
//
//   SPA: acc_valid[512], acc_val[512] (FP16), touched_cols FIFO
//
//   Architecture: Banked Partial Row Buffer + conflict detection
//   - N_MAC banks, bank = j % N_MAC
//   - Each bank processes one read-modify-write per cycle
//   - Conflicts (same bank or same column) handled by stall
//
//   NOTE: FP16 addition (`+`) is modeled as integer add for simulation.
//   For synthesis, replace with FP16 adder IP (e.g., Xilinx Float IP).
//=============================================================================

`include "defines.vh"

module pe_aggregation (
    // Input from MUL array
    input  wire [`N_MAC-1:0]             mul_valid,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] partial_value,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] col_idx,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] row_idx,

    // Control
    input  wire                          row_start,   // new C row start pulse
    input  wire                          row_end,     // current C row done pulse

    // Output: completed C row
    output reg                           out_valid,
    output reg  [`DATA_WIDTH-1:0]        out_col,
    output reg  [`DATA_WIDTH-1:0]        out_val,
    output reg  [`MAX_DIM_BITS-1:0]      out_row_id,
    output reg  [`MAX_DIM_BITS-1:0]      out_nnz,

    input  wire                          aclk,
    input  wire                          aresetn
);

    //=========================================================================
    // Partial Row Buffer (SPA)
    // acc_val[j]: current row partial sum at column j
    // acc_valid[j]: whether column j has been touched in current row
    // touched_cols: FIFO tracking which columns were touched
    //=========================================================================

    // acc_val: 512 deep, DATA_WIDTH wide (one entry per possible column)
    reg [`DATA_WIDTH-1:0] acc_val [0:`MAX_N-1];
    reg [`MAX_N-1:0] acc_valid;

    // touched_cols FIFO: store column indices that have been touched
    reg [`MAX_DIM_BITS-1:0] touched_fifo [0:`MAX_N-1];
    reg [`MAX_DIM_BITS:0] touched_wr_ptr;
    reg [`MAX_DIM_BITS:0] touched_rd_ptr;
    wire touched_empty;
    wire touched_full;

    // Current row tracking
    reg [`MAX_DIM_BITS-1:0] current_row;
    reg [`MAX_DIM_BITS-1:0] current_row_nnz;

    //=========================================================================
    // Banked Access: split by col_idx % N_MAC
    //=========================================================================
    wire [`N_MAC_BITS-1:0] bank_id [`N_MAC-1:0];
    wire [`MAX_DIM_BITS-1:0] col_int [`N_MAC-1:0];

    genvar m;
    generate
        for (m = 0; m < `N_MAC; m = m + 1) begin : gen_bank_extract
            assign col_int[m] = col_idx[m*`DATA_WIDTH +: `MAX_DIM_BITS];
            assign bank_id[m] = col_int[m][`N_MAC_BITS-1:0];
        end
    endgenerate

    //=========================================================================
    // Conflict Detection
    //=========================================================================
    reg stall;

    // Check for bank conflicts: more than one lane targeting the same bank
    // OR two lanes targeting the same column
    always @(*) begin
        stall = 1'b0;
        if (mul_valid != 0) begin
            for (integer i = 0; i < `N_MAC; i = i + 1) begin
                if (mul_valid[i]) begin
                    for (integer j = i + 1; j < `N_MAC; j = j + 1) begin
                        if (mul_valid[j] && (bank_id[i] == bank_id[j] || col_int[i] == col_int[j]))
                            stall = 1'b1;
                    end
                end
            end
        end
    end

    //=========================================================================
    // Read-Modify-Write per lane
    //=========================================================================
    reg [`N_MAC-1:0] acc_rd_data_valid;
    reg [`N_MAC*`DATA_WIDTH-1:0] acc_rd_data;
    reg [`N_MAC*`DATA_WIDTH-1:0] col_latched;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            acc_valid <= 0;
            touched_wr_ptr <= 0;
            touched_rd_ptr <= 0;
            current_row <= 0;
            current_row_nnz <= 0;
        end else begin
            // Clear SPA on new row
            if (row_start) begin
                // Invalidate touched columns
                // For simplicity, clear all touched entries
                while (!touched_empty) begin
                    acc_valid[touched_fifo[touched_rd_ptr]] <= 1'b0;
                    touched_rd_ptr <= touched_rd_ptr + 1'b1;
                end
                touched_wr_ptr <= 0;
            end

            // Aggregation pipeline
            if (mul_valid != 0 && !stall) begin
                for (integer m = 0; m < `N_MAC; m = m + 1) begin
                    if (mul_valid[m]) begin
                        if (!acc_valid[col_int[m]]) begin
                            // First time touching this column
                            acc_val[col_int[m]] <= partial_value[m*`DATA_WIDTH +: `DATA_WIDTH];
                            acc_valid[col_int[m]] <= 1'b1;
                            if (!touched_full) begin
                                touched_fifo[touched_wr_ptr] <= col_int[m];
                                touched_wr_ptr <= touched_wr_ptr + 1'b1;
                            end
                            current_row_nnz <= current_row_nnz + 1;
                        end else begin
                            // Accumulate
                            acc_val[col_int[m]] <= acc_val[col_int[m]] +
                                                   partial_value[m*`DATA_WIDTH +: `DATA_WIDTH];
                        end
                    end
                end
            end

            // Row end: output results sequentially
            if (row_end && !touched_empty) begin
                out_valid <= 1'b1;
                out_col   <= {{`DATA_WIDTH-`MAX_DIM_BITS{1'b0}}, touched_fifo[touched_rd_ptr]};
                out_val   <= acc_val[touched_fifo[touched_rd_ptr]];
                out_row_id <= current_row;
                out_nnz  <= current_row_nnz;
                acc_valid[touched_fifo[touched_rd_ptr]] <= 1'b0;
                touched_rd_ptr <= touched_rd_ptr + 1'b1;
            end else if (row_end && touched_empty) begin
                out_valid <= 1'b0;
                current_row <= current_row + 1;
                current_row_nnz <= 0;
            end else begin
                out_valid <= 1'b0;
            end
        end
    end

    assign touched_empty = (touched_wr_ptr == touched_rd_ptr);
    assign touched_full  = (touched_wr_ptr - touched_rd_ptr >= `MAX_N);

endmodule
