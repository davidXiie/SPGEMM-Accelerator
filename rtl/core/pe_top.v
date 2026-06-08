//=============================================================================
// File     : pe_top.v
// Project  : SPGEMM-Accelerator
// Brief    : PE Top-level - integrates Decompress, MUL Array, and Aggregation.
//
//   Each PE:
//     - Receives task descriptor (row_start, row_end, a_ptr_start, a_ptr_end)
//     - Contains local A Buffer and B Buffer (full B CSR copy)
//     - Decompress: CSR → MAC streams
//     - MUL Array: N_MAC parallel multipliers
//     - Aggregation: column-indexed SPA accumulation
//     - Outputs completed C rows to C CSR Writer
//=============================================================================

`include "defines.vh"

module pe_top #(
    parameter integer PE_ID = 0
) (
    input  wire                      start,
    output wire                      done,

    // Task descriptor
    input  wire [`MAX_DIM_BITS-1:0]  row_start,
    input  wire [`MAX_DIM_BITS-1:0]  row_end,
    input  wire [31:0]               a_ptr_start,
    input  wire [31:0]               a_ptr_end,
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,

    // GlobalBuffer read (for loading B CSR into local buffer)
    output reg                       gbuf_rd_en,
    output reg  [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr,
    input  wire [`BANK_BLOCK_SIZE-1:0] gbuf_rd_data,
    input  wire                      gbuf_rd_valid,

    // Output to CSR Writer
    output wire [`MAX_DIM_BITS-1:0]  out_row_id,
    output wire [`MAX_DIM_BITS-1:0]  out_nnz,
    output wire [`DATA_WIDTH-1:0]    out_col,
    output wire [`DATA_WIDTH-1:0]    out_val,
    output wire                      out_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // A Buffer: stores only this PE's assigned A rows
    //=========================================================================
    std_scratchpad #(
        .DEPTH(`PE_ABUF_DEPTH), .DEPTH_LOG(`PE_ABUF_DEPTH_LOG), .DATA_WIDTH(`DATA_WIDTH)
    ) u_a_buffer (
        .wr_en    (1'b0),  // A Buffer loaded during load phase from GlobalBuffer
        .wr_addr  (0),
        .wr_data  (0),
        .rd_en    (a_buf_rd_en),
        .rd_addr  (a_buf_rd_addr),
        .rd_data  (a_buf_rd_data),
        .rd_valid (a_buf_rd_valid),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );
    wire a_buf_rd_en;
    wire [`PE_ABUF_DEPTH_LOG-1:0] a_buf_rd_addr;
    wire [`DATA_WIDTH-1:0] a_buf_rd_data;
    wire a_buf_rd_valid;

    //=========================================================================
    // B Buffer: full B CSR copy (banked for parallel access)
    // Store: B_row_ptr, B_col_idx, B_val in separate regions
    //=========================================================================
    banked_scratchpad #(
        .N_BANKS(`N_MAC), .DEPTH(`PE_BBUF_DEPTH), .DEPTH_LOG(`PE_BBUF_DEPTH_LOG), .BANK_WIDTH(`DATA_WIDTH)
    ) u_b_buffer (
        .wr_en    (1'b0),  // B Buffer loaded during load phase
        .wr_addr  (0),
        .wr_data  (0),
        .rd_en    (b_buf_rd_en),
        .rd_addr  (b_buf_rd_addr),
        .rd_data  (b_buf_rd_data),
        .rd_valid (b_buf_rd_valid),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );
    wire [`N_MAC-1:0]            b_buf_rd_en;
    wire [`N_MAC*`PE_BBUF_DEPTH_LOG-1:0] b_buf_rd_addr;
    wire [`N_MAC*`DATA_WIDTH-1:0] b_buf_rd_data;
    wire [`N_MAC-1:0]            b_buf_rd_valid;

    //=========================================================================
    // Interconnect wires
    //=========================================================================
    wire decomp_done;
    wire [`N_MAC-1:0]                  lane_valid;
    wire [`N_MAC*`DATA_WIDTH-1:0]      lane_a_val;
    wire [`N_MAC*`DATA_WIDTH-1:0]      lane_b_val;
    wire [`N_MAC*`DATA_WIDTH-1:0]      lane_col_idx;
    wire [`N_MAC*`DATA_WIDTH-1:0]      lane_row_idx;

    wire [`N_MAC-1:0]                  mul_valid;
    wire [`N_MAC*`DATA_WIDTH-1:0]      partial_value;
    wire [`N_MAC*`DATA_WIDTH-1:0]      mul_col_idx;
    wire [`N_MAC*`DATA_WIDTH-1:0]      mul_row_idx;

    // Aggregation control
    wire agg_row_start;
    wire agg_row_end;
    reg  [`MAX_DIM_BITS-1:0] prev_row;

    // Row start/end detection
    assign agg_row_start = (lane_row_idx[0 +: `MAX_DIM_BITS] != prev_row);
    assign agg_row_end   = decomp_done;

    always @(posedge aclk) begin
        if (lane_valid[0])
            prev_row <= lane_row_idx[0 +: `MAX_DIM_BITS];
    end

    //=========================================================================
    // Sub-module Instantiations
    //=========================================================================

    // Decompress Unit
    pe_decompress u_decompress (
        .start         (start),
        .done          (decomp_done),
        .row_start     (row_start),
        .row_end       (row_end),
        .K             (K),
        .N             (N),
        .a_buf_rd_en   (a_buf_rd_en),
        .a_buf_rd_addr (a_buf_rd_addr),
        .a_buf_rd_data (a_buf_rd_data),
        .a_buf_rd_valid(a_buf_rd_valid),
        .b_buf_rd_en   (b_buf_rd_en),
        .b_buf_rd_addr (b_buf_rd_addr),
        .b_buf_rd_data (b_buf_rd_data),
        .b_buf_rd_valid(b_buf_rd_valid),
        .lane_valid    (lane_valid),
        .lane_a_val    (lane_a_val),
        .lane_b_val    (lane_b_val),
        .lane_col_idx  (lane_col_idx),
        .lane_row_idx  (lane_row_idx),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // MUL Array
    pe_mul_array u_mul_array (
        .lane_valid    (lane_valid),
        .lane_a_val    (lane_a_val),
        .lane_b_val    (lane_b_val),
        .lane_col_idx  (lane_col_idx),
        .lane_row_idx  (lane_row_idx),
        .mul_valid     (mul_valid),
        .partial_value (partial_value),
        .col_idx       (mul_col_idx),
        .row_idx       (mul_row_idx),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // Aggregation Unit + SPA
    pe_aggregation u_aggregation (
        .mul_valid     (mul_valid),
        .partial_value (partial_value),
        .col_idx       (mul_col_idx),
        .row_idx       (mul_row_idx),
        .row_start     (agg_row_start),
        .row_end       (agg_row_end),
        .out_valid     (out_valid),
        .out_col       (out_col),
        .out_val       (out_val),
        .out_row_id    (out_row_id),
        .out_nnz       (out_nnz),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // Done
    assign done = decomp_done;

endmodule
