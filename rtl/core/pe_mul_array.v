//=============================================================================
// File     : pe_mul_array.v
// Project  : SPGEMM-Accelerator
// Brief    : FP16 MUL Array - N_MAC parallel FP16 multipliers with 3-stage pipeline.
//
//   IEEE 754 FP16 format: sign[15] | exp[14:10] | mantissa[9:0]
//   Each multiplier produces: partial = a_val * b_val (FP16 multiply)
//
//   NOTE: The `*` operator simulates as integer multiply. For real FP16 behavior,
//   replace with an FP16 multiplier IP (e.g., Xilinx Floating-point Operator,
//   custom FP16 multiply module). The 3-stage pipeline structure stays the same.
//
//   Input:  lane_valid[m], lane_a_val[m], lane_b_val[m], lane_col_idx[m], lane_row_idx[m]
//   Output: mul_valid[m], partial_value[m], col_idx[m], row_idx[m]
//=============================================================================

`include "defines.vh"

module pe_mul_array (
    input  wire [`N_MAC-1:0]                  lane_valid,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_a_val,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_b_val,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_col_idx,
    input  wire [`N_MAC*`DATA_WIDTH-1:0]      lane_row_idx,

    // Pipelined output (3 stages: reg → mul → reg)
    output reg  [`N_MAC-1:0]                  mul_valid,
    output reg  [`N_MAC*`DATA_WIDTH-1:0]      partial_value,  // FP16 result
    output reg  [`N_MAC*`DATA_WIDTH-1:0]      col_idx,
    output reg  [`N_MAC*`DATA_WIDTH-1:0]      row_idx,

    input  wire                               aclk,
    input  wire                               aresetn
);

    // Stage 0: input registers (pipeline for timing)
    reg [`N_MAC-1:0]                  valid_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      a_val_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      b_val_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      col_s0;
    reg [`N_MAC*`DATA_WIDTH-1:0]      row_s0;

    // Stage 1: FP16 multiply (replace `*` with FP16 multiplier IP in synthesis)
    reg [`N_MAC-1:0]                  valid_s1;
    reg [`N_MAC*`DATA_WIDTH-1:0]      partial_s1;  // FP16
    reg [`N_MAC*`DATA_WIDTH-1:0]      col_s1;
    reg [`N_MAC*`DATA_WIDTH-1:0]      row_s1;

    // Stage 0: register inputs
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            valid_s0 <= 0;
            a_val_s0 <= 0;
            b_val_s0 <= 0;
            col_s0   <= 0;
            row_s0   <= 0;
        end else begin
            valid_s0 <= lane_valid;
            a_val_s0 <= lane_a_val;
            b_val_s0 <= lane_b_val;
            col_s0   <= lane_col_idx;
            row_s0   <= lane_row_idx;
        end
    end

    // Stage 1: FP16 multiply
    // TODO: Replace `*` with synthesizable FP16 multiplier (e.g. instantiate
    //       fp16_mul sub-module that implements IEEE 754 half-precision multiply)
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            valid_s1   <= 0;
            partial_s1 <= 0;
            col_s1     <= 0;
            row_s1     <= 0;
        end else begin
            valid_s1 <= valid_s0;
            col_s1   <= col_s0;
            row_s1   <= row_s0;
            for (integer m = 0; m < `N_MAC; m = m + 1) begin
                if (valid_s0[m]) begin
                    partial_s1[m*`DATA_WIDTH +: `DATA_WIDTH] <=
                        a_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH] *
                        b_val_s0[m*`DATA_WIDTH +: `DATA_WIDTH];
                end
            end
        end
    end

    // Stage 2: output registers
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            mul_valid     <= 0;
            partial_value <= 0;
            col_idx       <= 0;
            row_idx       <= 0;
        end else begin
            mul_valid     <= valid_s1;
            partial_value <= partial_s1;
            col_idx       <= col_s1;
            row_idx       <= row_s1;
        end
    end

endmodule
