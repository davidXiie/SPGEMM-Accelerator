//=============================================================================
// File     : decode.v
// Project  : SPGEMM-Accelerator
// Brief    : Instruction decode for SPGEMM accelerator
//           Reusable from old SPMM accelerator (remapped from Decode.scala)
//=============================================================================

`include "defines.vh"
`include "isa.vh"

//=============================================================================
// FetchDecode: Opcode-level dispatch decode (used by Fetch)
//=============================================================================
module fetch_decode (
    input  wire [`INST_WIDTH-1:0] inst,
    output wire                   is_load,
    output wire                   is_store,
    output wire                   is_spgemm,
    output wire                   is_sched,
    output wire                   is_finish
);

    wire [2:0] opcode = inst[2:0];

    assign is_load   = (opcode == `OP_LOAD);
    assign is_store  = (opcode == `OP_STORE);
    assign is_spgemm = (opcode == `OP_SPGEMM);
    assign is_sched  = (opcode == `OP_SCHED);
    assign is_finish = (opcode == `OP_FINISH);

endmodule


//=============================================================================
// LoadDecode: Decode Load instruction fields
//=============================================================================
module load_decode (
    input  wire [`INST_WIDTH-1:0]  inst,
    output wire [`AXI_ADDR_WIDTH-1:0] dram_offset,
    output wire [31:0]                sram_offset,
    output wire [31:0]                xsize,
    output wire [2:0]                 mem_id
);

    assign dram_offset = {{`AXI_ADDR_WIDTH-58{1'b0}}, inst[`LOAD_DRAM_BASE_HI:`LOAD_DRAM_BASE_LO]};
    assign sram_offset = inst[`LOAD_SRAM_OFFSET_HI:`LOAD_SRAM_OFFSET_LO];
    assign xsize       = inst[`LOAD_XSIZE_HI:`LOAD_XSIZE_LO];
    assign mem_id      = inst[5:3];

endmodule


//=============================================================================
// StoreDecode: Decode Store instruction fields
//=============================================================================
module store_decode (
    input  wire [`INST_WIDTH-1:0]  inst,
    output wire [`AXI_ADDR_WIDTH-1:0] dram_offset,
    output wire [31:0]                sram_offset,
    output wire [31:0]                xsize,
    output wire [2:0]                 mem_id
);

    assign dram_offset = {{`AXI_ADDR_WIDTH-58{1'b0}}, inst[`STORE_DRAM_BASE_HI:`STORE_DRAM_BASE_LO]};
    assign sram_offset = inst[`STORE_SRAM_OFFSET_HI:`STORE_SRAM_OFFSET_LO];
    assign xsize       = inst[`STORE_XSIZE_HI:`STORE_XSIZE_LO];
    assign mem_id      = inst[5:3];

endmodule


//=============================================================================
// SpGEMMDecode: Decode SpGEMM instruction fields
//=============================================================================
module spgemm_decode (
    input  wire [`INST_WIDTH-1:0]  inst,
    output wire [31:0]                a_row_ptr_sram,
    output wire [31:0]                a_col_idx_sram,
    output wire [31:0]                a_val_sram,
    output wire [31:0]                b_row_ptr_sram,
    output wire [31:0]                b_col_idx_sram,
    output wire [31:0]                b_val_sram,
    output wire [`MAX_DIM_BITS-1:0]   M,
    output wire [`MAX_DIM_BITS-1:0]   K,
    output wire [`MAX_DIM_BITS-1:0]   N
);

    assign a_row_ptr_sram = {{`INST_WIDTH-64{1'b0}}, inst[`SPGEMM_A_ROW_SRAM_HI:`SPGEMM_A_ROW_SRAM_LO]};
    assign a_col_idx_sram = inst[`SPGEMM_A_COL_SRAM_HI:`SPGEMM_A_COL_SRAM_LO];
    assign a_val_sram     = inst[`SPGEMM_A_VAL_SRAM_HI:`SPGEMM_A_VAL_SRAM_LO];
    assign b_row_ptr_sram = inst[`SPGEMM_B_ROW_SRAM_HI:`SPGEMM_B_ROW_SRAM_LO];
    assign b_col_idx_sram = inst[`SPGEMM_B_COL_SRAM_HI:`SPGEMM_B_COL_SRAM_LO];
    assign b_val_sram     = inst[`SPGEMM_B_VAL_SRAM_HI:`SPGEMM_B_VAL_SRAM_LO];
    assign M              = inst[`SPGEMM_M_HI:`SPGEMM_M_LO];
    assign K              = inst[`SPGEMM_K_HI:`SPGEMM_K_LO];
    assign N              = inst[`SPGEMM_N_HI:`SPGEMM_N_LO];

endmodule
