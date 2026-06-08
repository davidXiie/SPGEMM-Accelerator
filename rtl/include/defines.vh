//=============================================================================
// File     : defines.vh
// Project  : SPGEMM-Accelerator
// Brief    : Global parameter definitions for SPGEMM accelerator
//=============================================================================

`ifndef DEFINES_VH
`define DEFINES_VH

//=============================================================================
// Matrix Dimensions
//=============================================================================
`define MAX_M         512     // A rows max
`define MAX_K         512     // A cols = B rows max
`define MAX_N         512     // B cols max
`define MAX_DIM_BITS  10      // log2(512) = 10

//=============================================================================
// PE & MAC Configuration
//=============================================================================
`define N_PE          8       // Number of PEs
`define N_MAC         4       // MAC units per PE
`define N_MAC_BITS    3       // log2(N_MAC)
`define N_PE_BITS     3       // log2(N_PE)

//=============================================================================
// Data Width (FP16: half-precision floating point)
//=============================================================================
`define DATA_WIDTH    16      // FP16 (IEEE 754 half precision: 1 sign, 5 exp, 10 mantissa)
`define DATA_BYTES    2       // DATA_WIDTH / 8
`define DATA_BITS_LOG2 4      // log2(16)

//=============================================================================
// AXI Bus Parameters 
//=============================================================================
`define AXI_DATA_WIDTH 512    // AXI data width                 AXI协议数据宽度
`define AXI_ADDR_WIDTH 64     // AXI address width              AXI协议地址宽度
`define AXI_LEN_WIDTH  8      // AXI burst length width         AXI协议burst长度宽度
`define AXI_STRB_WIDTH 64     // AXI strobe width (= AXI_DATA_WIDTH/8, byte-level)          AXI协议strobe宽度
`define AXI_ID_WIDTH   4      // AXI transaction ID width       AXI协议ID宽度

//=============================================================================
// Instruction Format (256-bit)
//=============================================================================
`define INST_WIDTH    256
`define OPCODE_WIDTH  3       // opcode [2:0]
`define MEMID_WIDTH   3       // memory type ID [5:3]

// Opcode values
`define OP_LOAD       3'b001
`define OP_STORE      3'b010
`define OP_SPGEMM     3'b011
`define OP_SCHED      3'b100
`define OP_FINISH     3'b111

// Memory type IDs
`define MEM_ROW_PTR   3'b000
`define MEM_COL_IDX   3'b001
`define MEM_VAL       3'b010
`define MEM_OUTPUT    3'b011
`define MEM_PSUM      3'b100

//=============================================================================
// Scratchpad / Buffer Sizes
//   Sizing rationale for 512×512 FP16 matrices:
//   - PE B Buffer: full B CSR, worst-case dense 512×512 nnz = 262144 entries
//     plus B_row_ptr (513 entries). In practice, sparsity >> 0%, so allocate 131K.
//   - PE A Buffer: holds this PE's assigned A rows (worst ~256 rows × dense ~512 nnz/row
//     = 131K elements). 32K is a practical buffer with spill handling assumed.
//   - Global Buffer: caches both A CSR + B CSR from DRAM.
//   - SPA depth = MAX_N = 512 (one accumulator per possible C column).
//=============================================================================
`define GBUF_DEPTH      65536   // Global Buffer depth (entries)
`define GBUF_DEPTH_LOG  16      // log2(65536)

`define PE_ABUF_DEPTH   32768   // PE A Buffer depth
`define PE_ABUF_DEPTH_LOG 15    // log2(32768)

`define PE_BBUF_DEPTH   131072  // PE B Buffer depth (full B CSR)
`define PE_BBUF_DEPTH_LOG 17    // log2(131072)

`define PE_SPA_DEPTH    512     // PE Partial Row Buffer / SPA depth (= MAX_N)
`define PE_SPA_DEPTH_LOG 9      // log2(512)

`define OUTBUF_DEPTH    65536   // Output Buffer depth (streaming to Store)
`define OUTBUF_DEPTH_LOG 16     // log2(65536)

//=============================================================================
// Scheduler Parameters
//=============================================================================
`define WORKLOAD_BITS 20      // Bit width for workload counters
`define SCHED_BUF_DEPTH 512   // b_row_nnz / row_cyc buffer (= MAX_K / MAX_M)

//=============================================================================
// CSR Writer Parameters
//=============================================================================
`define CSR_ADDR_BITS 20      // CSR output address width
`define CSR_NNZ_BITS  10      // Per-row nnz counter bits

//=============================================================================
// Global Constants
//=============================================================================
`define AXI_BURST_MAX 256     // Max AXI burst length

//=============================================================================
// Derived Constants
//=============================================================================
`define N_MAC_PER_PE  `N_MAC
`define TOTAL_MAC     (`N_PE * `N_MAC)
`define PE_ID_BITS    `N_PE_BITS

// Bank block size for GlobalBuffer: DATA_WIDTH * N_MAC (16*4=64 bits per bank block)
// One bank block = one cycle of N_MAC parallel FP16 reads
`define BANK_BLOCK_SIZE (`DATA_WIDTH * `N_MAC)      // 64 bits
`define BANK_BLOCK_BYTES (`BANK_BLOCK_SIZE / 8)      // 8 bytes

// AXI beat carries N_ELEM_PER_AXI_BEAT elements (512/16=32 FP16 elements per beat)
`define N_ELEM_PER_AXI_BEAT (`AXI_DATA_WIDTH / `DATA_WIDTH)

`endif // DEFINES_VH
