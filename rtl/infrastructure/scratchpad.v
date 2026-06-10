//=============================================================================
// File     : scratchpad.v
// Project  : SPGEMM-Accelerator
// Brief    : Scratchpad / SRAM modules: GlobalBuffer, BankedScratchpad,
//           StandardScratchpad, OutputScratchpad.
//           Reusable from old SPMM accelerator (remapped from ScratchPad.scala)
//=============================================================================

`include "defines.vh"

//=============================================================================
// GlobalBuffer: 16-bit wide single-entry SRAM
// Caches CSR data from DRAM (one FP16 element per address) before
// distribution to scheduler/PEs.
//=============================================================================
module global_buffer #(
    parameter integer DEPTH     = `GBUF_DEPTH,
    parameter integer DEPTH_LOG = `GBUF_DEPTH_LOG
) (
    // Write port (from Load module, 16-bit per element)
    input  wire                      wr_en,
    input  wire [DEPTH_LOG-1:0]      wr_addr,
    input  wire [`DATA_WIDTH-1:0]    wr_data,

    // Read port (to scheduler, 16-bit per element)
    input  wire                      rd_en,
    input  wire [DEPTH_LOG-1:0]      rd_addr,
    output wire [`DATA_WIDTH-1:0]    rd_data,
    output wire                      rd_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    // SRAM storage: one 16-bit FP16 element per address
    reg [`DATA_WIDTH-1:0] ram [0:DEPTH-1];
    integer i;

    always @(posedge aclk) begin
        if (wr_en) begin
            ram[wr_addr] <= wr_data;
        end
    end

    // Read: registered output
    reg [`DATA_WIDTH-1:0] rd_data_reg;
    reg rd_valid_reg;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rd_data_reg  <= 0;
            rd_valid_reg <= 1'b0;
        end else begin
            rd_valid_reg <= rd_en;
            if (rd_en) begin
                rd_data_reg <= ram[rd_addr];
            end
        end
    end

    assign rd_data  = rd_data_reg;
    assign rd_valid = rd_valid_reg;

endmodule


//=============================================================================
// BankedScratchpad: Multi-bank parallel read SRAM
// Used for PE B Buffer: each bank reads a different column data simultaneously
//=============================================================================
module banked_scratchpad #(
    parameter integer N_BANKS    = `N_MAC,
    parameter integer DEPTH      = `PE_BBUF_DEPTH,
    parameter integer DEPTH_LOG  = `PE_BBUF_DEPTH_LOG,
    parameter integer BANK_WIDTH = `DATA_WIDTH
) (
    // Write port
    input  wire                          wr_en,
    input  wire [DEPTH_LOG-1:0]          wr_addr,
    input  wire [N_BANKS*BANK_WIDTH-1:0] wr_data,

    // Read ports (N_BANKS parallel)
    input  wire [N_BANKS-1:0]            rd_en,
    input  wire [N_BANKS*DEPTH_LOG-1:0]  rd_addr,
    output wire [N_BANKS*BANK_WIDTH-1:0] rd_data,
    output wire [N_BANKS-1:0]            rd_valid,

    input  wire                          aclk,
    input  wire                          aresetn
);

    genvar b;
    generate
        for (b = 0; b < N_BANKS; b = b + 1) begin : gen_bank
            reg [BANK_WIDTH-1:0] ram [0:DEPTH-1];
            reg [BANK_WIDTH-1:0] rd_reg;
            reg rd_valid_reg;
            integer i;

            wire [DEPTH_LOG-1:0] rd_addr_b = rd_addr[b*DEPTH_LOG +: DEPTH_LOG];

            always @(posedge aclk) begin
                if (wr_en) begin
                    ram[wr_addr] <= wr_data[(b+1)*BANK_WIDTH-1 -: BANK_WIDTH];
                end
            end

            always @(posedge aclk or negedge aresetn) begin
                if (!aresetn) begin
                    rd_reg       <= 0;
                    rd_valid_reg <= 1'b0;
                end else begin
                    rd_valid_reg <= rd_en[b];
                    if (rd_en[b]) begin
                        rd_reg <= ram[rd_addr_b];
                    end
                end
            end

            assign rd_data[(b+1)*BANK_WIDTH-1 -: BANK_WIDTH] = rd_reg;
            assign rd_valid[b] = rd_valid_reg;
        end
    endgenerate

endmodule


//=============================================================================
// StandardScratchpad: Single-port or simple dual-port SRAM
// Used for A Buffer, partial row buffer, etc.
//=============================================================================
module std_scratchpad #(
    parameter integer DEPTH      = 1024,
    parameter integer DEPTH_LOG  = 10,
    parameter integer DATA_WIDTH = `DATA_WIDTH
) (
    // Write port
    input  wire                  wr_en,
    input  wire [DEPTH_LOG-1:0]  wr_addr,
    input  wire [DATA_WIDTH-1:0] wr_data,

    // Read port
    input  wire                  rd_en,
    input  wire [DEPTH_LOG-1:0]  rd_addr,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  rd_valid,

    input  wire                  aclk,
    input  wire                  aresetn
);

    reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];
    reg [DATA_WIDTH-1:0] rd_data_reg;
    reg rd_valid_reg;
    integer i;

    always @(posedge aclk) begin
        if (wr_en) begin
            ram[wr_addr] <= wr_data;
        end
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rd_data_reg  <= 0;
            rd_valid_reg <= 1'b0;
        end else begin
            rd_valid_reg <= rd_en;
            if (rd_en) begin
                rd_data_reg <= ram[rd_addr];
            end
        end
    end

    assign rd_data  = rd_data_reg;
    assign rd_valid = rd_valid_reg;

endmodule


//=============================================================================
// OutputScratchpad: Wide-write, wide-read buffer for output results
//=============================================================================
module output_scratchpad #(
    parameter integer DEPTH      = `OUTBUF_DEPTH,
    parameter integer DEPTH_LOG  = `OUTBUF_DEPTH_LOG,
    parameter integer DATA_WIDTH = `DATA_WIDTH
) (
    // Write port (from C CSR Writer, data_width)
    input  wire                      wr_en,
    input  wire [DEPTH_LOG-1:0]      wr_addr,
    input  wire [DATA_WIDTH-1:0]     wr_data,

    // Read port (to Store module, AXI_DATA_WIDTH)
    input  wire                      rd_en,
    input  wire [DEPTH_LOG-1:0]      rd_addr,
    output wire [`AXI_DATA_WIDTH-1:0] rd_data,
    output wire                      rd_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam integer N_PER_BEAT = `AXI_DATA_WIDTH / DATA_WIDTH;  // 512/16 = 32 FP16 elements per AXI beat
    localparam integer N_PER_BEAT_LOG = 5;

    reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];
    reg [`AXI_DATA_WIDTH-1:0] rd_data_reg;
    reg rd_valid_reg;
    integer i;

    always @(posedge aclk) begin
        if (wr_en) begin
            ram[wr_addr] <= wr_data;
        end
    end

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rd_data_reg  <= 0;
            rd_valid_reg <= 1'b0;
        end else begin
            rd_valid_reg <= rd_en;
            if (rd_en) begin
                // Read N_PER_BEAT consecutive entries
                for (i = 0; i < N_PER_BEAT; i = i + 1) begin
                    rd_data_reg[i*DATA_WIDTH +: DATA_WIDTH] <=
                        ram[{rd_addr[DEPTH_LOG-1:N_PER_BEAT_LOG], i[N_PER_BEAT_LOG-1:0]}];
                end
            end
        end
    end

    assign rd_data  = rd_data_reg;
    assign rd_valid = rd_valid_reg;

endmodule


//=============================================================================
// SimpleFIFO: Synchronous FIFO for buffering
//=============================================================================
module sync_fifo #(
    parameter integer WIDTH      = 32,
    parameter integer DEPTH      = 16,
    parameter integer DEPTH_LOG  = 4
) (
    input  wire                  wr_en,
    input  wire [WIDTH-1:0]      wr_data,
    output wire                  wr_full,

    input  wire                  rd_en,
    output wire [WIDTH-1:0]      rd_data,
    output wire                  rd_empty,

    output wire [DEPTH_LOG:0]    count,

    input  wire                  aclk,
    input  wire                  aresetn
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [DEPTH_LOG:0] wr_ptr, rd_ptr;
    integer i;

    assign count     = (wr_ptr >= rd_ptr) ? (wr_ptr - rd_ptr) : (DEPTH + wr_ptr - rd_ptr);
    assign wr_full   = (count >= DEPTH);
    assign rd_empty  = (count == 0);
    assign rd_data   = mem[rd_ptr[DEPTH_LOG-1:0]];

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (wr_en && !wr_full)
                wr_ptr <= wr_ptr + 1'b1;
            if (rd_en && !rd_empty)
                rd_ptr <= rd_ptr + 1'b1;
        end
    end

    always @(posedge aclk) begin
        if (wr_en && !wr_full)
            mem[wr_ptr[DEPTH_LOG-1:0]] <= wr_data;
    end

endmodule
