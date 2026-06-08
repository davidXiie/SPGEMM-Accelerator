//=============================================================================
// File     : tb_core_top.v
// Project  : SPGEMM-Accelerator
// Brief    : Testbench for SPGEMM Core Top
//=============================================================================

`include "defines.vh"
`include "isa.vh"

module tb_core_top;

    reg aclk;
    reg aresetn;

    // CR interface
    reg  cr_launch;
    reg  [`AXI_ADDR_WIDTH-1:0] ins_baddr;
    reg  [31:0] ins_count;
    wire cr_finish;

    // AXI Read
    wire m_axi_arvalid;
    reg  m_axi_arready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0]  m_axi_arlen;

    reg  m_axi_rvalid;
    wire m_axi_rready;
    reg  [`AXI_DATA_WIDTH-1:0] m_axi_rdata;
    reg  m_axi_rlast;

    // AXI Write
    wire m_axi_awvalid;
    reg  m_axi_awready;
    wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0]  m_axi_awlen;

    wire m_axi_wvalid;
    reg  m_axi_wready;
    wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata;
    wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb;
    wire m_axi_wlast;

    reg  m_axi_bvalid;
    wire m_axi_bready;
    reg  [1:0] m_axi_bresp;

    wire [31:0] cycle_counter;

    //=========================================================================
    // Clock and Reset
    //=========================================================================
    always #5 aclk = ~aclk;  // 100MHz clock

    initial begin
        aclk   = 1'b0;
        aresetn = 1'b0;
        cr_launch  = 1'b0;
        ins_baddr  = 64'h0;
        ins_count  = 32'd0;

        m_axi_arready = 1'b0;
        m_axi_rvalid  = 1'b0;
        m_axi_rdata   = 512'h0;
        m_axi_rlast   = 1'b0;
        m_axi_awready = 1'b0;
        m_axi_wready  = 1'b0;
        m_axi_bvalid  = 1'b0;
        m_axi_bresp   = 2'b00;

        // Reset
        #100;
        aresetn = 1'b1;
        #20;

        // --- Test 1: Simple Launch ---
        $display("[TB] Test 1: Launch accelerator");
        ins_baddr = 64'h1000;
        ins_count = 32'd6;  // 6 instructions total
        cr_launch = 1'b1;
        #10;
        cr_launch = 1'b0;

        // Wait for fetch AXI reads
        // Simulate AXI read responses for instructions
        // (In real simulation, a memory model would be used)

        #100;
        $display("[TB] Cycle: %d, State active", $time);

        // Let the simulation run
        #1000;
        $display("[TB] Test completed at cycle %d", $time);
        $finish;
    end

    //=========================================================================
    // DUT Instantiation
    //=========================================================================
    core_top u_dut (
        .cr_launch     (cr_launch),
        .ins_baddr     (ins_baddr),
        .ins_count     (ins_count),
        .cr_finish     (cr_finish),
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_awvalid (m_axi_awvalid),
        .m_axi_awready (m_axi_awready),
        .m_axi_awaddr  (m_axi_awaddr),
        .m_axi_awlen   (m_axi_awlen),
        .m_axi_wvalid  (m_axi_wvalid),
        .m_axi_wready  (m_axi_wready),
        .m_axi_wdata   (m_axi_wdata),
        .m_axi_wstrb   (m_axi_wstrb),
        .m_axi_wlast   (m_axi_wlast),
        .m_axi_bvalid  (m_axi_bvalid),
        .m_axi_bready  (m_axi_bready),
        .m_axi_bresp   (m_axi_bresp),
        .cycle_counter (cycle_counter),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    //=========================================================================
    // Simple AXI memory model for instruction fetch
    //=========================================================================
    reg [511:0] mem [0:1023];  // small memory model

    initial begin
        // Load some test instructions into memory
        // SpGEMM instruction (opcode = 3'b011)
        // Fields: A/B SRAM bases, dimensions M=4, K=4, N=4
        mem[0] = {224'h0, 4'd4, 4'd4, 4'd4,    // M,K,N (last 12 bits of dims)
                  32'h100, 32'h080, 32'h060,     // B_val, B_col, B_row SRAM bases
                  32'h040, 32'h020, 32'h000};    // A_val, A_col, A_row SRAM bases
        // mem[0][2:0] = 3'b011 (SPGEMM op)

        // Store instruction
        mem[1] = 256'h0;

        $display("[TB] Memory model initialized");
    end

    // AXI read responder
    reg [7:0] rd_cnt;
    reg [`AXI_ADDR_WIDTH-1:0] rd_addr;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axi_arready <= 1'b1;
            m_axi_rvalid  <= 1'b0;
            rd_cnt <= 0;
            rd_addr <= 0;
        end else begin
            if (m_axi_arvalid && m_axi_arready) begin
                rd_addr <= m_axi_araddr;
                rd_cnt  <= 0;
                m_axi_arready <= 1'b0;
            end else if (rd_cnt <= m_axi_arlen && !m_axi_arvalid) begin
                m_axi_rvalid <= 1'b1;
                m_axi_rdata  <= mem[rd_addr[9:0] + rd_cnt];
                m_axi_rlast  <= (rd_cnt == m_axi_arlen);
                rd_cnt <= rd_cnt + 1;
                if (rd_cnt == m_axi_arlen)
                    m_axi_arready <= 1'b1;
            end else if (m_axi_rvalid && m_axi_rready) begin
                m_axi_rvalid <= 1'b0;
            end
        end
    end

    // AXI write responder
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            m_axi_awready <= 1'b1;
            m_axi_wready  <= 1'b1;
            m_axi_bvalid  <= 1'b0;
        end else begin
            if (m_axi_awvalid && m_axi_awready) begin
                m_axi_awready <= 1'b0;
            end
            if (m_axi_wvalid && m_axi_wready && m_axi_wlast) begin
                m_axi_bvalid  <= 1'b1;
                m_axi_wready  <= 1'b0;
            end
            if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid  <= 1'b0;
                m_axi_awready <= 1'b1;
                m_axi_wready  <= 1'b1;
            end
        end
    end

    //=========================================================================
    // Monitoring
    //=========================================================================
    always @(posedge aclk) begin
        if (cr_finish)
            $display("[TB] *** Accelerator FINISHED at cycle %d ***", cycle_counter);
    end

endmodule
