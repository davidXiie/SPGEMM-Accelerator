//=============================================================================
// File     : core_top.v
// Project  : SPGEMM-Accelerator
// Brief    : Top-level core module - connects all sub-modules:
//           Fetch, Load, Store, GlobalBuffer, Scheduler, PE Array, C CSR Writer.
//           Main state machine: Idle → Load_A → Load_B → Schedule → Compute → WriteCSR → Store → Finish
//=============================================================================

`include "defines.vh"
`include "isa.vh"

module core_top (
    // Control Register interface (AXI-Lite slave signals via Wrapper)
    input  wire                      cr_launch,     //启动信号
    input  wire [`AXI_ADDR_WIDTH-1:0] ins_baddr,     //指令起始地址
    input  wire [15:0]               ins_count,        ///指令数量
    output wire                      cr_finish,         //完成信号

    // AXI Read Master (for Fetch + Load)
    output wire                      m_axi_arvalid,     
    input  wire                      m_axi_arready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,

    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,
    input  wire [`AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire                      m_axi_rlast,

    // AXI Write Master (for Store)
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,
    output wire [`AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,

    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,
    output wire [`AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [`AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,

    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    input  wire [1:0]                m_axi_bresp,

    // Debug / Performance counters
    output wire [15:0]               cycle_counter,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // State Machine
    //=========================================================================
    localparam STATE_IDLE      = 4'd0;
    localparam STATE_LOAD_A    = 4'd1;
    localparam STATE_LOAD_B    = 4'd2;
    localparam STATE_SCHEDULE  = 4'd3;
    localparam STATE_COMPUTE   = 4'd4;
    localparam STATE_WRITE_CSR = 4'd5;
    localparam STATE_STORE     = 4'd6;
    localparam STATE_FINISH    = 4'd7;

    reg [3:0] state, state_next;

    // Instruction counter
    reg [15:0] ins_count_curr;
    reg [15:0] ins_count_total;

    // Load sub-states (0: A_row_ptr, 1: A_col_idx, 2: A_val, 3: B_row_ptr, 4: B_col_idx, 5: B_val)
    reg [2:0] load_sub_state;
    localparam LOAD_A_ROW   = 3'd0;
    localparam LOAD_A_COL   = 3'd1;
    localparam LOAD_A_VAL   = 3'd2;
    localparam LOAD_B_ROW   = 3'd3;
    localparam LOAD_B_COL   = 3'd4;
    localparam LOAD_B_VAL   = 3'd5;
    localparam LOAD_DONE    = 3'd6;

    //=========================================================================
    // Instruction Decode registers (from SpGEMM instruction)
    //=========================================================================
    reg [15:0] a_row_ptr_sram;
    reg [15:0] a_col_idx_sram;
    reg [15:0] a_val_sram;
    reg [15:0] b_row_ptr_sram;
    reg [15:0] b_col_idx_sram;
    reg [15:0] b_val_sram;
    reg [`MAX_DIM_BITS-1:0] M, K, N;

    //=========================================================================
    // Sub-module signals
    //=========================================================================

    // Fetch → Decode
    wire [`INST_WIDTH-1:0] fetch_ld_inst, fetch_sp_inst, fetch_st_inst;
    wire fetch_ld_valid, fetch_ld_ready;
    wire fetch_sp_valid, fetch_sp_ready;
    wire fetch_st_valid, fetch_st_ready;
    wire fetch_sch_valid, fetch_sch_ready;

    // Load
    wire load_done;
    wire load_gbuf_wr_en;
    wire [`GBUF_DEPTH_LOG-1:0] load_gbuf_wr_addr;
    wire [`DATA_WIDTH-1:0]   load_gbuf_wr_data;

    // GlobalBuffer
    wire gbuf_rd_en;
    wire [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr;
    wire [`DATA_WIDTH-1:0]     gbuf_rd_data;
    wire gbuf_rd_valid;

    // Scheduler
    wire sched_start;
    wire sched_done;
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_row_start, pe_row_end;
    wire [`N_PE-1:0][15:0]              pe_a_ptr_start, pe_a_ptr_end;
    wire [`N_PE-1:0]                    pe_task_valid;

    // PE Array
    wire [`N_PE-1:0] pe_done;
    wire pe_all_done;
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_out_row_id;
    wire [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_out_nnz;
    wire [`N_PE-1:0][`DATA_WIDTH-1:0]   pe_out_col;     // single element stream
    wire [`N_PE-1:0][`DATA_WIDTH-1:0]   pe_out_val;
    wire [`N_PE-1:0]                    pe_out_valid;

    // C CSR Writer
    wire csr_done;

    // Store
    wire store_done;

    // AXI read mux
    wire fetch_arvalid, load_arvalid;
    wire fetch_arready, load_arready;
    wire [`AXI_ADDR_WIDTH-1:0] fetch_araddr, load_araddr;
    wire [7:0] fetch_arlen, load_arlen;
    wire fetch_rvalid, load_rvalid;
    wire fetch_rready, load_rready;
    wire [`AXI_DATA_WIDTH-1:0] fetch_rdata, load_rdata;
    wire fetch_rlast, load_rlast;

    //=========================================================================
    // Cycle counter
    //=========================================================================
    reg [15:0] cycle_cnt;
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) cycle_cnt <= 0;
        else          cycle_cnt <= cycle_cnt + 1;
    end
    assign cycle_counter = cycle_cnt;

    //=========================================================================
    // State Machine
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state <= STATE_IDLE;
            load_sub_state <= LOAD_A_ROW;
            ins_count_curr <= 0;
        end else begin
            state <= state_next;

            case (state)
                STATE_IDLE: begin
                    load_sub_state <= LOAD_A_ROW;       //开始的话要从A_row开始
                    if (cr_launch)
                        ins_count_total <= ins_count;       //存储指令数
                end
                STATE_LOAD_A: begin
                    if (load_done) begin
                        if (load_sub_state == LOAD_DONE)        //一个子状态结束就切换到下一个子状态
                            load_sub_state <= LOAD_B_ROW;
                        else
                            load_sub_state <= load_sub_state + 1;       //继续下一个子状态
                    end
                end
                STATE_LOAD_B: begin
                    if (load_done) begin
                        if (load_sub_state == LOAD_DONE)        //一个子状态结束就切换到下一个子状态
                            load_sub_state <= LOAD_DONE;
                        else
                            load_sub_state <= load_sub_state + 1;
                    end
                end
                STATE_SCHEDULE: begin       //运行调度
                    // wait for scheduler done
                end
                STATE_COMPUTE: begin
                    // wait for all PEs done
                end
                STATE_WRITE_CSR: begin
                    // wait for CSR writer done
                end
                STATE_STORE: begin
                    if (store_done) begin                       //指令寄存器加一
                        ins_count_curr <= ins_count_curr + 1;
                    end
                end
                STATE_FINISH: begin
                    // done, wait for idle
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        state_next = state;
        case (state)
            STATE_IDLE: begin
                if (cr_launch)
                    state_next = STATE_LOAD_A;
            end
            STATE_LOAD_A: begin
                if (load_done) begin
                    if (load_sub_state == LOAD_DONE)
                        state_next = STATE_LOAD_B;
                end
            end
            STATE_LOAD_B: begin
                if (load_done) begin
                    if (load_sub_state == LOAD_DONE)
                        state_next = STATE_SCHEDULE;
                end
            end
            STATE_SCHEDULE: begin
                if (sched_done)
                    state_next = STATE_COMPUTE;
            end
            STATE_COMPUTE: begin
                if (pe_all_done)
                    state_next = STATE_WRITE_CSR;
            end
            STATE_WRITE_CSR: begin
                if (csr_done)
                    state_next = STATE_STORE;
            end
            STATE_STORE: begin
                if (store_done) begin
                    if (ins_count_curr + 1 >= ins_count_total)
                        state_next = STATE_FINISH;
                    else
                        state_next = STATE_LOAD_A;
                end
            end
            STATE_FINISH: begin
                // stay until re-launched
            end
            default: state_next = STATE_IDLE;
        endcase
    end

    assign cr_finish = (state == STATE_FINISH);

    //=========================================================================
    // Module Instantiation
    //=========================================================================

    // --- Fetch ---
    fetch #(
        .INST_QUEUE_DEPTH(16),
        .INST_QUEUE_DEPTH_LOG(4)
    ) u_fetch (
        .launch        (cr_launch),
        .ins_baddr     (ins_baddr),
        .ins_count     (ins_count),
        .m_axi_arvalid (fetch_arvalid),
        .m_axi_arready (fetch_arready),
        .m_axi_araddr  (fetch_araddr),
        .m_axi_arlen   (fetch_arlen),
        .m_axi_rvalid  (fetch_rvalid),
        .m_axi_rready  (fetch_rready),
        .m_axi_rdata   (fetch_rdata),
        .m_axi_rlast   (fetch_rlast),
        .ld_inst_valid (fetch_ld_valid),
        .ld_inst_ready (fetch_ld_ready),
        .ld_inst       (fetch_ld_inst),
        .sp_inst_valid (fetch_sp_valid),
        .sp_inst_ready (fetch_sp_ready),
        .sp_inst       (fetch_sp_inst),
        .st_inst_valid (fetch_st_valid),
        .st_inst_ready (fetch_st_ready),
        .st_inst       (fetch_st_inst),
        .sch_inst_valid(fetch_sch_valid),
        .sch_inst_ready(fetch_sch_ready),
        .sch_inst      (),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // --- Decode SpGEMM instruction ---
    wire [15:0] sp_a_row_sram, sp_a_col_sram, sp_a_val_sram;
    wire [15:0] sp_b_row_sram, sp_b_col_sram, sp_b_val_sram;

    spgemm_decode u_spgemm_decode (
        .inst          (fetch_sp_inst),
        .a_row_ptr_sram(sp_a_row_sram),
        .a_col_idx_sram(sp_a_col_sram),
        .a_val_sram    (sp_a_val_sram),
        .b_row_ptr_sram(sp_b_row_sram),
        .b_col_idx_sram(sp_b_col_sram),
        .b_val_sram    (sp_b_val_sram),
        .M             (M),
        .K             (K),
        .N             (N)
    );

    // Latch SpGEMM parameters on instruction
    always @(posedge aclk) begin
        if (fetch_sp_valid && fetch_sp_ready) begin
            a_row_ptr_sram <= sp_a_row_sram;
            a_col_idx_sram <= sp_a_col_sram;
            a_val_sram     <= sp_a_val_sram;
            b_row_ptr_sram <= sp_b_row_sram;
            b_col_idx_sram <= sp_b_col_sram;
            b_val_sram     <= sp_b_val_sram;
        end
    end

    // Fetch instruction ready signals
    assign fetch_sp_ready = 1'b1;  // always ready; spgemm insts arrive after LOADs, state already past IDLE
    // fetch_ld_ready and fetch_st_ready are driven by load/store modules' inst_ready ports
    // (not assigned here, to avoid multiple-driver conflict with the modules' internal assign)
    assign fetch_sch_ready = 1'b0; // Scheduler doesn't need explicit inst

    // --- AXI Read Mux ---
    axi_read_mux #(.N_CLIENTS(2)) u_read_mux (
        .m_axi_arvalid (m_axi_arvalid),
        .m_axi_arready (m_axi_arready),
        .m_axi_araddr  (m_axi_araddr),
        .m_axi_arlen   (m_axi_arlen),
        .m_axi_arid    (),
        .m_axi_arsize  (),
        .m_axi_arburst (),
        .m_axi_rvalid  (m_axi_rvalid),
        .m_axi_rready  (m_axi_rready),
        .m_axi_rresp   (2'b00),
        .m_axi_rdata   (m_axi_rdata),
        .m_axi_rlast   (m_axi_rlast),
        .m_axi_rid     (4'b0),
        .s_axi_arvalid ({load_arvalid, fetch_arvalid}),
        .s_axi_arready ({load_arready, fetch_arready}),
        .s_axi_araddr  ({load_araddr,  fetch_araddr}),
        .s_axi_arlen   ({load_arlen,   fetch_arlen}),
        .s_axi_rvalid  ({load_rvalid,  fetch_rvalid}),
        .s_axi_rready  ({load_rready,  fetch_rready}),
        .s_axi_rdata   ({load_rdata,   fetch_rdata}),
        .s_axi_rlast   ({load_rlast,   fetch_rlast}),
        .aclk          (aclk),
        .aresetn       (aresetn)
    );

    // --- Global Buffer ---
    global_buffer #(
        .DEPTH(`GBUF_DEPTH), .DEPTH_LOG(`GBUF_DEPTH_LOG)
    ) u_global_buffer (
        .wr_en    (load_gbuf_wr_en),
        .wr_addr  (load_gbuf_wr_addr),
        .wr_data  (load_gbuf_wr_data),
        .rd_en    (gbuf_rd_en),
        .rd_addr  (gbuf_rd_addr),
        .rd_data  (gbuf_rd_data),
        .rd_valid (gbuf_rd_valid),
        .aclk     (aclk),
        .aresetn  (aresetn)
    );

    // --- Load ---
    load u_load (
        .inst_valid     (fetch_ld_valid),
        .inst_ready     (fetch_ld_ready),
        .inst_data      (fetch_ld_inst),
        .ext_valid      ((state == STATE_LOAD_A) || (state == STATE_LOAD_B)),
        .done           (load_done),
        .m_axi_arvalid  (load_arvalid),
        .m_axi_arready  (load_arready),
        .m_axi_araddr   (load_araddr),
        .m_axi_arlen    (load_arlen),
        .m_axi_rvalid   (load_rvalid),
        .m_axi_rready   (load_rready),
        .m_axi_rdata    (load_rdata),
        .m_axi_rlast    (load_rlast),
        .gbuf_wr_en     (load_gbuf_wr_en),
        .gbuf_wr_addr   (load_gbuf_wr_addr),
        .gbuf_wr_data   (load_gbuf_wr_data),
        .aclk           (aclk),
        .aresetn        (aresetn)
    );

    // --- Scheduler ---
    scheduler u_scheduler (
        .start         (sched_start),
        .done          (sched_done),
        .M             (M),
        .K             (K),
        .N             (N),
        .a_row_ptr_base(a_row_ptr_sram),
        .a_col_idx_base(a_col_idx_sram),
        .b_row_ptr_base(b_row_ptr_sram),
        .pe_row_start  (pe_row_start),
        .pe_row_end    (pe_row_end),
        .pe_a_ptr_start(pe_a_ptr_start),
        .pe_a_ptr_end  (pe_a_ptr_end),
        .pe_task_valid (pe_task_valid),
        // GlobalBuffer read interface
        .gbuf_rd_en   (gbuf_rd_en),
        .gbuf_rd_addr (gbuf_rd_addr),
        .gbuf_rd_data (gbuf_rd_data),
        .gbuf_rd_valid(gbuf_rd_valid),
        .aclk         (aclk),
        .aresetn      (aresetn)
    );
    assign sched_start = (state == STATE_SCHEDULE);

    // --- PE Array ---
    genvar pe_idx;
    generate
        for (pe_idx = 0; pe_idx < `N_PE; pe_idx = pe_idx + 1) begin : gen_pe_array
            pe_top #(
                .PE_ID(pe_idx)
            ) u_pe (
                .start          ((state == STATE_COMPUTE) && pe_task_valid[pe_idx]),
                .done           (pe_done[pe_idx]),
                .row_start      (pe_row_start[pe_idx]),
                .row_end        (pe_row_end[pe_idx]),
                .a_ptr_start    (pe_a_ptr_start[pe_idx]),
                .a_ptr_end      (pe_a_ptr_end[pe_idx]),
                .K              (K),
                .N              (N),
                // GlobalBuffer read for A CSR and B CSR
                .gbuf_rd_en     (),
                .gbuf_rd_addr   (),
                .gbuf_rd_data   (gbuf_rd_data),
                .gbuf_rd_valid  (gbuf_rd_valid),
                // Output to CSR Writer
                .out_row_id     (pe_out_row_id[pe_idx]),
                .out_nnz        (pe_out_nnz[pe_idx]),
                .out_col        (pe_out_col[pe_idx]),
                .out_val        (pe_out_val[pe_idx]),
                .out_valid      (pe_out_valid[pe_idx]),
                .aclk           (aclk),
                .aresetn        (aresetn)
            );
        end
    endgenerate

    assign pe_all_done = &pe_done;

    // --- C CSR Writer ---
    wire csr_done_int;
    wire                                 csr_wr_en;
    wire [`OUTBUF_DEPTH_LOG-1:0]         csr_wr_addr;
    wire [`DATA_WIDTH-1:0]               csr_wr_data;

    c_csr_writer u_csr_writer (
        .start       (state == STATE_WRITE_CSR),
        .done        (csr_done_int),
        .pe_row_valid(pe_out_valid),
        .pe_row_id   (pe_out_row_id),
        .pe_nnz      (pe_out_nnz),
        .pe_col      (pe_out_col),
        .pe_val      (pe_out_val),
        .M           (M),
        .obuf_wr_en  (csr_wr_en),
        .obuf_wr_addr(csr_wr_addr),
        .obuf_wr_data(csr_wr_data),
        .row_ptr_base_addr(),
        .col_val_base_addr(),
        .aclk        (aclk),
        .aresetn     (aresetn)
    );
    assign csr_done = csr_done_int;

    // --- Output Scratchpad ---
    wire                                 osp_rd_en;
    wire [`OUTBUF_DEPTH_LOG-1:0]         osp_rd_addr;
    wire [`AXI_DATA_WIDTH-1:0]           osp_rd_data;
    wire                                 osp_rd_valid;

    output_scratchpad #(
        .DEPTH(`OUTBUF_DEPTH), .DEPTH_LOG(`OUTBUF_DEPTH_LOG)
    ) u_outbuf (
        .wr_en   (csr_wr_en),
        .wr_addr (csr_wr_addr),
        .wr_data (csr_wr_data),
        .rd_en   (osp_rd_en),
        .rd_addr (osp_rd_addr),
        .rd_data (osp_rd_data),
        .rd_valid(osp_rd_valid),
        .aclk    (aclk),
        .aresetn (aresetn)
    );

    // --- Store ---
    store u_store (
        .inst_valid     (fetch_st_valid),
        .inst_ready     (fetch_st_ready),
        .inst_data      (fetch_st_inst),
        .ext_valid      (state == STATE_STORE),
        .done           (store_done),
        .osp_rd_en      (osp_rd_en),
        .osp_rd_addr    (osp_rd_addr),
        .osp_rd_data    (osp_rd_data),
        .osp_rd_valid   (osp_rd_valid),
        .m_axi_awvalid  (m_axi_awvalid),
        .m_axi_awready  (m_axi_awready),
        .m_axi_awaddr   (m_axi_awaddr),
        .m_axi_awlen    (m_axi_awlen),
        .m_axi_wvalid   (m_axi_wvalid),
        .m_axi_wready   (m_axi_wready),
        .m_axi_wdata    (m_axi_wdata),
        .m_axi_wstrb    (m_axi_wstrb),
        .m_axi_wlast    (m_axi_wlast),
        .m_axi_bvalid   (m_axi_bvalid),
        .m_axi_bready   (m_axi_bready),
        .m_axi_bresp    (m_axi_bresp),
        .aclk           (aclk),
        .aresetn        (aresetn)
    );

endmodule
