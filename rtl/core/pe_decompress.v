//=============================================================================
// File     : pe_decompress.v
// Project  : SPGEMM-Accelerator
// Brief    : Decompress / Sparse Decode Unit with Row-Block execution.
//
//   Row-Block 行块拼接执行:
//     同一 A 行所有 A(i,k) 对应的 B(k,:) 拼接为连续流，统一按 N_MAC 分块。
//     通过预加载机制消除跨 k 边界的空闲周期，只有行末 batch 有 potential waste。
//
//   Per-A-Row Two-Phase:
//     Phase A (PRE_SCAN): 扫描 A_col_idx → 每个 k，读 B_row_ptr[k]/[k+1] 得 b_len_k，
//                         累加 total_B_nnz，将 (a_val, b_start, b_len) 写入 entry_buf。
//     Phase B (ROW_STREAM): 连续回放 entry_buf，跨 k 拼接 B 元素。
//                           entry pre-load 消除边界 gap。
//
//   Scheduling formula (in scheduler.v):
//     row_cyc[i] = ceil(total_B_nnz / N_MAC)     -- Row-Block formula
//     Old:        sum(ceil(b_len_k / N_MAC))      -- per-element waste
//=============================================================================

`include "defines.vh"

module pe_decompress (
    input  wire                      start,
    output reg                       done,

    input  wire [`MAX_DIM_BITS-1:0]  row_start,
    input  wire [`MAX_DIM_BITS-1:0]  row_end,
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,

    // A Buffer
    output reg                       a_buf_rd_en,
    output reg  [`PE_ABUF_DEPTH_LOG-1:0] a_buf_rd_addr,
    input  wire [`DATA_WIDTH-1:0]    a_buf_rd_data,
    input  wire                      a_buf_rd_valid,

    // B Buffer (per-lane addr)
    output reg  [`N_MAC-1:0]         b_buf_rd_en,
    output reg  [`N_MAC*`PE_BBUF_DEPTH_LOG-1:0] b_buf_rd_addr,
    input  wire [`N_MAC*`DATA_WIDTH-1:0] b_buf_rd_data,
    input  wire [`N_MAC-1:0]         b_buf_rd_valid,

    // MAC lane output
    output reg  [`N_MAC-1:0]         lane_valid,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_a_val,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_b_val,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_col_idx,
    output reg  [`N_MAC*`DATA_WIDTH-1:0] lane_row_idx,

    input  wire                      aclk,
    input  wire                      aresetn
);

    localparam ENTRY_BUF_DEPTH = 64;
    localparam ENTRY_BUF_BITS  = 6;
    // Entry: [41:32] b_len (10b), [31:16] b_start (16b), [15:0] a_val (16b)
    localparam E_LEN_HI  = 41;
    localparam E_LEN_LO  = 32;
    localparam E_START_HI = 31;
    localparam E_START_LO = 16;
    localparam E_VAL_HI  = 15;
    localparam E_VAL_LO  = 0;

    //=========================================================================
    // State Machine
    //=========================================================================
    localparam ST_IDLE       = 3'd0;
    localparam ST_ROW_SETUP  = 3'd1;
    localparam ST_PRE_SCAN   = 3'd2;
    localparam ST_ROW_STREAM = 3'd3;
    localparam ST_DONE       = 3'd4;
    reg [2:0] state, state_next;

    // Pre-Scan sub
    localparam PS_INIT  = 2'd0;
    localparam PS_A_RD  = 2'd1;   // waiting for A_col_idx read → k
    localparam PS_B_RD0 = 2'd2;   // waiting for B_row_ptr[k]   → b_start
    localparam PS_B_RD1 = 2'd3;   // waiting for B_row_ptr[k+1] → b_end, b_len
    reg [1:0] ps_sub, ps_sub_next;

    // Row-Stream: always in active streaming (no sub-states needed anymore)
    // Pre-load logic eliminates gaps.

    //=========================================================================
    // A Row / Element State
    //=========================================================================
    reg [`MAX_DIM_BITS-1:0] current_row;
    reg [15:0] a_ptr_start_i, a_ptr_end_i;
    reg [15:0] a_ptr_current;
    reg [`DATA_WIDTH-1:0]    a_val_cur;
    reg [`MAX_DIM_BITS-1:0]  a_k_cur;
    reg [15:0] b_start_cur, b_len_cur;

    // Entry Buffer
    reg [41:0] entry_buf [0:ENTRY_BUF_DEPTH-1];
    reg [ENTRY_BUF_BITS-1:0] entry_wr_ptr;
    reg [ENTRY_BUF_BITS:0]   entry_count;
    reg [ENTRY_BUF_BITS-1:0] entry_rd_ptr;

    // Pre-loaded "next" entry registers (loaded 1 cycle ahead of need)
    reg                        next_valid;    // 1: next entry is pre-loaded
    reg [`DATA_WIDTH-1:0]      next_a_val;
    reg [15:0]                 next_b_start;
    reg [15:0]                 next_b_len;

    // Concatenated B Stream
    reg [`WORKLOAD_BITS-1:0] total_B_nnz;
    reg [`WORKLOAD_BITS-1:0] total_batches;
    reg [`WORKLOAD_BITS-1:0] batch_cnt;
    reg [`WORKLOAD_BITS-1:0] b_pos_in_concat;
    reg [`WORKLOAD_BITS-1:0] b_rem;    // remaining in current entry's B segment

    // Pipeline
    reg a_buf_rd_valid_r;
    reg [`DATA_WIDTH-1:0] a_buf_rd_data_r;
    reg b_buf_rd_valid_r;
    reg [`N_MAC*`DATA_WIDTH-1:0] b_buf_rd_data_r;

    // A row_ptr load tracking
    reg a_ptr_end_i_loaded;

    // Row-Block streaming base address
    reg [15:0] b_base;

    //=========================================================================
    // State Transition
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state  <= ST_IDLE;
            ps_sub <= PS_INIT;
        end else begin
            state  <= state_next;
            ps_sub <= ps_sub_next;
        end
    end

    always @(*) begin
        state_next = state;
        case (state)
            ST_IDLE:      if (start) state_next = ST_ROW_SETUP;
            ST_ROW_SETUP: if (a_buf_rd_valid_r && a_ptr_end_i_loaded) begin
                state_next = (a_ptr_end_i > a_ptr_start_i) ? ST_PRE_SCAN : ST_DONE;
            end
            ST_PRE_SCAN: begin
                // Done when all A elements scanned (detected by ps_sub returning to INIT)
                if (ps_sub == PS_A_RD && a_ptr_current >= a_ptr_end_i)
                    state_next = ST_ROW_STREAM;
            end
            ST_ROW_STREAM: begin
                if (batch_cnt >= total_batches)
                    state_next = ST_DONE;
            end
            ST_DONE: begin
                if (current_row >= row_end) state_next = ST_IDLE;
                else                        state_next = ST_ROW_SETUP;
            end
            default: state_next = ST_IDLE;
        endcase
    end

    always @(*) begin
        ps_sub_next = ps_sub;
        if (state == ST_PRE_SCAN) begin
            case (ps_sub)
                PS_INIT:  ps_sub_next = PS_A_RD;   // issue first A read
                PS_A_RD:  if (a_buf_rd_valid_r) ps_sub_next = PS_B_RD0;
                PS_B_RD0: if (b_buf_rd_valid_r) ps_sub_next = PS_B_RD1;
                PS_B_RD1: if (b_buf_rd_valid_r) begin
                    if (a_ptr_current >= a_ptr_end_i)
                        ps_sub_next = PS_INIT;     // all done
                    else
                        ps_sub_next = PS_A_RD;     // next A element (already issued)
                end
            endcase
        end else begin
            ps_sub_next = PS_INIT;
        end
    end

    //=========================================================================
    // Sequential Logic
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            current_row    <= 0;
            a_ptr_start_i  <= 0;
            a_ptr_end_i    <= 0;
            a_ptr_current  <= 0;
            a_val_cur      <= 0;
            a_k_cur        <= 0;
            b_start_cur    <= 0;
            b_len_cur      <= 0;
            entry_wr_ptr   <= 0;
            entry_count    <= 0;
            entry_rd_ptr   <= 0;
            next_valid     <= 0;
            next_a_val     <= 0;
            next_b_start   <= 0;
            next_b_len     <= 0;
            total_B_nnz    <= 0;
            total_batches  <= 0;
            batch_cnt      <= 0;
            b_pos_in_concat <= 0;
            b_rem          <= 0;
        end else begin

            //--- ST_ROW_SETUP ---
            if (state == ST_ROW_SETUP && a_buf_rd_valid_r) begin
                a_ptr_start_i <= a_buf_rd_data_r;
                a_ptr_end_i   <= a_buf_rd_data_r;  // same cycle: both reads complete sequentially...
                // Actually: first read is A_row_ptr[i], second is A_row_ptr[i+1]
                // We detect by a_ptr_end_i_loaded state
            end

            // Proper A_row_ptr loading (must fix: store both values)
            // Handled below in separate block

            //--- ST_PRE_SCAN ---
            if (state == ST_PRE_SCAN) begin
                case (ps_sub)
                    PS_A_RD: begin
                        if (a_buf_rd_valid_r) begin
                            a_k_cur   <= a_buf_rd_data_r[`MAX_DIM_BITS-1:0];
                            a_val_cur <= a_buf_rd_data_r;
                            a_ptr_current <= a_ptr_current + 1;
                        end
                    end
                    PS_B_RD0: begin
                        if (b_buf_rd_valid_r)
                            b_start_cur <= b_buf_rd_data_r[15:0];
                    end
                    PS_B_RD1: begin
                        if (b_buf_rd_valid_r) begin
                            b_len_cur  <= b_buf_rd_data_r[15:0] - b_start_cur;
                            entry_buf[entry_wr_ptr] <= {
                                b_buf_rd_data_r[15:0] - b_start_cur,  // b_len
                                b_start_cur[15:0],
                                a_val_cur
                            };
                            entry_wr_ptr <= entry_wr_ptr + 1;
                            total_B_nnz  <= total_B_nnz + (b_buf_rd_data_r[15:0] - b_start_cur);
                        end
                    end
                endcase
            end

            // End of pre-scan → init stream
            if (state == ST_PRE_SCAN && state_next == ST_ROW_STREAM) begin
                entry_count    <= entry_wr_ptr;
                entry_rd_ptr   <= 0;
                total_batches  <= (total_B_nnz + `N_MAC - 1) >> `N_MAC_BITS;
                batch_cnt      <= 0;
                b_pos_in_concat <= 0;
                // Load first entry
                b_start_cur    <= entry_buf[0][E_START_HI:E_START_LO];
                b_len_cur      <= entry_buf[0][E_LEN_HI:E_LEN_LO];
                a_val_cur      <= entry_buf[0][E_VAL_HI:E_VAL_LO];
                b_rem          <= entry_buf[0][E_LEN_HI:E_LEN_LO];
                // Pre-load second entry if exists
                if (entry_wr_ptr > 1) begin
                    next_valid   <= 1'b1;
                    next_a_val   <= entry_buf[1][E_VAL_HI:E_VAL_LO];
                    next_b_start <= entry_buf[1][E_START_HI:E_START_LO];
                    next_b_len   <= entry_buf[1][E_LEN_HI:E_LEN_LO];
                end else begin
                    next_valid <= 1'b0;
                end
            end

            //--- ST_ROW_STREAM: continuous B streaming with pre-load ---
            if (state == ST_ROW_STREAM) begin

                // Pre-load next-next entry (2 ahead) when current batch
                // will exhaust current entry (b_rem <= N_MAC)
                if (b_rem <= `N_MAC && next_valid && entry_rd_ptr + 2 < entry_count) begin
                    next_a_val   <= entry_buf[entry_rd_ptr + 2][E_VAL_HI:E_VAL_LO];
                    next_b_start <= entry_buf[entry_rd_ptr + 2][E_START_HI:E_START_LO];
                    next_b_len   <= entry_buf[entry_rd_ptr + 2][E_LEN_HI:E_LEN_LO];
                end

                if (b_buf_rd_valid_r) begin
                    batch_cnt      <= batch_cnt + 1;
                    b_pos_in_concat <= b_pos_in_concat + `N_MAC;
                    if (b_rem > `N_MAC) begin
                        b_rem <= b_rem - `N_MAC;
                        // Still in current entry, no transition
                    end else begin
                        // Current entry exhausted this batch (b_rem <= N_MAC).
                        // consume_from_next = N_MAC - b_rem elements taken from next entry.
                        if (next_valid && next_b_len > (`N_MAC - b_rem)) begin
                            // Next entry has enough remaining elements
                            b_rem       <= next_b_len - (`N_MAC - b_rem);
                            b_start_cur <= next_b_start;
                            b_len_cur   <= next_b_len;
                            a_val_cur   <= next_a_val;
                            entry_rd_ptr <= entry_rd_ptr + 1;
                            if (entry_rd_ptr + 2 < entry_count) begin
                                next_valid <= 1'b1;
                            end else begin
                                next_valid <= 1'b0;
                            end
                        end else if (next_valid) begin
                            // Next entry also exhausted in this batch!
                            // Advance past it, try to load the entry after that.
                            b_rem       <= 0;
                            entry_rd_ptr <= entry_rd_ptr + 2;
                            if (entry_rd_ptr + 3 < entry_count) begin
                                b_start_cur <= entry_buf[entry_rd_ptr + 2][E_START_HI:E_START_LO];
                                b_len_cur   <= entry_buf[entry_rd_ptr + 2][E_LEN_HI:E_LEN_LO];
                                a_val_cur   <= entry_buf[entry_rd_ptr + 2][E_VAL_HI:E_VAL_LO];
                                next_valid  <= 1'b1;
                                // Pre-load next-next
                                next_a_val   <= entry_buf[entry_rd_ptr + 3][E_VAL_HI:E_VAL_LO];
                                next_b_start <= entry_buf[entry_rd_ptr + 3][E_START_HI:E_START_LO];
                                next_b_len   <= entry_buf[entry_rd_ptr + 3][E_LEN_HI:E_LEN_LO];
                            end else begin
                                next_valid <= 1'b0;
                            end
                        end else begin
                            // No more entries: row streaming will conclude
                            b_rem <= 0;
                        end
                    end
                end
            end

            //--- ST_DONE: next row ---
            if (state == ST_DONE && current_row < row_end) begin
                current_row    <= current_row + 1;
                a_ptr_start_i  <= 0;
                a_ptr_end_i    <= 0;
                a_ptr_current  <= 0;
                entry_wr_ptr   <= 0;
                entry_count    <= 0;
                entry_rd_ptr   <= 0;
                next_valid     <= 0;
                total_B_nnz    <= 0;
                total_batches  <= 0;
                batch_cnt      <= 0;
                b_pos_in_concat <= 0;
                b_rem          <= 0;
            end

            if (state == ST_IDLE && start) begin
                current_row    <= row_start;
                a_ptr_start_i  <= 0;
                a_ptr_end_i    <= 0;
                a_ptr_current  <= 0;
                entry_wr_ptr   <= 0;
                entry_count    <= 0;
                entry_rd_ptr   <= 0;
                next_valid     <= 0;
                total_B_nnz    <= 0;
                total_batches  <= 0;
                batch_cnt      <= 0;
                b_pos_in_concat <= 0;
                b_rem          <= 0;
            end
        end
    end

    // A_row_ptr sequential load: read [i] then [i+1] from A buffer
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            a_ptr_end_i_loaded <= 1'b0;
        end else if (state == ST_IDLE || state == ST_DONE) begin
            a_ptr_end_i_loaded <= 1'b0;
        end else if (state == ST_ROW_SETUP && a_buf_rd_valid_r) begin
            if (!a_ptr_end_i_loaded) begin
                a_ptr_start_i <= a_buf_rd_data_r;
                a_ptr_end_i_loaded <= 1'b1;
            end else begin
                a_ptr_end_i   <= a_buf_rd_data_r;
                a_ptr_end_i_loaded <= 1'b0;
            end
        end
    end

    //=========================================================================
    // Buffer Read Address Generation
    //=========================================================================
    always @(*) begin
        a_buf_rd_en   = 1'b0;
        a_buf_rd_addr = 0;
        b_buf_rd_en   = {`N_MAC{1'b0}};
        b_buf_rd_addr = 0;

        case (state)

            ST_ROW_SETUP: begin
                a_buf_rd_en   = 1'b1;
                a_buf_rd_addr = current_row + {9'd0, a_ptr_end_i_loaded};
            end

            ST_PRE_SCAN: begin
                case (ps_sub)
                    PS_A_RD: begin
                        a_buf_rd_en   = 1'b1;
                        a_buf_rd_addr = a_ptr_current;
                    end
                    PS_B_RD0: begin
                        b_buf_rd_en[0] = 1'b1;
                        b_buf_rd_addr[0 +: `PE_BBUF_DEPTH_LOG] = a_k_cur[`PE_BBUF_DEPTH_LOG-1:0];
                    end
                    PS_B_RD1: begin
                        b_buf_rd_en[0] = 1'b1;
                        b_buf_rd_addr[0 +: `PE_BBUF_DEPTH_LOG] = a_k_cur + 1;
                    end
                    default: ;
                endcase
            end

            ST_ROW_STREAM: begin
                // Row-Block B streaming: read N_MAC B elements from current entry's segment.
                // b_start_cur + (b_len_cur - b_rem) = offset within current B row.
                b_base = b_start_cur + (b_len_cur - b_rem);
                for (integer m = 0; m < `N_MAC; m = m + 1) begin
                    if (batch_cnt < total_batches && (b_pos_in_concat + m < total_B_nnz)) begin
                        if (m < b_rem) begin
                            // Within current entry
                            b_buf_rd_en[m] = 1'b1;
                            b_buf_rd_addr[m*`PE_BBUF_DEPTH_LOG +: `PE_BBUF_DEPTH_LOG]
                                = b_base + m;
                        end else if (next_valid) begin
                            // Cross boundary: read from next entry
                            b_buf_rd_en[m] = 1'b1;
                            b_buf_rd_addr[m*`PE_BBUF_DEPTH_LOG +: `PE_BBUF_DEPTH_LOG]
                                = next_b_start + (m - b_rem);
                        end else begin
                            b_buf_rd_en[m] = 1'b0;
                        end
                    end else begin
                        b_buf_rd_en[m] = 1'b0;
                    end
                end
            end

        endcase
    end

    //=========================================================================
    // Pipeline
    //=========================================================================
    always @(posedge aclk) begin
        a_buf_rd_valid_r <= a_buf_rd_valid;
        a_buf_rd_data_r  <= a_buf_rd_data;
        b_buf_rd_valid_r <= |b_buf_rd_valid;
        b_buf_rd_data_r  <= b_buf_rd_data;
    end

    //=========================================================================
    // MAC Lane Dispatcher
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            lane_valid   <= 0;
            lane_a_val   <= 0;
            lane_b_val   <= 0;
            lane_col_idx <= 0;
            lane_row_idx <= 0;
        end else begin
            lane_valid <= 0;

            if (state == ST_ROW_STREAM && b_buf_rd_valid_r) begin
                for (integer m = 0; m < `N_MAC; m = m + 1) begin
                    if (b_buf_rd_en[m] && (b_pos_in_concat + m < total_B_nnz)) begin
                        lane_valid[m] <= 1'b1;
                        // a_val: current entry's value for lanes within b_rem
                        //        next entry's value for lanes beyond b_rem
                        if (m < b_rem)
                            lane_a_val[m*`DATA_WIDTH +: `DATA_WIDTH] <= a_val_cur;
                        else if (next_valid)
                            lane_a_val[m*`DATA_WIDTH +: `DATA_WIDTH] <= next_a_val;
                        lane_b_val[m*`DATA_WIDTH +: `DATA_WIDTH]
                            <= b_buf_rd_data_r[m*`DATA_WIDTH +: `DATA_WIDTH];
                        lane_col_idx[m*`DATA_WIDTH +: `DATA_WIDTH]
                            <= b_buf_rd_data_r[m*`DATA_WIDTH +: `DATA_WIDTH];
                        lane_row_idx[m*`DATA_WIDTH +: `DATA_WIDTH]
                            <= {6'd0, current_row};
                    end
                end
            end
        end
    end

    //=========================================================================
    // Done
    //=========================================================================
    always @(*) begin
        done = (state == ST_DONE) && (current_row >= row_end);
    end

endmodule
