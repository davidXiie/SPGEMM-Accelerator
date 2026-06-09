//=============================================================================
// File     : scheduler.v
// Project  : SPGEMM-Accelerator
// Brief    : Scheduler module - Workload analysis and Row-Block task partitioning
//
//   Sub-modules:
//     1. B Row Length Generator: b_row_nnz[k] = B_row_ptr[k+1] - B_row_ptr[k]
//     2. A Row Workload Analyzer: row_cyc[i] = ceil(row_eff[i] / N_MAC)
//        where row_eff[i] = sum(b_row_nnz[A_col_idx[p]])  (Row-Block formula)
//     3. Remaining-Aware Row Partitioner: dynamic_target + nearest boundary
//     4. Task Descriptor Generator: PE task descriptors including ptr ranges
//
//   Algorithm: "动态剩余目标 + 最近边界切分 + Row-Block 工作量估计"
//   - Row-Block: all A(i,k) B elements in one row are concatenated, one ceil() per row
//   - dynamic_target = ceil(remaining_work / remaining_pe)
//   - When cur_load + w >= dynamic_target, compare distances
//   - Protection: if cur_load == 0, must take the current row
//=============================================================================

`include "defines.vh"

module scheduler (
    input  wire                      start,
    output reg                       done,

    input  wire [`MAX_DIM_BITS-1:0]  M,
    input  wire [`MAX_DIM_BITS-1:0]  K,
    input  wire [`MAX_DIM_BITS-1:0]  N,

    // SRAM base addresses (offsets in GlobalBuffer, each entry = BANK_BLOCK_SIZE bits)
    input  wire [15:0]               a_row_ptr_base,
    input  wire [15:0]               a_col_idx_base,
    input  wire [15:0]               b_row_ptr_base,

    // Task output (to PEs)
    output reg  [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_row_start,
    output reg  [`N_PE-1:0][`MAX_DIM_BITS-1:0] pe_row_end,
    output reg  [`N_PE-1:0][15:0]              pe_a_ptr_start,
    output reg  [`N_PE-1:0][15:0]              pe_a_ptr_end,
    output reg  [`N_PE-1:0]                    pe_task_valid,

    // GlobalBuffer read interface
    output reg                       gbuf_rd_en,
    output reg  [`GBUF_DEPTH_LOG-1:0] gbuf_rd_addr,
    input  wire [`BANK_BLOCK_SIZE-1:0] gbuf_rd_data,
    input  wire                      gbuf_rd_valid,

    input  wire                      aclk,
    input  wire                      aresetn
);

    //=========================================================================
    // Phase State Machine
    //=========================================================================
    localparam PHASE_IDLE       = 4'd0;
    localparam PHASE_B_ROW_NNZ  = 4'd1;  // Compute b_row_nnz from B_row_ptr
    localparam PHASE_A_ANALYZE  = 4'd2;  // Row-Block: scan A_col_idx, compute row_cyc
    localparam PHASE_PARTITION  = 4'd3;  // Remaining-aware row partitioning
    localparam PHASE_GEN_TASK   = 4'd4;  // Generate task descriptors (read A_row_ptr)
    localparam PHASE_DONE       = 4'd5;

    reg [3:0] phase;
    reg [3:0] phase_next;

    //=========================================================================
    // Phase 2 (A Row Workload Analyzer) sub-states
    //=========================================================================
    localparam ANA_RD_PTR_I    = 3'd0;  // Read A_row_ptr[i]
    localparam ANA_RD_PTR_IP1  = 3'd1;  // Read A_row_ptr[i+1], check if non-empty
    localparam ANA_RD_COL      = 3'd2;  // Read A_col_idx[p], accumulate row_eff
    localparam ANA_ROW_DONE    = 3'd3;  // Store row_cyc[i], move to next row
    localparam ANA_ALL_DONE    = 3'd4;  // All rows processed

    reg [2:0] ana_sub;
    reg [2:0] ana_sub_next;

    //=========================================================================
    // Phase 4 (Task Descriptor Generator) sub-states
    //=========================================================================
    localparam TASK_RD_START   = 2'd0;  // Read A_row_ptr[pe_row_start[p]]
    localparam TASK_RD_END     = 2'd1;  // Read A_row_ptr[pe_row_end[p]+1], latch
    localparam TASK_NEXT_PE    = 2'd2;  // Move to next PE

    reg [1:0] task_sub;
    reg [1:0] task_sub_next;
    reg [`N_PE_BITS:0] task_pe_idx;  // current PE being processed

    //=========================================================================
    // B Row Length Generator State
    //=========================================================================
    reg [`MAX_DIM_BITS-1:0] b_row_ptr_idx;  // current row k being processed
    reg [15:0] b_row_ptr_k, b_row_ptr_kp1;
    reg [`WORKLOAD_BITS-1:0] b_row_nnz [`MAX_K-1:0];  // b_row_nnz[k] array
    reg b_read_phase;  // 0: read B_row_ptr[k], 1: read B_row_ptr[k+1]

    //=========================================================================
    // A Row Workload Analyzer State (Row-Block formula)
    //=========================================================================
    reg [`MAX_DIM_BITS-1:0] a_row_idx;     // current A row i
    reg [15:0] a_ptr_start_i, a_ptr_end_i;  // A_row_ptr[i], A_row_ptr[i+1]
    reg [15:0] a_ptr_current;               // current position p in A CSR
    reg [`WORKLOAD_BITS-1:0] row_eff_acc;   // accumulator: sum of b_row_nnz for current row
    reg [`WORKLOAD_BITS-1:0] row_cyc [`MAX_M-1:0];  // row_cyc[i] = ceil(row_eff[i]/N_MAC)
    reg [`WORKLOAD_BITS-1:0] total_cycle_work;

    //=========================================================================
    // Remaining-Aware Row Partitioner State
    //=========================================================================
    reg [`MAX_DIM_BITS-1:0] part_row_idx;   // current row being assigned
    reg [15:0] assigned_work;
    reg [15:0] cur_pe_load;
    reg [`N_PE_BITS:0] cur_pe;              // current PE index (0 to N_PE)
    reg [15:0] dynamic_target;
    reg [15:0] remaining_work;
    reg [15:0] remaining_pe;
    reg [`WORKLOAD_BITS-1:0] row_w;

    //=========================================================================
    // Pipeline registers
    //=========================================================================
    reg rdata_valid_r;
    reg [`BANK_BLOCK_SIZE-1:0] rdata_r;
    reg gbuf_rd_valid_r;

    //=========================================================================
    // Phase Transition (combinational)
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            phase <= PHASE_IDLE;
            ana_sub <= ANA_RD_PTR_I;
            task_sub <= TASK_RD_START;
        end else begin
            phase <= phase_next;
            ana_sub <= ana_sub_next;
            task_sub <= task_sub_next;
        end
    end

    always @(*) begin
        phase_next = phase;
        case (phase)
            PHASE_IDLE: begin
                if (start)
                    phase_next = PHASE_B_ROW_NNZ;
            end
            PHASE_B_ROW_NNZ: begin
                // Done when we've read B_row_ptr[K] (the last entry, index K)
                // b_read_phase==1 and rdata_valid_r means we just read B_row_ptr[b_row_ptr_idx+1]
                // which means b_row_ptr_idx has just been incremented to K-1's next
                // Actually: after processing k=K-1, b_row_ptr_idx becomes K, and then
                // the last read is B_row_ptr[K-1+1]=B_row_ptr[K]
                // We're done when b_row_ptr_idx == K and we've read the last pair
                if (b_row_ptr_idx == K && b_read_phase == 1'b1 && rdata_valid_r)    
                    phase_next = PHASE_A_ANALYZE;
            end
            PHASE_A_ANALYZE: begin
                if (ana_sub == ANA_ALL_DONE)
                    phase_next = PHASE_PARTITION;
            end
            PHASE_PARTITION: begin
                if (cur_pe == `N_PE && part_row_idx >= M)
                    phase_next = PHASE_GEN_TASK;
            end
            PHASE_GEN_TASK: begin
                // Done when all PEs processed
                if (task_pe_idx == `N_PE && task_sub == TASK_NEXT_PE)
                    phase_next = PHASE_DONE;
            end
            PHASE_DONE: begin
                phase_next = PHASE_IDLE;
            end
        endcase
    end

    //=========================================================================
    // Sub-state transition for Phase 2 (A Row Workload Analyzer)
    //=========================================================================
    always @(*) begin
        ana_sub_next = ana_sub;
        case (phase)
            PHASE_A_ANALYZE: begin
                case (ana_sub)
                    ANA_RD_PTR_I: begin
                        if (rdata_valid_r)
                            ana_sub_next = ANA_RD_PTR_IP1;
                    end
                    ANA_RD_PTR_IP1: begin
                        if (rdata_valid_r) begin
                            if (a_ptr_end_i != a_ptr_start_i)
                                ana_sub_next = ANA_RD_COL;  // non-empty row, scan col_idx
                            else
                                ana_sub_next = ANA_ROW_DONE; // empty row, skip
                        end
                    end
                    ANA_RD_COL: begin
                        if (rdata_valid_r) begin
                            if (a_ptr_current + 1 >= a_ptr_end_i) begin
                                // Last element of this row read
                                ana_sub_next = ANA_ROW_DONE;
                            end
                            // else stay in ANA_RD_COL for next element
                        end
                    end
                    ANA_ROW_DONE: begin
                        // Update row_cyc and move to next row
                        if (a_row_idx + 1 >= M)
                            ana_sub_next = ANA_ALL_DONE;
                        else
                            ana_sub_next = ANA_RD_PTR_I;
                    end
                endcase
            end
            default: ana_sub_next = ANA_RD_PTR_I;
        endcase
    end

    //=========================================================================
    // Sub-state transition for Phase 4 (Task Descriptor Generator)
    //=========================================================================
    always @(*) begin
        task_sub_next = task_sub;
        case (phase)
            PHASE_GEN_TASK: begin
                case (task_sub)
                    TASK_RD_START: begin
                        if (rdata_valid_r)
                            task_sub_next = TASK_RD_END;
                    end
                    TASK_RD_END: begin
                        if (rdata_valid_r)
                            task_sub_next = TASK_NEXT_PE;
                    end
                    TASK_NEXT_PE: begin
                        if (task_pe_idx < `N_PE)
                            task_sub_next = TASK_RD_START;
                    end
                endcase
            end
            default: task_sub_next = TASK_RD_START;
        endcase
    end

    //=========================================================================
    // Phase 1: B Row Length Generator
    //   Compute b_row_nnz[k] = B_row_ptr[k+1] - B_row_ptr[k]
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            b_row_ptr_idx <= 0;
            b_read_phase <= 1'b0;
            b_row_ptr_k <= 0;
            b_row_ptr_kp1 <= 0;
        end else if (phase == PHASE_IDLE && start) begin
            b_row_ptr_idx <= 0;
            b_read_phase <= 1'b0;
            b_row_ptr_k <= 0;
            b_row_ptr_kp1 <= 0;
        end else if (phase == PHASE_B_ROW_NNZ) begin
            if (rdata_valid_r) begin
                if (!b_read_phase) begin
                    // Just read B_row_ptr[k] (lower 32 bits)
                    b_row_ptr_k <= rdata_r[15:0];
                    b_read_phase <= 1'b1;
                end else begin
                    // Just read B_row_ptr[k+1]
                    b_row_ptr_kp1 <= rdata_r[15:0];
                    b_read_phase <= 1'b0;
                    b_row_ptr_idx <= b_row_ptr_idx + 1'b1;
                end
            end
        end
    end

    // Store b_row_nnz[k] when both reads complete
    always @(posedge aclk) begin
        if (phase == PHASE_B_ROW_NNZ && rdata_valid_r && b_read_phase) begin
            b_row_nnz[b_row_ptr_idx] <= rdata_r[15:0] - b_row_ptr_k;
        end
    end

    //=========================================================================
    // Phase 2: A Row Workload Analyzer (Row-Block formula)
    //   row_eff[i] = sum(b_row_nnz[A_col_idx[p]])  for all A(i,k)
    //   row_cyc[i] = ceil(row_eff[i] / N_MAC)
    //
    //   Key difference from old formula:
    //   Old: row_cyc[i] = sum(ceil(b_row_nnz[k] / N_MAC))  — per-element ceil
    //   New: row_cyc[i] = ceil(sum(b_row_nnz[k]) / N_MAC)  — single ceil per row
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            a_row_idx <= 0;
            a_ptr_start_i <= 0;
            a_ptr_end_i <= 0;
            a_ptr_current <= 0;
            row_eff_acc <= 0;
            total_cycle_work <= 0;
        end else if (phase == PHASE_IDLE && start) begin
            a_row_idx <= 0;
            a_ptr_start_i <= 0;
            a_ptr_end_i <= 0;
            a_ptr_current <= 0;
            row_eff_acc <= 0;
            total_cycle_work <= 0;
        end else if (phase == PHASE_A_ANALYZE) begin
            if (rdata_valid_r) begin
                case (ana_sub)
                    ANA_RD_PTR_I: begin
                        // Got A_row_ptr[i] from [15:0]
                        a_ptr_start_i <= rdata_r[15:0];
                    end
                    ANA_RD_PTR_IP1: begin
                        // Got A_row_ptr[i+1]
                        a_ptr_end_i <= rdata_r[15:0];
                        a_ptr_current <= a_ptr_start_i;  // start scanning from p = a_start
                        row_eff_acc <= 0;  // clear accumulator for new row
                    end
                    ANA_RD_COL: begin
                        // Got A_col_idx[p] = k (lower 32 bits)
                        // Look up b_row_nnz[k] and accumulate row_eff
                        row_eff_acc <= row_eff_acc + b_row_nnz[rdata_r[`MAX_DIM_BITS-1:0]];
                        a_ptr_current <= a_ptr_current + 1;
                    end
                    ANA_ROW_DONE: begin
                        // Store row_cyc[i] = ceil(row_eff / N_MAC)
                        // total_cycle_work updated below
                        a_row_idx <= a_row_idx + 1'b1;
                    end
                endcase
            end
        end
    end

    // Store row_cyc[i] and accumulate total_cycle_work
    // (separate block because the ceil computation uses row_eff_acc from previous state)
    always @(posedge aclk) begin
        if (phase == PHASE_A_ANALYZE && rdata_valid_r) begin
            if (ana_sub == ANA_RD_COL && a_ptr_current + 1 >= a_ptr_end_i) begin
                // Last element: compute row_cyc = ceil((row_eff_acc + b_row_nnz[just_read]) / N_MAC)
                // row_eff_acc hasn't been updated yet (it updates same cycle)
                // So we compute: (row_eff_acc + b_row_nnz[k]) / N_MAC round up
                row_cyc[a_row_idx] <= (row_eff_acc + b_row_nnz[rdata_r[`MAX_DIM_BITS-1:0]] + `N_MAC - 1) >> `N_MAC_BITS;
                total_cycle_work <= total_cycle_work + ((row_eff_acc + b_row_nnz[rdata_r[`MAX_DIM_BITS-1:0]] + `N_MAC - 1) >> `N_MAC_BITS);
            end else if (ana_sub == ANA_ROW_DONE && a_ptr_start_i == a_ptr_end_i) begin
                // Empty row: row_cyc = 0
                row_cyc[a_row_idx] <= 0;
                total_cycle_work <= total_cycle_work;
            end
        end
    end

    //=========================================================================
    // Phase 3: Remaining-Aware Row Partitioner
    //   dynamic_target = ceil(remaining_work / remaining_pe)
    //   Nearest boundary comparison
    //=========================================================================
    wire [15:0] new_load = cur_pe_load + row_w;
    wire cross_boundary = (new_load >= dynamic_target) && (remaining_pe > 1);
    wire [15:0] err_before = dynamic_target - cur_pe_load;
    wire [15:0] err_after  = new_load - dynamic_target;
    wire take_this_row = (cur_pe_load == 0) || (err_after <= err_before) || !cross_boundary;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            part_row_idx <= 0;
            assigned_work <= 0;
            cur_pe_load <= 0;
            cur_pe <= 0;
            dynamic_target <= 0;
            remaining_work <= 0;
            remaining_pe <= 0;
            row_w <= 0;
        end else if (phase == PHASE_A_ANALYZE && ana_sub == ANA_ALL_DONE) begin
            // Initialize partitioning
            part_row_idx <= 0;
            assigned_work <= 0;
            cur_pe_load <= 0;
            cur_pe <= 0;
            pe_row_start[0] <= 0;
        end else if (phase == PHASE_PARTITION) begin
            // Load current row workload
            row_w <= row_cyc[part_row_idx];
            remaining_work <= (total_cycle_work > assigned_work) ? total_cycle_work - assigned_work : 0;
            remaining_pe <= (`N_PE > cur_pe) ? (`N_PE - cur_pe) : 1;
            dynamic_target <= ((total_cycle_work > assigned_work) && (`N_PE > cur_pe)) ?
                              ((total_cycle_work - assigned_work + `N_PE - cur_pe - 1) / (`N_PE - cur_pe)) : 0;

            // Decision: assign current row to current PE or move to next PE?
            if (cur_pe >= `N_PE) begin
                // All PEs assigned, force remaining rows to last PE
                pe_row_end[`N_PE-1] <= M - 1;
                part_row_idx <= M;  // jump to end
            end else if (take_this_row) begin
                // Take this row
                cur_pe_load <= cur_pe_load + row_w;
                pe_row_end[cur_pe] <= part_row_idx;
                if (part_row_idx + 1 >= M) begin
                    // Last row, close current PE
                    assigned_work <= assigned_work + cur_pe_load + row_w;
                    cur_pe <= `N_PE;  // mark all done
                end
                part_row_idx <= part_row_idx + 1'b1;
            end else begin
                // Move to next PE
                assigned_work <= assigned_work + cur_pe_load;
                cur_pe <= cur_pe + 1'b1;
                cur_pe_load <= 0;
                pe_row_start[cur_pe + 1] <= part_row_idx;
            end
        end
    end

    //=========================================================================
    // Phase 4: Task Descriptor Generator
    //   For each PE p, read A_row_ptr[pe_row_start[p]] and A_row_ptr[pe_row_end[p]+1]
    //   to generate pe_a_ptr_start[p] and pe_a_ptr_end[p].
    //=========================================================================
    reg [15:0] task_start_val;  // temporary latch for A_row_ptr[start]
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            task_pe_idx <= 0;
            task_start_val <= 0;
        end else if (phase == PHASE_PARTITION && phase_next == PHASE_GEN_TASK) begin
            task_pe_idx <= 0;
            task_start_val <= 0;
            // Initialize PE task valid signals
            for (integer p = 0; p < `N_PE; p = p + 1) begin
                pe_task_valid[p] <= (pe_row_end[p] >= pe_row_start[p]);
            end
        end else if (phase == PHASE_GEN_TASK) begin
            if (rdata_valid_r) begin
                case (task_sub)
                    TASK_RD_START: begin
                        // Got A_row_ptr[pe_row_start[p]]
                        task_start_val <= rdata_r[15:0];
                    end
                    TASK_RD_END: begin
                        // Got A_row_ptr[pe_row_end[p]+1]
                        pe_a_ptr_start[task_pe_idx] <= task_start_val;
                        pe_a_ptr_end[task_pe_idx]   <= rdata_r[15:0];
                    end
                    TASK_NEXT_PE: begin
                        // No data latch here, just advance PE index
                    end
                endcase
            end
            // Advance PE index after completing TASK_NEXT_PE state
            if (task_sub == TASK_NEXT_PE) begin
                task_pe_idx <= task_pe_idx + 1'b1;
            end
        end
    end

    //=========================================================================
    // GlobalBuffer Read Address Generation
    //   Each GBuf address returns BANK_BLOCK_SIZE (64 bits).
    //   Row_ptr/col_idx values are 16-bit, extracted from [15:0] of result.
    //=========================================================================
    always @(*) begin
        gbuf_rd_en = 1'b0;
        gbuf_rd_addr = 0;

        case (phase)
            PHASE_B_ROW_NNZ: begin
                gbuf_rd_en = 1'b1;
                // B_row_ptr[b_row_ptr_idx + b_read_phase] at offset b_row_ptr_base
                gbuf_rd_addr = b_row_ptr_base[`GBUF_DEPTH_LOG-1:0]
                             + b_row_ptr_idx
                             + b_read_phase;
            end

            PHASE_A_ANALYZE: begin
                gbuf_rd_en = 1'b1;
                case (ana_sub)
                    ANA_RD_PTR_I: begin
                        // Read A_row_ptr[a_row_idx]
                        gbuf_rd_addr = a_row_ptr_base[`GBUF_DEPTH_LOG-1:0] + a_row_idx;
                    end
                    ANA_RD_PTR_IP1: begin
                        // Read A_row_ptr[a_row_idx+1]
                        gbuf_rd_addr = a_row_ptr_base[`GBUF_DEPTH_LOG-1:0] + a_row_idx + 1;
                    end
                    ANA_RD_COL: begin
                        // Read A_col_idx[a_ptr_current]
                        gbuf_rd_addr = a_col_idx_base[`GBUF_DEPTH_LOG-1:0] + a_ptr_current[`GBUF_DEPTH_LOG-1:0];
                    end
                    default: gbuf_rd_en = 1'b0;
                endcase
            end

            PHASE_GEN_TASK: begin
                case (task_sub)
                    TASK_RD_START: begin
                        if (task_pe_idx < `N_PE) begin
                            gbuf_rd_en = 1'b1;
                            // Read A_row_ptr[pe_row_start[p]]
                            gbuf_rd_addr = a_row_ptr_base[`GBUF_DEPTH_LOG-1:0]
                                         + {9'd0, pe_row_start[task_pe_idx]};
                        end
                    end
                    TASK_RD_END: begin
                        if (task_pe_idx < `N_PE) begin
                            gbuf_rd_en = 1'b1;
                            // Read A_row_ptr[pe_row_end[p] + 1]
                            gbuf_rd_addr = a_row_ptr_base[`GBUF_DEPTH_LOG-1:0]
                                         + {9'd0, pe_row_end[task_pe_idx]}
                                         + 1;
                        end
                    end
                endcase
            end
        endcase
    end

    //=========================================================================
    // Pipeline: latch GBuf read valid and data
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            rdata_valid_r <= 1'b0;
            rdata_r <= 0;
            gbuf_rd_valid_r <= 1'b0;
        end else begin
            gbuf_rd_valid_r <= gbuf_rd_valid;
            rdata_valid_r <= gbuf_rd_valid;
            if (gbuf_rd_valid)
                rdata_r <= gbuf_rd_data;
        end
    end

    //=========================================================================
    // Done signal
    //=========================================================================
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            done <= 1'b0;
        end else begin
            done <= (phase == PHASE_DONE);
        end
    end

endmodule
