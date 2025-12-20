`include "riscv_define.v"

// Simple instruction buffer (FIFO) between IF and PreDecode.
// Accepts up to 2 fetched instructions per cycle, provides up to 2 to consumer,
// supports backpressure (stall_if) and flush.
module InstructionBuffer #(
    parameter DEPTH = 8 // number of instruction slots (must be >= 2)
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        flush,

    // Enqueue from IF
    input  wire [`IF_BATCH_SIZE-1:0]   in_valid,
    input  wire [`INST_WIDTH-1:0]      in_inst_0,
    input  wire [`INST_WIDTH-1:0]      in_inst_1,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_0,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pc_1,
    input  wire                        in_pred_taken_0,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pred_target_0,
    input  wire [`BP_GHR_BITS-1:0]     in_pred_hist_0,
    input  wire                        in_pred_taken_1,
    input  wire [`INST_ADDR_WIDTH-1:0] in_pred_target_1,
    input  wire [`BP_GHR_BITS-1:0]     in_pred_hist_1,

    // Consumer ready (when downstream can accept a new bundle)
    input  wire                        out_ready,

    // Outputs to PreDecode
    output wire [`IF_BATCH_SIZE-1:0]   out_valid,
    output wire [`INST_WIDTH-1:0]      out_inst_0,
    output wire [`INST_WIDTH-1:0]      out_inst_1,
    output wire [`INST_ADDR_WIDTH-1:0] out_pc_0,
    output wire [`INST_ADDR_WIDTH-1:0] out_pc_1,
    output wire                        out_pred_taken_0,
    output wire [`INST_ADDR_WIDTH-1:0] out_pred_target_0,
    output wire [`BP_GHR_BITS-1:0]     out_pred_hist_0,
    output wire                        out_pred_taken_1,
    output wire [`INST_ADDR_WIDTH-1:0] out_pred_target_1,
    output wire [`BP_GHR_BITS-1:0]     out_pred_hist_1,

    // Backpressure to IF when buffer is (near) full
    output wire                        stall_if
);
    localparam PTR_W = $clog2(DEPTH);

    reg [`INST_WIDTH-1:0]      fifo_inst   [0:DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] fifo_pc     [0:DEPTH-1];
    reg                        fifo_pred_taken  [0:DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] fifo_pred_target [0:DEPTH-1];
    reg [`BP_GHR_BITS-1:0]     fifo_pred_hist   [0:DEPTH-1];
    reg [PTR_W-1:0] head;
    reg [PTR_W-1:0] tail;
    reg [PTR_W:0]   count;
    reg [PTR_W-1:0] tail_next;

    wire [1:0] deq_cnt = out_ready ? ((count >= 2) ? 2'd2 : count[1:0]) : 2'd0;
    wire [1:0] in_cnt   = in_valid[0] + in_valid[1];
    wire [PTR_W:0] free_slots = DEPTH - count;
    wire [PTR_W:0] avail_next = free_slots + deq_cnt; // space after dequeue in this cycle
    wire can_enq = (in_cnt <= avail_next);
    assign stall_if = !can_enq;

    wire [PTR_W-1:0] head_plus_1 = (head == DEPTH-1) ? {PTR_W{1'b0}} : head + 1'b1;
    wire [PTR_W-1:0] head_plus_2 = (head_plus_1 == DEPTH-1) ? {PTR_W{1'b0}} : head_plus_1 + 1'b1;

    // Combinational outputs from current head
    assign out_valid[0] = (count != 0);
    assign out_valid[1] = (count > 1);
    assign out_inst_0   = out_valid[0] ? fifo_inst[head]       : {`INST_WIDTH{1'b0}};
    assign out_pc_0     = out_valid[0] ? fifo_pc[head]         : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pred_taken_0  = out_valid[0] ? fifo_pred_taken[head]  : 1'b0;
    assign out_pred_target_0 = out_valid[0] ? fifo_pred_target[head] : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pred_hist_0   = out_valid[0] ? fifo_pred_hist[head]   : {`BP_GHR_BITS{1'b0}};
    assign out_inst_1   = out_valid[1] ? fifo_inst[head_plus_1]: {`INST_WIDTH{1'b0}};
    assign out_pc_1     = out_valid[1] ? fifo_pc[head_plus_1]  : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pred_taken_1  = out_valid[1] ? fifo_pred_taken[head_plus_1]  : 1'b0;
    assign out_pred_target_1 = out_valid[1] ? fifo_pred_target[head_plus_1] : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pred_hist_1   = out_valid[1] ? fifo_pred_hist[head_plus_1]   : {`BP_GHR_BITS{1'b0}};

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head  <= {PTR_W{1'b0}};
            tail  <= {PTR_W{1'b0}};
            count <= {(PTR_W+1){1'b0}};
            for (i = 0; i < DEPTH; i = i + 1) begin
                fifo_inst[i] <= {`INST_WIDTH{1'b0}};
                fifo_pc[i]   <= {`INST_ADDR_WIDTH{1'b0}};
                fifo_pred_taken[i]  <= 1'b0;
                fifo_pred_target[i] <= {`INST_ADDR_WIDTH{1'b0}};
                fifo_pred_hist[i]   <= {`BP_GHR_BITS{1'b0}};
            end
        end else if (flush) begin
            head  <= {PTR_W{1'b0}};
            tail  <= {PTR_W{1'b0}};
            count <= {(PTR_W+1){1'b0}};
        end else begin
            // Dequeue
            if (deq_cnt != 0) begin
                case (deq_cnt)
                    2'd1: head <= head_plus_1;
                    2'd2: head <= head_plus_2;
                    default: ;
                endcase
            end

            // Enqueue (up to 2)
            if (can_enq && (in_cnt != 0)) begin
                tail_next = tail;
                if (in_valid[0]) begin
                    fifo_inst[tail_next] <= in_inst_0;
                    fifo_pc[tail_next]   <= in_pc_0;
                    fifo_pred_taken[tail_next]  <= in_pred_taken_0;
                    fifo_pred_target[tail_next] <= in_pred_target_0;
                    fifo_pred_hist[tail_next]   <= in_pred_hist_0;
                    tail_next = (tail_next == DEPTH-1) ? {PTR_W{1'b0}} : tail_next + 1'b1;
                end
                if (in_valid[1]) begin
                    fifo_inst[tail_next] <= in_inst_1;
                    fifo_pc[tail_next]   <= in_pc_1;
                    fifo_pred_taken[tail_next]  <= in_pred_taken_1;
                    fifo_pred_target[tail_next] <= in_pred_target_1;
                    fifo_pred_hist[tail_next]   <= in_pred_hist_1;
                    tail_next = (tail_next == DEPTH-1) ? {PTR_W{1'b0}} : tail_next + 1'b1;
                end
                tail <= tail_next;
            end

            // Update count (deq then enq)
            count <= count - deq_cnt + ((can_enq) ? in_cnt : 0);
        end
    end

endmodule
