`include "riscv_define.v"
// Fetch instruction from I$, predict branch, and send to decode.
// This version integrates a simple Gshare predictor and outputs prediction
// metadata alongside the fetched instructions.
module IF(
    input clk,
    input rst_n,
    input stall,
    input flush,
    input redirect_valid,
    input [`INST_ADDR_WIDTH-1:0] redirect_pc,

    // Branch predictor updates from execute
    input bp_update0_valid,
    input [`INST_ADDR_WIDTH-1:0] bp_update0_pc,
    input bp_update0_taken,
    input [`INST_ADDR_WIDTH-1:0] bp_update0_target,
    input [`BP_GHR_BITS-1:0]     bp_update0_hist,
    input bp_update0_is_call,
    input bp_update0_is_return,
    input bp_update1_valid,
    input [`INST_ADDR_WIDTH-1:0] bp_update1_pc,
    input bp_update1_taken,
    input [`INST_ADDR_WIDTH-1:0] bp_update1_target,
    input [`BP_GHR_BITS-1:0]     bp_update1_hist,
    input bp_update1_is_call,
    input bp_update1_is_return,

    // Outputs
    output reg [`INST_WIDTH-1:0] out_inst_addr_0,
    output reg [`INST_WIDTH-1:0] out_inst_addr_1,
    output reg [`INST_WIDTH-1:0] out_inst_0,
    output reg [`INST_WIDTH-1:0] out_inst_1,
    output reg [`IF_BATCH_SIZE-1:0]   out_inst_valid, // mask to indicate which instruction is valid
    output reg                        out_pred_taken_0,
    output reg [`INST_ADDR_WIDTH-1:0] out_pred_target_0,
    output reg [`BP_GHR_BITS-1:0]     out_pred_hist_0,
    output reg                        out_pred_taken_1,
    output reg [`INST_ADDR_WIDTH-1:0] out_pred_target_1,
    output reg [`BP_GHR_BITS-1:0]     out_pred_hist_1
);
    reg [`INST_ADDR_WIDTH-1:0] reg_PC;
    reg                   first_clk_passed;  // reserved for memory related timing
    reg                   second_clk_passed; // reserved for memory related timing

    localparam [(`INST_ADDR_WIDTH-1):0] FETCH_STRIDE = `IF_BATCH_SIZE * `INST_ADD_STEP;

    wire bp_pred_taken;
    wire [`INST_ADDR_WIDTH-1:0] bp_pred_target;
    wire [`BP_GHR_BITS-1:0] bp_pred_hist;

    BranchPredictor u_bp (
        .clk(clk),
        .rst_n(rst_n),
        .flush(flush | redirect_valid),
        .fetch_valid(~stall),
        .fetch_pc(reg_PC),
        .pred_taken(bp_pred_taken),
        .pred_target(bp_pred_target),
        .pred_hist(bp_pred_hist),
        .update0_valid(bp_update0_valid),
        .update0_pc(bp_update0_pc),
        .update0_taken(bp_update0_taken),
        .update0_target(bp_update0_target),
        .update0_hist(bp_update0_hist),
        .update0_is_call(bp_update0_is_call),
        .update0_is_return(bp_update0_is_return),
        .update1_valid(bp_update1_valid),
        .update1_pc(bp_update1_pc),
        .update1_taken(bp_update1_taken),
        .update1_target(bp_update1_target),
        .update1_hist(bp_update1_hist),
        .update1_is_call(bp_update1_is_call),
        .update1_is_return(bp_update1_is_return)
    );

    wire [`INST_ADDR_WIDTH-1:0] seq_next_pc = reg_PC + FETCH_STRIDE;
    wire [`INST_ADDR_WIDTH-1:0] chosen_target = bp_pred_taken ? bp_pred_target : seq_next_pc;

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            reg_PC <= `INST_INIT;
            first_clk_passed <= 1'b0;
            second_clk_passed <= 1'b0;
            out_inst_addr_0 <= {`INST_WIDTH{1'b0}};
            out_inst_addr_1 <= {`INST_WIDTH{1'b0}};
            out_inst_0 <= {`INST_WIDTH{1'b0}};
            out_inst_1 <= {`INST_WIDTH{1'b0}};
            out_inst_valid  <= {`IF_BATCH_SIZE{1'b0}};
            out_pred_taken_0 <= 1'b0;
            out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        end else if (flush) begin
            reg_PC <= redirect_valid ? redirect_pc : `INST_INIT;
            first_clk_passed <= 1'b0;
            second_clk_passed <= 1'b0;
            out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
            out_pred_taken_0 <= 1'b0;
            out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        end else if (stall) begin
            out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
        end else if (redirect_valid) begin
            reg_PC <= redirect_pc;
            first_clk_passed <= 1'b0;
            second_clk_passed <= 1'b0;
            out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
            out_pred_taken_0 <= 1'b0;
            out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        end else begin
            reg_PC <= chosen_target;
            first_clk_passed <= 1'b1;
            second_clk_passed <= first_clk_passed;
            out_inst_addr_0 <= reg_PC;
            out_inst_addr_1 <= reg_PC + `INST_ADD_STEP;
            out_inst_0 <= 32'h0010_0093; // ADDI x1, x0, 1 (bring-up)
            out_inst_1 <= 32'h0000_0013;
            out_inst_valid  <= second_clk_passed ? (bp_pred_taken ? 2'b01 : {`IF_BATCH_SIZE{1'b1}}) : {`IF_BATCH_SIZE{1'b0}};
            out_pred_taken_0 <= bp_pred_taken & second_clk_passed;
            out_pred_target_0 <= bp_pred_target;
            out_pred_hist_0 <= bp_pred_hist;
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= reg_PC + `INST_ADD_STEP;
            out_pred_hist_1 <= bp_pred_hist;
        end
    end

endmodule
