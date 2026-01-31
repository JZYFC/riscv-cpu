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

    // I-Cache interface
    output wire                        ic_req,
    output wire [`INST_ADDR_WIDTH-1:0] ic_paddr,
    input  wire [127:0]                ic_rdata_line,
    input  wire                        ic_valid,
    input  wire                        ic_stall,

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
    reg                        line_valid;
    reg [`INST_ADDR_WIDTH-1:4] line_base;
    reg [127:0]                line_data;

    localparam [(`INST_ADDR_WIDTH-1):0] FETCH_STRIDE = `IF_BATCH_SIZE * `INST_ADD_STEP;

    wire bp_pred_taken;
    wire [`INST_ADDR_WIDTH-1:0] bp_pred_target;
    wire [`BP_GHR_BITS-1:0] bp_pred_hist;

    wire [1:0] word_idx = reg_PC[3:2];
    wire       cross_line = (word_idx == 2'b11);
    wire [`INST_ADDR_WIDTH-1:0] seq_next_pc = reg_PC + (cross_line ? `INST_ADD_STEP : FETCH_STRIDE);
    wire [`INST_ADDR_WIDTH-1:0] chosen_target = bp_pred_taken ? bp_pred_target : seq_next_pc;

    wire line_hit = line_valid && (line_base == reg_PC[31:4]);
    wire need_line = !line_hit;

    assign ic_req = (!stall) && (!flush) && (!redirect_valid) && need_line;
    assign ic_paddr = reg_PC;

    wire [127:0] line_data_eff = ic_valid ? ic_rdata_line : line_data;
    wire line_ready = line_hit || ic_valid;
    wire fetch_fire = (!stall) && (!ic_stall) && line_ready && (!flush) && (!redirect_valid);

    BranchPredictor u_bp (
        .clk(clk),
        .rst_n(rst_n),
        .flush(flush | redirect_valid),
        .fetch_valid(fetch_fire),
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

    function automatic [`INST_WIDTH-1:0] line_word;
        input [127:0] line;
        input [1:0] idx;
        begin
            case (idx)
                2'b00: line_word = line[31:0];
                2'b01: line_word = line[63:32];
                2'b10: line_word = line[95:64];
                2'b11: line_word = line[127:96];
            endcase
        end
    endfunction

    wire [`INST_WIDTH-1:0] inst0_word = line_word(line_data_eff, word_idx);
    wire [`INST_WIDTH-1:0] inst1_word = cross_line ? {`INST_WIDTH{1'b0}} : line_word(line_data_eff, word_idx + 2'd1);
    wire [`IF_BATCH_SIZE-1:0] fetch_valid_mask = bp_pred_taken ? 2'b01 :
                                                 (cross_line ? 2'b01 : {`IF_BATCH_SIZE{1'b1}});

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            reg_PC <= `INST_INIT;
            line_valid <= 1'b0;
            line_base <= {(`INST_ADDR_WIDTH-4){1'b0}};
            line_data <= 128'b0;
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
            line_valid <= 1'b0;
            out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
            out_pred_taken_0 <= 1'b0;
            out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        end else if (stall || ic_stall) begin
            out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
        end else if (redirect_valid) begin
            reg_PC <= redirect_pc;
            line_valid <= 1'b0;
            out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
            out_pred_taken_0 <= 1'b0;
            out_pred_target_0 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_0 <= {`BP_GHR_BITS{1'b0}};
            out_pred_taken_1 <= 1'b0;
            out_pred_target_1 <= {`INST_ADDR_WIDTH{1'b0}};
            out_pred_hist_1 <= {`BP_GHR_BITS{1'b0}};
        end else begin
            if (ic_valid) begin
                line_valid <= 1'b1;
                line_base  <= reg_PC[31:4];
                line_data  <= ic_rdata_line;
            end
            if (line_ready) begin
                reg_PC <= chosen_target;
                out_inst_addr_0 <= reg_PC;
                out_inst_addr_1 <= reg_PC + `INST_ADD_STEP;
                out_inst_0 <= inst0_word;
                out_inst_1 <= inst1_word;
                out_inst_valid  <= fetch_valid_mask;
                out_pred_taken_0 <= bp_pred_taken;
                out_pred_target_0 <= bp_pred_target;
                out_pred_hist_0 <= bp_pred_hist;
                out_pred_taken_1 <= 1'b0;
                out_pred_target_1 <= reg_PC + `INST_ADD_STEP;
                out_pred_hist_1 <= bp_pred_hist;
            end else begin
                out_inst_valid <= {`IF_BATCH_SIZE{1'b0}};
            end
        end
    end

endmodule
