`include "riscv_define.v"

// Reservation-station based issue/execute (dual-issue).
// Supports INT, MULDIV, BRANCH/JAL/JALR, SYSTEM/CSR. LSU/FP remain stubs.
module IssueBuffer #(
    parameter RS_DEPTH = 8
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         flush,
    input  wire                         flush_is_redirect,
    input  wire [`ROB_IDX_WIDTH-1:0]    flush_rob_idx,

    // Incoming micro-ops from PostDecode
    input  wire [`IF_BATCH_SIZE-1:0]    in_inst_valid,
    input  wire [`INST_WIDTH-1:0]       in_inst_0,
    input  wire [`INST_WIDTH-1:0]       in_inst_1,
    input  wire [`INST_ADDR_WIDTH-1:0]  in_pc_0,
    input  wire [`INST_ADDR_WIDTH-1:0]  in_pc_1,
    input  wire                         in_pred_taken_0,
    input  wire [`INST_ADDR_WIDTH-1:0]  in_pred_target_0,
    input  wire [`BP_GHR_BITS-1:0]      in_pred_hist_0,
    input  wire                         in_pred_taken_1,
    input  wire [`INST_ADDR_WIDTH-1:0]  in_pred_target_1,
    input  wire [`BP_GHR_BITS-1:0]      in_pred_hist_1,
    input  wire [`REG_ADDR_WIDTH-1:0]   in_rs1_0,
    input  wire [`REG_ADDR_WIDTH-1:0]   in_rs2_0,
    input  wire [`REG_ADDR_WIDTH-1:0]   in_rd_0,
    input  wire [`DATA_WIDTH-1:0]       in_imm_0,
    input  wire                         in_use_imm_0,
    input  wire                         in_rs1_is_fp_0,
    input  wire                         in_rs2_is_fp_0,
    input  wire                         in_rd_is_fp_0,
    input  wire [`ROB_IDX_WIDTH-1:0]    in_rob_idx_0,
    input  wire                         in_rob_idx_valid_0,
    input  wire [`ROB_GEN_WIDTH-1:0]    in_rob_gen_0,
    input  wire [`PREG_IDX_WIDTH-1:0]   in_rs1_preg_0,
    input  wire                         in_rs1_preg_valid_0,
    input  wire                         in_rs1_ready_now_0,
    input  wire [`PREG_IDX_WIDTH-1:0]   in_rs2_preg_0,
    input  wire                         in_rs2_preg_valid_0,
    input  wire                         in_rs2_ready_now_0,
    input  wire [`PREG_IDX_WIDTH-1:0]   in_rd_preg_0,
    input  wire                         in_rd_preg_valid_0,
    input  wire [`FU_DEC_WIDTH-1:0]     in_fu_sel_0,
    input  wire [`ALU_OP_WIDTH-1:0]     in_int_op_0,
    input  wire                         in_int_is_sub_0,
    input  wire                         in_cmp_signed_0,
    input  wire [`ALU_OP_WIDTH-1:0]     in_muldiv_op_0,
    input  wire                         in_mul_high_0,
    input  wire                         in_mul_signed_rs1_0,
    input  wire                         in_mul_signed_rs2_0,
    input  wire                         in_div_signed_0,
    input  wire                         in_div_is_rem_0,
    input  wire [`BR_OP_WIDTH-1:0]      in_branch_op_0,
    input  wire [`MEM_OP_WIDTH-1:0]     in_mem_op_0,
    input  wire                         in_mem_is_load_0,
    input  wire                         in_mem_unsigned_0,
    input  wire [`CSR_OP_WIDTH-1:0]     in_csr_op_0,
    input  wire [11:0]                  in_csr_addr_0,
    input  wire [`FP_OP_WIDTH-1:0]      in_fp_op_0,
    input  wire                         in_illegal_0,

    input  wire [`REG_ADDR_WIDTH-1:0]   in_rs1_1,
    input  wire [`REG_ADDR_WIDTH-1:0]   in_rs2_1,
    input  wire [`REG_ADDR_WIDTH-1:0]   in_rd_1,
    input  wire [`DATA_WIDTH-1:0]       in_imm_1,
    input  wire                         in_use_imm_1,
    input  wire                         in_rs1_is_fp_1,
    input  wire                         in_rs2_is_fp_1,
    input  wire                         in_rd_is_fp_1,
    input  wire [`ROB_IDX_WIDTH-1:0]    in_rob_idx_1,
    input  wire                         in_rob_idx_valid_1,
    input  wire [`ROB_GEN_WIDTH-1:0]    in_rob_gen_1,
    input  wire [`PREG_IDX_WIDTH-1:0]   in_rs1_preg_1,
    input  wire                         in_rs1_preg_valid_1,
    input  wire                         in_rs1_ready_now_1,
    input  wire [`PREG_IDX_WIDTH-1:0]   in_rs2_preg_1,
    input  wire                         in_rs2_preg_valid_1,
    input  wire                         in_rs2_ready_now_1,
    input  wire [`PREG_IDX_WIDTH-1:0]   in_rd_preg_1,
    input  wire                         in_rd_preg_valid_1,
    input  wire [`FU_DEC_WIDTH-1:0]     in_fu_sel_1,
    input  wire [`ALU_OP_WIDTH-1:0]     in_int_op_1,
    input  wire                         in_int_is_sub_1,
    input  wire                         in_cmp_signed_1,
    input  wire [`ALU_OP_WIDTH-1:0]     in_muldiv_op_1,
    input  wire                         in_mul_high_1,
    input  wire                         in_mul_signed_rs1_1,
    input  wire                         in_mul_signed_rs2_1,
    input  wire                         in_div_signed_1,
    input  wire                         in_div_is_rem_1,
    input  wire [`BR_OP_WIDTH-1:0]      in_branch_op_1,
    input  wire [`MEM_OP_WIDTH-1:0]     in_mem_op_1,
    input  wire                         in_mem_is_load_1,
    input  wire                         in_mem_unsigned_1,
    input  wire [`CSR_OP_WIDTH-1:0]     in_csr_op_1,
    input  wire [11:0]                  in_csr_addr_1,
    input  wire [`FP_OP_WIDTH-1:0]      in_fp_op_1,
    input  wire                         in_illegal_1,

    // Backpressure to dispatch
    output wire                         stall_dispatch,

    // ROB head (for non-speculative store issue)
    input  wire [`ROB_IDX_WIDTH-1:0]    rob_head,

    // Writeback outputs to ROB/PRF
    output reg                          wb0_valid,
    output reg  [`ROB_IDX_WIDTH-1:0]    wb0_rob_idx,
    output reg  [`ROB_GEN_WIDTH-1:0]    wb0_rob_gen,
    output reg  [`DATA_WIDTH-1:0]       wb0_value,
    output reg                          wb0_exception,
    output reg                          wb1_valid,
    output reg  [`ROB_IDX_WIDTH-1:0]    wb1_rob_idx,
    output reg  [`ROB_GEN_WIDTH-1:0]    wb1_rob_gen,
    output reg  [`DATA_WIDTH-1:0]       wb1_value,
    output reg                          wb1_exception,
    output reg                          wb2_valid,
    output reg  [`ROB_IDX_WIDTH-1:0]    wb2_rob_idx,
    output reg  [`ROB_GEN_WIDTH-1:0]    wb2_rob_gen,
    output reg  [`DATA_WIDTH-1:0]       wb2_value,
    output reg                          wb2_exception,

    // LSU interface (single outstanding, ordered)
    output reg                          lsu_valid,
    output reg  [31:0]                  lsu_addr,
    output reg  [31:0]                  lsu_wdata,
    output reg  [`MEM_OP_WIDTH-1:0]     lsu_mem_op,
    output reg                          lsu_mem_is_load,
    output reg                          lsu_mem_unsigned,
    output reg  [`ROB_IDX_WIDTH-1:0]    lsu_rob_idx,
    output reg  [`ROB_GEN_WIDTH-1:0]    lsu_rob_gen,
    output reg  [`PREG_IDX_WIDTH-1:0]   lsu_rd_tag,
    output reg                          lsu_rd_is_fp,
    input  wire                         lsu_busy,
    input  wire                         lsu_wb_valid,
    input  wire [`DATA_WIDTH-1:0]       lsu_wb_value,
    input  wire [`ROB_IDX_WIDTH-1:0]    lsu_wb_rob_idx,
    input  wire [`ROB_GEN_WIDTH-1:0]    lsu_wb_rob_gen,
    input  wire [`PREG_IDX_WIDTH-1:0]   lsu_wb_dest_tag,
    input  wire                         lsu_wb_dest_is_fp,
    input  wire                         lsu_wb_exception,

    // Branch redirect
    output reg                          redirect_valid,
    output reg  [`INST_ADDR_WIDTH-1:0]  redirect_target,
    output reg  [`ROB_IDX_WIDTH-1:0]    redirect_rob_idx,

    // Predictor updates
    output reg                          bp_update0_valid,
    output reg  [`INST_ADDR_WIDTH-1:0]  bp_update0_pc,
    output reg                          bp_update0_taken,
    output reg  [`INST_ADDR_WIDTH-1:0]  bp_update0_target,
    output reg  [`BP_GHR_BITS-1:0]      bp_update0_hist,
    output reg                          bp_update0_is_call,
    output reg                          bp_update0_is_return,
    output reg                          bp_update1_valid,
    output reg  [`INST_ADDR_WIDTH-1:0]  bp_update1_pc,
    output reg                          bp_update1_taken,
    output reg  [`INST_ADDR_WIDTH-1:0]  bp_update1_target,
    output reg  [`BP_GHR_BITS-1:0]      bp_update1_hist,
    output reg                          bp_update1_is_call,
    output reg                          bp_update1_is_return,

    // PRF interface (shared)
    input  wire [`DATA_WIDTH-1:0]       int_rs1_data0,
    input  wire [`DATA_WIDTH-1:0]       int_rs2_data0,
    input  wire [`DATA_WIDTH-1:0]       int_rs1_data1,
    input  wire [`DATA_WIDTH-1:0]       int_rs2_data1,
    input  wire [`DATA_WIDTH-1:0]       fp_rs1_data0,
    input  wire [`DATA_WIDTH-1:0]       fp_rs2_data0,
    input  wire [`DATA_WIDTH-1:0]       fp_rs1_data1,
    input  wire [`DATA_WIDTH-1:0]       fp_rs2_data1,
    output reg  [`PREG_IDX_WIDTH-1:0]   i_wr_addr0,
    output reg  [`DATA_WIDTH-1:0]       i_wr_data0,
    output reg                          i_wr_we0,
    output reg  [`PREG_IDX_WIDTH-1:0]   i_wr_addr1,
    output reg  [`DATA_WIDTH-1:0]       i_wr_data1,
    output reg                          i_wr_we1,
    output reg  [`PREG_IDX_WIDTH-1:0]   f_wr_addr0,
    output reg  [`DATA_WIDTH-1:0]       f_wr_data0,
    output reg                          f_wr_we0,
    output reg  [`PREG_IDX_WIDTH-1:0]   f_wr_addr1,
    output reg  [`DATA_WIDTH-1:0]       f_wr_data1,
    output reg                          f_wr_we1
);

    localparam AGE_W = 8;

    // RS storage (parallel arrays)
    reg                        rs_valid      [0:RS_DEPTH-1];
    reg [AGE_W-1:0]            rs_age        [0:RS_DEPTH-1];
    reg [`FU_DEC_WIDTH-1:0]    rs_fu_sel     [0:RS_DEPTH-1];
    reg [`ALU_OP_WIDTH-1:0]    rs_int_op     [0:RS_DEPTH-1];
    reg                        rs_int_sub    [0:RS_DEPTH-1];
    reg                        rs_cmp_signed [0:RS_DEPTH-1];
    reg [`ALU_OP_WIDTH-1:0]    rs_muldiv_op  [0:RS_DEPTH-1];
    reg                        rs_mul_high   [0:RS_DEPTH-1];
    reg                        rs_mul_signed_rs1 [0:RS_DEPTH-1];
    reg                        rs_mul_signed_rs2 [0:RS_DEPTH-1];
    reg                        rs_div_signed [0:RS_DEPTH-1];
    reg                        rs_div_is_rem [0:RS_DEPTH-1];
    reg [`BR_OP_WIDTH-1:0]     rs_branch_op  [0:RS_DEPTH-1];
    reg [`MEM_OP_WIDTH-1:0]    rs_mem_op     [0:RS_DEPTH-1];
    reg                        rs_mem_is_load[0:RS_DEPTH-1];
    reg                        rs_mem_unsigned[0:RS_DEPTH-1];
    reg [`CSR_OP_WIDTH-1:0]    rs_csr_op     [0:RS_DEPTH-1];
    reg [11:0]                 rs_csr_addr   [0:RS_DEPTH-1];
    reg [`FP_OP_WIDTH-1:0]     rs_fp_op      [0:RS_DEPTH-1];
    reg                        rs_illegal    [0:RS_DEPTH-1];
    reg [`INST_WIDTH-1:0]      rs_inst       [0:RS_DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] rs_pc         [0:RS_DEPTH-1];
    reg [`DATA_WIDTH-1:0]      rs_imm        [0:RS_DEPTH-1];
    reg                        rs_use_imm    [0:RS_DEPTH-1];
    reg                        rs_rs1_ready  [0:RS_DEPTH-1];
    reg                        rs_rs2_ready  [0:RS_DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  rs_rs1_tag    [0:RS_DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  rs_rs2_tag    [0:RS_DEPTH-1];
    reg [`DATA_WIDTH-1:0]      rs_rs1_val    [0:RS_DEPTH-1];
    reg [`DATA_WIDTH-1:0]      rs_rs2_val    [0:RS_DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  rs_rd_tag     [0:RS_DEPTH-1];
    reg                        rs_rd_is_fp   [0:RS_DEPTH-1];
    reg [`ROB_IDX_WIDTH-1:0]   rs_rob_idx    [0:RS_DEPTH-1];
    reg [`ROB_GEN_WIDTH-1:0]   rs_rob_gen    [0:RS_DEPTH-1];
    reg                        rs_pred_taken [0:RS_DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] rs_pred_target[0:RS_DEPTH-1];
    reg [`BP_GHR_BITS-1:0]     rs_pred_hist  [0:RS_DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  rs_arch_rd    [0:RS_DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  rs_arch_rs1   [0:RS_DEPTH-1];

    reg [RS_DEPTH:0] rs_count;
    reg [AGE_W-1:0] age_counter;
    integer issue_idx0, issue_idx1;

    reg [`DATA_WIDTH-1:0] csr_file [0:4095];

    wire [2:0] enq_need = {2'b0, in_inst_valid[0]} + {2'b0, in_inst_valid[1]};
    reg [RS_DEPTH:0] rs_valid_count;
    integer count_i;
    always @(*) begin
        rs_valid_count = 0;
        for (count_i = 0; count_i < RS_DEPTH; count_i = count_i + 1) begin
            if (rs_valid[count_i]) rs_valid_count = rs_valid_count + 1'b1;
        end
    end
    wire [RS_DEPTH:0] free_slots = RS_DEPTH - rs_valid_count;
    assign stall_dispatch = (enq_need > free_slots);

    // CDB
    reg                       cdb0_valid, cdb1_valid;
    reg [`PREG_IDX_WIDTH-1:0] cdb0_tag, cdb1_tag;
    reg [`DATA_WIDTH-1:0]     cdb0_value, cdb1_value;

    // Bypass helpers for newly-enqueued ops: see whether CDB already has the value
    wire rs1_cdb0_hit_0 = cdb0_valid && (in_rs1_preg_0 == cdb0_tag);
    wire rs1_cdb1_hit_0 = cdb1_valid && (in_rs1_preg_0 == cdb1_tag);
    wire rs2_cdb0_hit_0 = cdb0_valid && (in_rs2_preg_0 == cdb0_tag);
    wire rs2_cdb1_hit_0 = cdb1_valid && (in_rs2_preg_0 == cdb1_tag);
    wire rs1_cdb0_hit_1 = cdb0_valid && (in_rs1_preg_1 == cdb0_tag);
    wire rs1_cdb1_hit_1 = cdb1_valid && (in_rs1_preg_1 == cdb1_tag);
    wire rs2_cdb0_hit_1 = cdb0_valid && (in_rs2_preg_1 == cdb0_tag);
    wire rs2_cdb1_hit_1 = cdb1_valid && (in_rs2_preg_1 == cdb1_tag);

    wire rs1_is_x0_0 = (!in_rs1_is_fp_0) && (in_rs1_0 == {`REG_ADDR_WIDTH{1'b0}});
    wire rs2_is_x0_0 = (!in_rs2_is_fp_0) && (in_rs2_0 == {`REG_ADDR_WIDTH{1'b0}});
    wire rs1_is_x0_1 = (!in_rs1_is_fp_1) && (in_rs1_1 == {`REG_ADDR_WIDTH{1'b0}});
    wire rs2_is_x0_1 = (!in_rs2_is_fp_1) && (in_rs2_1 == {`REG_ADDR_WIDTH{1'b0}});
    wire rs1_ready_now_0 = rs1_is_x0_0 || in_rs1_ready_now_0 || rs1_cdb0_hit_0 || rs1_cdb1_hit_0;
    wire rs2_ready_now_0 = rs2_is_x0_0 || in_rs2_ready_now_0 || rs2_cdb0_hit_0 || rs2_cdb1_hit_0;
    wire rs1_ready_now_1 = rs1_is_x0_1 || in_rs1_ready_now_1 || rs1_cdb0_hit_1 || rs1_cdb1_hit_1;
    wire rs2_ready_now_1 = rs2_is_x0_1 || in_rs2_ready_now_1 || rs2_cdb0_hit_1 || rs2_cdb1_hit_1;

    // Branch resolve signals (combinational for current issue selection)
    reg br0_taken, br1_taken;
    reg [`INST_ADDR_WIDTH-1:0] br0_target, br1_target;
    wire br0_is_branch = (issue_idx0 != -1) && rs_valid[issue_idx0] && (rs_fu_sel[issue_idx0] == `FU_DEC_BRANCH);
    wire br1_is_branch = (issue_idx1 != -1) && rs_valid[issue_idx1] && (rs_fu_sel[issue_idx1] == `FU_DEC_BRANCH);
    wire [`INST_ADDR_WIDTH-1:0] br0_seq_next = br0_is_branch ? (rs_pc[issue_idx0] + `INST_ADD_STEP) : {`INST_ADDR_WIDTH{1'b0}};
    wire [`INST_ADDR_WIDTH-1:0] br1_seq_next = br1_is_branch ? (rs_pc[issue_idx1] + `INST_ADD_STEP) : {`INST_ADDR_WIDTH{1'b0}};
    always @(*) begin
        br0_taken  = 1'b0; br1_taken  = 1'b0;
        br0_target = {`INST_ADDR_WIDTH{1'b0}};
        br1_target = {`INST_ADDR_WIDTH{1'b0}};
        if (br0_is_branch) begin
            case (rs_branch_op[issue_idx0])
                `BR_OP_BEQ:  br0_taken = (rs_rs1_val[issue_idx0] == rs_rs2_val[issue_idx0]);
                `BR_OP_BNE:  br0_taken = (rs_rs1_val[issue_idx0] != rs_rs2_val[issue_idx0]);
                `BR_OP_BLT:  br0_taken = ($signed(rs_rs1_val[issue_idx0]) < $signed(rs_rs2_val[issue_idx0]));
                `BR_OP_BGE:  br0_taken = ($signed(rs_rs1_val[issue_idx0]) >= $signed(rs_rs2_val[issue_idx0]));
                `BR_OP_BLTU: br0_taken = ($unsigned(rs_rs1_val[issue_idx0]) < $unsigned(rs_rs2_val[issue_idx0]));
                `BR_OP_BGEU: br0_taken = ($unsigned(rs_rs1_val[issue_idx0]) >= $unsigned(rs_rs2_val[issue_idx0]));
                `BR_OP_JAL,
                `BR_OP_JALR: br0_taken = 1'b1;
                default:     br0_taken = 1'b0;
            endcase
            if (rs_branch_op[issue_idx0] == `BR_OP_JALR) begin
                br0_target = (rs_rs1_val[issue_idx0] + rs_imm[issue_idx0]) & {{(`INST_ADDR_WIDTH-1){1'b1}},1'b0};
            end else begin
                br0_target = rs_pc[issue_idx0] + rs_imm[issue_idx0];
            end
        end
        if (br1_is_branch) begin
            case (rs_branch_op[issue_idx1])
                `BR_OP_BEQ:  br1_taken = (rs_rs1_val[issue_idx1] == rs_rs2_val[issue_idx1]);
                `BR_OP_BNE:  br1_taken = (rs_rs1_val[issue_idx1] != rs_rs2_val[issue_idx1]);
                `BR_OP_BLT:  br1_taken = ($signed(rs_rs1_val[issue_idx1]) < $signed(rs_rs2_val[issue_idx1]));
                `BR_OP_BGE:  br1_taken = ($signed(rs_rs1_val[issue_idx1]) >= $signed(rs_rs2_val[issue_idx1]));
                `BR_OP_BLTU: br1_taken = ($unsigned(rs_rs1_val[issue_idx1]) < $unsigned(rs_rs2_val[issue_idx1]));
                `BR_OP_BGEU: br1_taken = ($unsigned(rs_rs1_val[issue_idx1]) >= $unsigned(rs_rs2_val[issue_idx1]));
                `BR_OP_JAL,
                `BR_OP_JALR: br1_taken = 1'b1;
                default:     br1_taken = 1'b0;
            endcase
            if (rs_branch_op[issue_idx1] == `BR_OP_JALR) begin
                br1_target = (rs_rs1_val[issue_idx1] + rs_imm[issue_idx1]) & {{(`INST_ADDR_WIDTH-1){1'b1}},1'b0};
            end else begin
                br1_target = rs_pc[issue_idx1] + rs_imm[issue_idx1];
            end
        end
    end
    wire br0_mispredict = br0_is_branch && ((br0_taken != rs_pred_taken[issue_idx0]) ||
                          (br0_taken && rs_pred_taken[issue_idx0] && (br0_target != rs_pred_target[issue_idx0])));
    wire br1_mispredict = br1_is_branch && ((br1_taken != rs_pred_taken[issue_idx1]) ||
                          (br1_taken && rs_pred_taken[issue_idx1] && (br1_target != rs_pred_target[issue_idx1])));
    wire br0_is_call   = br0_is_branch && ((rs_branch_op[issue_idx0]==`BR_OP_JAL) || (rs_branch_op[issue_idx0]==`BR_OP_JALR)) &&
                         (rs_arch_rd[issue_idx0] == 5'd1);
    wire br1_is_call   = br1_is_branch && ((rs_branch_op[issue_idx1]==`BR_OP_JAL) || (rs_branch_op[issue_idx1]==`BR_OP_JALR)) &&
                         (rs_arch_rd[issue_idx1] == 5'd1);
    wire br0_is_return = br0_is_branch && (rs_branch_op[issue_idx0]==`BR_OP_JALR) &&
                         (rs_arch_rs1[issue_idx0] == 5'd1) && (rs_arch_rd[issue_idx0] == {`REG_ADDR_WIDTH{1'b0}});
    wire br1_is_return = br1_is_branch && (rs_branch_op[issue_idx1]==`BR_OP_JALR) &&
                         (rs_arch_rs1[issue_idx1] == 5'd1) && (rs_arch_rd[issue_idx1] == {`REG_ADDR_WIDTH{1'b0}});

    // FP execute (combinational, 2-source only; no rs3 path yet).
    wire fp_issue0 = (issue_idx0 != -1) && rs_valid[issue_idx0] && (rs_fu_sel[issue_idx0] == `FU_DEC_FP);
    wire fp_issue1 = (issue_idx1 != -1) && rs_valid[issue_idx1] && (rs_fu_sel[issue_idx1] == `FU_DEC_FP);
    wire [`DATA_WIDTH-1:0] fpu_a0 = fp_issue0 ? rs_rs1_val[issue_idx0] : {`DATA_WIDTH{1'b0}};
    wire [`DATA_WIDTH-1:0] fpu_b0 = fp_issue0 ? rs_rs2_val[issue_idx0] : {`DATA_WIDTH{1'b0}};
    wire [`DATA_WIDTH-1:0] fpu_a1 = fp_issue1 ? rs_rs1_val[issue_idx1] : {`DATA_WIDTH{1'b0}};
    wire [`DATA_WIDTH-1:0] fpu_b1 = fp_issue1 ? rs_rs2_val[issue_idx1] : {`DATA_WIDTH{1'b0}};
    wire [`FP_OP_WIDTH-1:0] fpu_op0 = fp_issue0 ? rs_fp_op[issue_idx0] : `FP_OP_DUMMY;
    wire [`FP_OP_WIDTH-1:0] fpu_op1 = fp_issue1 ? rs_fp_op[issue_idx1] : `FP_OP_DUMMY;
    wire [`ROUNDING_MODE_WIDTH-1:0] fpu_rm = `ROUNDING_MODE_RNE;
    wire [`DATA_WIDTH-1:0] fpu_res0;
    wire [`DATA_WIDTH-1:0] fpu_res1;
    wire fpu_exc0;
    wire fpu_exc1;

    ibex_fpu u_fpu0 (
        .A_i(fpu_a0),
        .B_i(fpu_b0),
        .C_i({`DATA_WIDTH{1'b0}}),
        .opcode_i(fpu_op0),
        .rounding_mode_i(fpu_rm),
        .result_o(fpu_res0),
        .exception_flag_o(fpu_exc0)
    );

    ibex_fpu u_fpu1 (
        .A_i(fpu_a1),
        .B_i(fpu_b1),
        .C_i({`DATA_WIDTH{1'b0}}),
        .opcode_i(fpu_op1),
        .rounding_mode_i(fpu_rm),
        .result_o(fpu_res1),
        .exception_flag_o(fpu_exc1)
    );

    // Candidate generation (issue0, issue1, divider)
    wire cand0_has_dest = (rs_rd_tag[issue_idx0] != {`PREG_IDX_WIDTH{1'b0}});
    wire cand0_valid = (issue_idx0 != -1) && rs_valid[issue_idx0] &&
                       (rs_fu_sel[issue_idx0] != `FU_DEC_LSU) &&
                       !(rs_fu_sel[issue_idx0]==`FU_DEC_MULDIV && rs_muldiv_op[issue_idx0][`ALU_OP_DIV]);
    wire [`DATA_WIDTH-1:0] cand0_val = select_result(rs_fu_sel[issue_idx0], alu_res_0, mul_res0, fpu_res0, rs_muldiv_op[issue_idx0], csr_old0, link0);
    wire [`PREG_IDX_WIDTH-1:0] cand0_tag = rs_rd_tag[issue_idx0];
    wire [`ROB_IDX_WIDTH-1:0]  cand0_rob = rs_rob_idx[issue_idx0];
    wire [`ROB_GEN_WIDTH-1:0]  cand0_rob_gen = rs_rob_gen[issue_idx0];
    wire cand0_is_fp = rs_rd_is_fp[issue_idx0];
    wire cand0_exc   = rs_illegal[issue_idx0];

    wire cand1_has_dest = (rs_rd_tag[issue_idx1] != {`PREG_IDX_WIDTH{1'b0}});
    wire cand1_valid = (issue_idx1 != -1) && rs_valid[issue_idx1] &&
                       (rs_fu_sel[issue_idx1] != `FU_DEC_LSU) &&
                       !(rs_fu_sel[issue_idx1]==`FU_DEC_MULDIV && rs_muldiv_op[issue_idx1][`ALU_OP_DIV]);
    wire [`DATA_WIDTH-1:0] cand1_val = select_result(rs_fu_sel[issue_idx1], alu_res_1, mul_res1, fpu_res1, rs_muldiv_op[issue_idx1], csr_old1, link1);
    wire [`PREG_IDX_WIDTH-1:0] cand1_tag = rs_rd_tag[issue_idx1];
    wire [`ROB_IDX_WIDTH-1:0]  cand1_rob = rs_rob_idx[issue_idx1];
    wire [`ROB_GEN_WIDTH-1:0]  cand1_rob_gen = rs_rob_gen[issue_idx1];
    wire cand1_is_fp = rs_rd_is_fp[issue_idx1];
    wire cand1_exc   = rs_illegal[issue_idx1];

    reg [`PREG_IDX_WIDTH-1:0] wb0_dest_tag, wb1_dest_tag, div_dest_tag;
    reg wb0_dest_valid, wb1_dest_valid, div_dest_valid;
    reg wb0_dest_is_fp, wb1_dest_is_fp, div_dest_is_fp;
    reg [`ROB_IDX_WIDTH-1:0] div_rob_idx;
    reg [`ROB_GEN_WIDTH-1:0] div_rob_gen;
    reg                      div_exception;
    reg                      div_busy;
    reg [3:0]                div_counter;
    reg [`DATA_WIDTH-1:0]    div_value;
    reg                      div_done;
    integer pick;

    wire cand2_valid = div_done && div_dest_valid;
    wire [`DATA_WIDTH-1:0] cand2_val = div_value;
    wire [`PREG_IDX_WIDTH-1:0] cand2_tag = div_dest_tag;
    wire [`ROB_IDX_WIDTH-1:0]  cand2_rob = div_rob_idx;
    wire [`ROB_GEN_WIDTH-1:0]  cand2_rob_gen = div_rob_gen;
    wire cand2_is_fp = div_dest_is_fp;
    wire cand2_exc   = div_exception;

    wire cand3_has_dest = (lsu_wb_dest_tag != {`PREG_IDX_WIDTH{1'b0}});
    wire cand3_valid = lsu_wb_valid && cand3_has_dest;
    wire [`DATA_WIDTH-1:0]    cand3_val = lsu_wb_value;
    wire [`PREG_IDX_WIDTH-1:0] cand3_tag = lsu_wb_dest_tag;
    wire [`ROB_IDX_WIDTH-1:0]  cand3_rob = lsu_wb_rob_idx;
    wire [`ROB_GEN_WIDTH-1:0]  cand3_rob_gen = lsu_wb_rob_gen;
    wire cand3_is_fp = lsu_wb_dest_is_fp;
    wire cand3_exc   = lsu_wb_exception;
    wire lsu_nodest_valid = lsu_wb_valid && !cand3_has_dest;

    // Ready mask and selection (LSU strictly ordered, single outstanding)
    reg [RS_DEPTH-1:0] base_ready_mask;
    reg [RS_DEPTH-1:0] issue_ready_mask;
    integer rm;
    integer best_age0, best_age1;
    integer mem_issue_idx;
    integer mem_best_age;
    integer rob_dist;
    reg     mem_issue_ready;
    reg     mem_issue_is_store;
    reg     lsu_pending;
    reg     lsu_can_issue;
    wire allow_issue1 = !lsu_wb_valid;
    wire [`ROB_IDX_WIDTH-1:0] flush_rob_idx_next =
        (flush_rob_idx == (`ROB_SIZE-1)) ? {`ROB_IDX_WIDTH{1'b0}} : (flush_rob_idx + 1'b1);
    // Redirect flush is applied one cycle after redirect detection.
    // If the branch committed in-between, rob_head has already moved past flush_rob_idx.
    // In that case, no in-flight older/equal uops remain and we must not use wrapped ranges.
    wire redirect_keep_none = flush_is_redirect && (rob_head == flush_rob_idx_next);

    function in_range_inclusive;
        input [`ROB_IDX_WIDTH-1:0] idx;
        input [`ROB_IDX_WIDTH-1:0] start;
        input [`ROB_IDX_WIDTH-1:0] var_end;
        begin
            if (start <= var_end) in_range_inclusive = (idx >= start) && (idx <= var_end);
            else in_range_inclusive = (idx >= start) || (idx <= var_end);
        end
    endfunction
    always @(*) begin
        base_ready_mask = {RS_DEPTH{1'b0}};
        for (rm = 0; rm < RS_DEPTH; rm = rm + 1) begin
            base_ready_mask[rm] = rs_valid[rm] && rs_rs1_ready[rm] && rs_rs2_ready[rm];
        end

        // Find oldest LSU op using ROB order (avoid age counter wrap issues)
        mem_issue_idx = -1; mem_best_age = 0;
        for (rm = 0; rm < RS_DEPTH; rm = rm + 1) begin
            if (rs_valid[rm] && (rs_fu_sel[rm] == `FU_DEC_LSU)) begin
                if (rs_rob_idx[rm] >= rob_head) rob_dist = rs_rob_idx[rm] - rob_head;
                else rob_dist = rs_rob_idx[rm] + `ROB_SIZE - rob_head;
                if (mem_issue_idx == -1 || rob_dist < mem_best_age) begin
                    mem_issue_idx = rm;
                    mem_best_age = rob_dist;
                end
            end
        end
        mem_issue_ready = (mem_issue_idx != -1) && rs_rs1_ready[mem_issue_idx] && rs_rs2_ready[mem_issue_idx];
        mem_issue_is_store = (mem_issue_idx != -1) && !rs_mem_is_load[mem_issue_idx];
        // lsu_busy is sampled a cycle late by the IssueBuffer (registered interface),
        // so block a new request while a request is pending acceptance.
        lsu_can_issue = mem_issue_ready && !lsu_busy && !lsu_pending;
        // Stores are only allowed to issue when they reach ROB head (avoid wrong-path stores)
        if (mem_issue_is_store) begin
            lsu_can_issue = lsu_can_issue && (rs_rob_idx[mem_issue_idx] == rob_head);
        end

        issue_ready_mask = {RS_DEPTH{1'b0}};
        for (rm = 0; rm < RS_DEPTH; rm = rm + 1) begin
            issue_ready_mask[rm] = base_ready_mask[rm] &&
                                   ((rs_fu_sel[rm] != `FU_DEC_LSU) || (lsu_can_issue && (rm == mem_issue_idx))) &&
                                   ((rs_fu_sel[rm] != `FU_DEC_BRANCH) || (rs_rob_idx[rm] == rob_head));
        end

        issue_idx0 = -1; best_age0 = 0;
        for (rm = 0; rm < RS_DEPTH; rm = rm + 1) begin
            if (issue_ready_mask[rm]) begin
                if (issue_idx0 == -1 || rs_age[rm] < best_age0) begin
                    issue_idx0 = rm;
                    best_age0 = rs_age[rm];
                end
            end
        end
        issue_idx1 = -1; best_age1 = 0;
        if (allow_issue1 && !((issue_idx0 != -1) && (rs_fu_sel[issue_idx0] == `FU_DEC_BRANCH))) begin
            for (rm = 0; rm < RS_DEPTH; rm = rm + 1) begin
                if (issue_ready_mask[rm] && rm != issue_idx0) begin
                    if (issue_idx1 == -1 || rs_age[rm] < best_age1) begin
                        issue_idx1 = rm;
                        best_age1 = rs_age[rm];
                    end
                end
            end
        end
    end

    // Operand paths
    wire [`DATA_WIDTH-1:0] opA_0 = (issue_idx0 != -1 && rs_inst[issue_idx0][6:0] == 7'b0010111) ? rs_pc[issue_idx0] : (issue_idx0 != -1 ? rs_rs1_val[issue_idx0] : {`DATA_WIDTH{1'b0}});
    wire [`DATA_WIDTH-1:0] opA_1 = (issue_idx1 != -1 && rs_inst[issue_idx1][6:0] == 7'b0010111) ? rs_pc[issue_idx1] : (issue_idx1 != -1 ? rs_rs1_val[issue_idx1] : {`DATA_WIDTH{1'b0}});
    wire [`DATA_WIDTH-1:0] opB_0 = (issue_idx0 != -1 && rs_use_imm[issue_idx0]) ? rs_imm[issue_idx0] : (issue_idx0 != -1 ? rs_rs2_val[issue_idx0] : {`DATA_WIDTH{1'b0}});
    wire [`DATA_WIDTH-1:0] opB_1 = (issue_idx1 != -1 && rs_use_imm[issue_idx1]) ? rs_imm[issue_idx1] : (issue_idx1 != -1 ? rs_rs2_val[issue_idx1] : {`DATA_WIDTH{1'b0}});
    wire [`DATA_WIDTH-1:0] opB_eff_0 = (issue_idx0 != -1 && rs_int_sub[issue_idx0]) ? ~opB_0 : opB_0;
    wire [`DATA_WIDTH-1:0] opB_eff_1 = (issue_idx1 != -1 && rs_int_sub[issue_idx1]) ? ~opB_1 : opB_1;
    wire add_cin_0 = (issue_idx0 != -1) && rs_int_sub[issue_idx0];
    wire add_cin_1 = (issue_idx1 != -1) && rs_int_sub[issue_idx1];

    wire [`DATA_WIDTH-1:0] alu_res_0, alu_res_1;
    IntAlu u_alu0 (.clk(clk), .rst_n(rst_n), .ALU_op(issue_idx0!=-1 ? rs_int_op[issue_idx0] : {`ALU_OP_WIDTH{1'b0}}), .rs1(opA_0), .rs2(opB_eff_0), .add_c_in(add_cin_0), .cmp_signed(issue_idx0!=-1 ? rs_cmp_signed[issue_idx0] : 1'b0), .ALU_result(alu_res_0));
    IntAlu u_alu1 (.clk(clk), .rst_n(rst_n), .ALU_op(issue_idx1!=-1 ? rs_int_op[issue_idx1] : {`ALU_OP_WIDTH{1'b0}}), .rs1(opA_1), .rs2(opB_eff_1), .add_c_in(add_cin_1), .cmp_signed(issue_idx1!=-1 ? rs_cmp_signed[issue_idx1] : 1'b0), .ALU_result(alu_res_1));

    wire signed [`DATA_WIDTH:0] mul_a0 = (issue_idx0!=-1 && rs_mul_signed_rs1[issue_idx0]) ? {rs_rs1_val[issue_idx0][`DATA_WIDTH-1], rs_rs1_val[issue_idx0]} : {1'b0, rs_rs1_val[issue_idx0]};
    wire signed [`DATA_WIDTH:0] mul_b0 = (issue_idx0!=-1 && rs_mul_signed_rs2[issue_idx0]) ? {rs_rs2_val[issue_idx0][`DATA_WIDTH-1], rs_rs2_val[issue_idx0]} : {1'b0, rs_rs2_val[issue_idx0]};
    wire signed [`DATA_WIDTH:0] mul_a1 = (issue_idx1!=-1 && rs_mul_signed_rs1[issue_idx1]) ? {rs_rs1_val[issue_idx1][`DATA_WIDTH-1], rs_rs1_val[issue_idx1]} : {1'b0, rs_rs1_val[issue_idx1]};
    wire signed [`DATA_WIDTH:0] mul_b1 = (issue_idx1!=-1 && rs_mul_signed_rs2[issue_idx1]) ? {rs_rs2_val[issue_idx1][`DATA_WIDTH-1], rs_rs2_val[issue_idx1]} : {1'b0, rs_rs2_val[issue_idx1]};
    wire signed [(`DOUBLE_DATA_WIDTH+1):0] mul_full0 = mul_a0 * mul_b0;
    wire signed [(`DOUBLE_DATA_WIDTH+1):0] mul_full1 = mul_a1 * mul_b1;
    wire [`DATA_WIDTH-1:0] mul_res0 = (issue_idx0!=-1 && rs_mul_high[issue_idx0]) ? mul_full0[`DOUBLE_DATA_WIDTH-1:`DATA_WIDTH] : mul_full0[`DATA_WIDTH-1:0];
    wire [`DATA_WIDTH-1:0] mul_res1 = (issue_idx1!=-1 && rs_mul_high[issue_idx1]) ? mul_full1[`DOUBLE_DATA_WIDTH-1:`DATA_WIDTH] : mul_full1[`DATA_WIDTH-1:0];

    reg [`DATA_WIDTH-1:0] csr_old0, csr_old1, csr_new0, csr_new1;
    reg csr_we0, csr_we1;
    reg [`DATA_WIDTH-1:0] link0, link1;
    always @(*) begin
        csr_old0 = {`DATA_WIDTH{1'b0}}; csr_old1 = {`DATA_WIDTH{1'b0}};
        csr_new0 = {`DATA_WIDTH{1'b0}}; csr_new1 = {`DATA_WIDTH{1'b0}};
        csr_we0 = 1'b0; csr_we1 = 1'b0;
        link0 = {`DATA_WIDTH{1'b0}}; link1 = {`DATA_WIDTH{1'b0}};
        if (issue_idx0!=-1) begin
            csr_old0 = csr_file[rs_csr_addr[issue_idx0]];
            csr_new0 = csr_old0;
            if (rs_csr_op[issue_idx0] != `CSR_OP_NONE) begin
                csr_we0 = 1'b1;
                case (rs_csr_op[issue_idx0])
                    `CSR_OP_RW: csr_new0 = rs_use_imm[issue_idx0] ? {27'b0, rs_imm[issue_idx0][4:0]} : rs_rs1_val[issue_idx0];
                    `CSR_OP_RS: csr_new0 = csr_old0 | (rs_use_imm[issue_idx0] ? {27'b0, rs_imm[issue_idx0][4:0]} : rs_rs1_val[issue_idx0]);
                    `CSR_OP_RC: csr_new0 = csr_old0 & ~(rs_use_imm[issue_idx0] ? {27'b0, rs_imm[issue_idx0][4:0]} : rs_rs1_val[issue_idx0]);
                endcase
            end
            link0 = rs_pc[issue_idx0] + `INST_ADD_STEP;
        end
        if (issue_idx1!=-1) begin
            csr_old1 = csr_file[rs_csr_addr[issue_idx1]];
            csr_new1 = csr_old1;
            if (rs_csr_op[issue_idx1] != `CSR_OP_NONE) begin
                csr_we1 = 1'b1;
                case (rs_csr_op[issue_idx1])
                    `CSR_OP_RW: csr_new1 = rs_use_imm[issue_idx1] ? {27'b0, rs_imm[issue_idx1][4:0]} : rs_rs1_val[issue_idx1];
                    `CSR_OP_RS: csr_new1 = csr_old1 | (rs_use_imm[issue_idx1] ? {27'b0, rs_imm[issue_idx1][4:0]} : rs_rs1_val[issue_idx1]);
                    `CSR_OP_RC: csr_new1 = csr_old1 & ~(rs_use_imm[issue_idx1] ? {27'b0, rs_imm[issue_idx1][4:0]} : rs_rs1_val[issue_idx1]);
                endcase
            end
            link1 = rs_pc[issue_idx1] + `INST_ADD_STEP;
        end
    end

    function automatic [`DATA_WIDTH-1:0] select_result;
        input [`FU_DEC_WIDTH-1:0] fu_sel;
        input [`DATA_WIDTH-1:0]   int_res;
        input [`DATA_WIDTH-1:0]   mul_res;
        input [`DATA_WIDTH-1:0]   fp_res;
        input [`ALU_OP_WIDTH-1:0] muldiv_op;
        input [`DATA_WIDTH-1:0]   csr_old;
        input [`DATA_WIDTH-1:0]   link_res;
    begin
        case (fu_sel)
            `FU_DEC_INT:      select_result = int_res;
            `FU_DEC_MULDIV:   select_result = muldiv_op[`ALU_OP_MUL] ? mul_res : {`DATA_WIDTH{1'b0}};
            `FU_DEC_BRANCH:   select_result = link_res;
            `FU_DEC_LSU:      select_result = {`DATA_WIDTH{1'b0}};
            `FU_DEC_SYSTEM:   select_result = csr_old;
            `FU_DEC_FP:       select_result = fp_res;
            default:          select_result = {`DATA_WIDTH{1'b0}};
        endcase
    end
    endfunction

    always @(*) begin
        wb0_valid = 1'b0; wb1_valid = 1'b0; wb2_valid = 1'b0;
        wb0_rob_idx = {`ROB_IDX_WIDTH{1'b0}}; wb1_rob_idx = {`ROB_IDX_WIDTH{1'b0}};
        wb2_rob_idx = {`ROB_IDX_WIDTH{1'b0}};
        wb0_rob_gen = {`ROB_GEN_WIDTH{1'b0}}; wb1_rob_gen = {`ROB_GEN_WIDTH{1'b0}};
        wb2_rob_gen = {`ROB_GEN_WIDTH{1'b0}};
        wb0_value = {`DATA_WIDTH{1'b0}}; wb1_value = {`DATA_WIDTH{1'b0}}; wb2_value = {`DATA_WIDTH{1'b0}};
        wb0_exception = 1'b0; wb1_exception = 1'b0; wb2_exception = 1'b0;
        wb0_dest_tag = {`PREG_IDX_WIDTH{1'b0}}; wb1_dest_tag = {`PREG_IDX_WIDTH{1'b0}}; div_dest_tag = {`PREG_IDX_WIDTH{1'b0}};
        wb0_dest_valid = 1'b0; wb1_dest_valid = 1'b0; div_dest_valid = 1'b0;
        wb0_dest_is_fp = 1'b0; wb1_dest_is_fp = 1'b0; div_dest_is_fp = 1'b0;
        cdb0_valid = 1'b0; cdb1_valid = 1'b0;
        cdb0_tag = {`PREG_IDX_WIDTH{1'b0}}; cdb1_tag = {`PREG_IDX_WIDTH{1'b0}};
        cdb0_value = {`DATA_WIDTH{1'b0}}; cdb1_value = {`DATA_WIDTH{1'b0}};

        pick = 0;
        if (lsu_nodest_valid) begin
            wb2_valid     = 1'b1;
            wb2_value     = lsu_wb_value;
            wb2_rob_idx   = lsu_wb_rob_idx;
            wb2_rob_gen   = lsu_wb_rob_gen;
            wb2_exception = lsu_wb_exception;
        end
        if (cand3_valid && pick < 2) begin
            wb0_valid     = 1'b1;
            wb0_value     = cand3_val;
            wb0_rob_idx   = cand3_rob;
            wb0_rob_gen   = cand3_rob_gen;
            wb0_exception = cand3_exc;
            wb0_dest_tag  = cand3_tag;
            wb0_dest_valid= cand3_has_dest;
            wb0_dest_is_fp= cand3_is_fp;
            if (cand3_has_dest) begin
                cdb0_valid    = 1'b1;
                cdb0_tag      = cand3_tag;
                cdb0_value    = cand3_val;
            end
            pick = pick + 1;
        end
        if (cand0_valid && pick < 2) begin
            if (pick==0) begin
                wb0_valid     = 1'b1;
                wb0_value     = cand0_val;
                wb0_rob_idx   = cand0_rob;
                wb0_rob_gen   = cand0_rob_gen;
                wb0_exception = cand0_exc;
                wb0_dest_tag  = cand0_tag;
                wb0_dest_valid= cand0_has_dest;
                wb0_dest_is_fp= cand0_is_fp;
                if (cand0_has_dest) begin
                    cdb0_valid    = 1'b1;
                    cdb0_tag      = cand0_tag;
                    cdb0_value    = cand0_val;
                end
            end else begin
                wb1_valid     = 1'b1;
                wb1_value     = cand0_val;
                wb1_rob_idx   = cand0_rob;
                wb1_rob_gen   = cand0_rob_gen;
                wb1_exception = cand0_exc;
                wb1_dest_tag  = cand0_tag;
                wb1_dest_valid= cand0_has_dest;
                wb1_dest_is_fp= cand0_is_fp;
                if (cand0_has_dest) begin
                    cdb1_valid    = 1'b1;
                    cdb1_tag      = cand0_tag;
                    cdb1_value    = cand0_val;
                end
            end
            pick = pick + 1;
        end
        if (cand1_valid && pick < 2) begin
            if (pick==0) begin
                wb0_valid     = 1'b1;
                wb0_value     = cand1_val;
                wb0_rob_idx   = cand1_rob;
                wb0_rob_gen   = cand1_rob_gen;
                wb0_exception = cand1_exc;
                wb0_dest_tag  = cand1_tag;
                wb0_dest_valid= cand1_has_dest;
                wb0_dest_is_fp= cand1_is_fp;
                if (cand1_has_dest) begin
                    cdb0_valid    = 1'b1;
                    cdb0_tag      = cand1_tag;
                    cdb0_value    = cand1_val;
                end
            end else begin
                wb1_valid     = 1'b1;
                wb1_value     = cand1_val;
                wb1_rob_idx   = cand1_rob;
                wb1_rob_gen   = cand1_rob_gen;
                wb1_exception = cand1_exc;
                wb1_dest_tag  = cand1_tag;
                wb1_dest_valid= cand1_has_dest;
                wb1_dest_is_fp= cand1_is_fp;
                if (cand1_has_dest) begin
                    cdb1_valid    = 1'b1;
                    cdb1_tag      = cand1_tag;
                    cdb1_value    = cand1_val;
                end
            end
            pick = pick + 1;
        end
        if (cand2_valid && pick < 2) begin
            if (pick==0) begin
                wb0_valid     = 1'b1;
                wb0_value     = cand2_val;
                wb0_rob_idx   = cand2_rob;
                wb0_rob_gen   = cand2_rob_gen;
                wb0_exception = cand2_exc;
                wb0_dest_tag  = cand2_tag;
                wb0_dest_valid= 1'b1;
                wb0_dest_is_fp= cand2_is_fp;
                cdb0_valid    = 1'b1;
                cdb0_tag      = cand2_tag;
                cdb0_value    = cand2_val;
            end else begin
                wb1_valid     = 1'b1;
                wb1_value     = cand2_val;
                wb1_rob_idx   = cand2_rob;
                wb1_rob_gen   = cand2_rob_gen;
                wb1_exception = cand2_exc;
                wb1_dest_tag  = cand2_tag;
                wb1_dest_valid= 1'b1;
                wb1_dest_is_fp= cand2_is_fp;
                cdb1_valid    = 1'b1;
                cdb1_tag      = cand2_tag;
                cdb1_value    = cand2_val;
            end
            pick = pick + 1;
        end
    end

    // PRF
    integer i_idx;
    integer issue_cnt;
    integer enq_cnt;
    integer found_idx0;
    integer found_idx1;
    integer new_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rs_count <= 0;
            age_counter <= 0;
            for (i_idx = 0; i_idx < RS_DEPTH; i_idx = i_idx + 1) begin
                rs_valid[i_idx] <= 1'b0;
            end
            i_wr_we0 <= 1'b0; i_wr_we1 <= 1'b0; f_wr_we0 <= 1'b0; f_wr_we1 <= 1'b0;
            wb0_valid <= 1'b0; wb1_valid <= 1'b0; wb2_valid <= 1'b0;
            wb0_exception <= 1'b0; wb1_exception <= 1'b0;
            wb0_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            wb1_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            wb2_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            lsu_valid <= 1'b0;
            lsu_pending <= 1'b0;
            lsu_addr <= 32'b0;
            lsu_wdata <= 32'b0;
            lsu_mem_op <= {`MEM_OP_WIDTH{1'b0}};
            lsu_mem_is_load <= 1'b0;
            lsu_mem_unsigned <= 1'b0;
            lsu_rob_idx <= {`ROB_IDX_WIDTH{1'b0}};
            lsu_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            lsu_rd_tag <= {`PREG_IDX_WIDTH{1'b0}};
            lsu_rd_is_fp <= 1'b0;
            div_busy <= 1'b0;
            div_done <= 1'b0;
            div_counter <= 4'b0;
            div_dest_tag <= {`PREG_IDX_WIDTH{1'b0}};
            div_dest_valid <= 1'b0;
            div_dest_is_fp <= 1'b0;
            div_rob_idx <= {`ROB_IDX_WIDTH{1'b0}};
            div_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            div_exception <= 1'b0;
            div_value <= {`DATA_WIDTH{1'b0}};
            redirect_valid <= 1'b0;
            redirect_target <= {`INST_ADDR_WIDTH{1'b0}};
            redirect_rob_idx <= {`ROB_IDX_WIDTH{1'b0}};
            bp_update0_valid <= 1'b0; bp_update1_valid <= 1'b0;
            bp_update0_pc <= {`INST_ADDR_WIDTH{1'b0}}; bp_update1_pc <= {`INST_ADDR_WIDTH{1'b0}};
            bp_update0_taken <= 1'b0; bp_update1_taken <= 1'b0;
            bp_update0_target <= {`INST_ADDR_WIDTH{1'b0}}; bp_update1_target <= {`INST_ADDR_WIDTH{1'b0}};
            bp_update0_hist <= {`BP_GHR_BITS{1'b0}}; bp_update1_hist <= {`BP_GHR_BITS{1'b0}};
            bp_update0_is_call <= 1'b0; bp_update1_is_call <= 1'b0;
            bp_update0_is_return <= 1'b0; bp_update1_is_return <= 1'b0;
        end else if (flush) begin
            i_wr_we0 <= 1'b0; i_wr_we1 <= 1'b0; f_wr_we0 <= 1'b0; f_wr_we1 <= 1'b0;
            wb0_valid <= 1'b0; wb1_valid <= 1'b0; wb2_valid <= 1'b0;
            wb0_exception <= 1'b0; wb1_exception <= 1'b0;
            wb0_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            wb1_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            wb2_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            lsu_valid <= 1'b0;
            lsu_pending <= 1'b0;
            lsu_rob_gen <= {`ROB_GEN_WIDTH{1'b0}};
            redirect_valid <= 1'b0;
            redirect_target <= {`INST_ADDR_WIDTH{1'b0}};
            redirect_rob_idx <= {`ROB_IDX_WIDTH{1'b0}};
            bp_update0_valid <= 1'b0; bp_update1_valid <= 1'b0;
            bp_update0_pc <= {`INST_ADDR_WIDTH{1'b0}}; bp_update1_pc <= {`INST_ADDR_WIDTH{1'b0}};
            bp_update0_taken <= 1'b0; bp_update1_taken <= 1'b0;
            bp_update0_target <= {`INST_ADDR_WIDTH{1'b0}}; bp_update1_target <= {`INST_ADDR_WIDTH{1'b0}};
            bp_update0_hist <= {`BP_GHR_BITS{1'b0}}; bp_update1_hist <= {`BP_GHR_BITS{1'b0}};
            bp_update0_is_call <= 1'b0; bp_update1_is_call <= 1'b0;
            bp_update0_is_return <= 1'b0; bp_update1_is_return <= 1'b0;
            if (flush_is_redirect && !redirect_keep_none) begin
                new_count = 0;
                for (i_idx = 0; i_idx < RS_DEPTH; i_idx = i_idx + 1) begin
                    if (rs_valid[i_idx] && in_range_inclusive(rs_rob_idx[i_idx], rob_head, flush_rob_idx)) begin
                        new_count = new_count + 1;
                    end else begin
                        rs_valid[i_idx] <= 1'b0;
                        rs_rs1_ready[i_idx] <= 1'b0;
                        rs_rs2_ready[i_idx] <= 1'b0;
                    end
                end
                rs_count <= new_count;
                if (div_busy && !in_range_inclusive(div_rob_idx, rob_head, flush_rob_idx)) begin
                    div_busy <= 1'b0;
                    div_done <= 1'b0;
                end
            end else begin
                rs_count <= 0;
                age_counter <= 0;
                for (i_idx = 0; i_idx < RS_DEPTH; i_idx = i_idx + 1) begin
                    rs_valid[i_idx] <= 1'b0;
                    rs_rs1_ready[i_idx] <= 1'b0;
                    rs_rs2_ready[i_idx] <= 1'b0;
                end
                div_busy <= 1'b0;
                div_done <= 1'b0;
            end
        end else begin
            // CSR writes
            if (csr_we0 && issue_idx0 != -1) csr_file[rs_csr_addr[issue_idx0]] <= csr_new0;
            if (csr_we1 && issue_idx1 != -1) csr_file[rs_csr_addr[issue_idx1]] <= csr_new1;

            // Default PRF writes low
            i_wr_we0 <= 1'b0; i_wr_we1 <= 1'b0; f_wr_we0 <= 1'b0; f_wr_we1 <= 1'b0;
            lsu_valid <= 1'b0;

            // Track LSU request in flight until LSU actually goes busy (registered accept).
            if (lsu_pending) begin
                if (lsu_busy || lsu_wb_valid) begin
                    lsu_pending <= 1'b0;
                end
            end else if (lsu_can_issue) begin
                lsu_pending <= 1'b1;
            end

            // PRF writes from WB
            if (wb0_valid && wb0_dest_valid) begin
                if (wb0_dest_is_fp) begin
                    f_wr_we0   <= 1'b1;
                    f_wr_addr0 <= wb0_dest_tag;
                    f_wr_data0 <= wb0_value;
                end else begin
                    i_wr_we0   <= 1'b1;
                    i_wr_addr0 <= wb0_dest_tag;
                    i_wr_data0 <= wb0_value;
                end
            end
            if (wb1_valid && wb1_dest_valid) begin
                if (wb1_dest_is_fp) begin
                    f_wr_we1   <= 1'b1;
                    f_wr_addr1 <= wb1_dest_tag;
                    f_wr_data1 <= wb1_value;
                end else begin
                    i_wr_we1   <= 1'b1;
                    i_wr_addr1 <= wb1_dest_tag;
                    i_wr_data1 <= wb1_value;
                end
            end
            if (div_done && div_dest_valid && !wb0_valid && !wb1_valid) begin
                if (div_dest_is_fp) begin
                    f_wr_we0   <= 1'b1;
                    f_wr_addr0 <= div_dest_tag;
                    f_wr_data0 <= div_value;
                end else begin
                    i_wr_we0   <= 1'b1;
                    i_wr_addr0 <= div_dest_tag;
                    i_wr_data0 <= div_value;
                end
            end

            // LSU request (single outstanding)
            if (lsu_can_issue) begin
                lsu_valid <= 1'b1;
                lsu_addr <= rs_rs1_val[mem_issue_idx] + rs_imm[mem_issue_idx];
                lsu_wdata <= rs_rs2_val[mem_issue_idx];
                lsu_mem_op <= rs_mem_op[mem_issue_idx];
                lsu_mem_is_load <= rs_mem_is_load[mem_issue_idx];
                lsu_mem_unsigned <= rs_mem_unsigned[mem_issue_idx];
                lsu_rob_idx <= rs_rob_idx[mem_issue_idx];
                lsu_rob_gen <= rs_rob_gen[mem_issue_idx];
                lsu_rd_tag <= rs_rd_tag[mem_issue_idx];
                lsu_rd_is_fp <= rs_rd_is_fp[mem_issue_idx];
            end

            // Clear issued entries (except div which is taken by latency path)
            issue_cnt = 0;
            if (issue_idx0 != -1 && rs_valid[issue_idx0]) begin
                if (rs_fu_sel[issue_idx0] == `FU_DEC_LSU) begin
                    if (lsu_can_issue && (issue_idx0 == mem_issue_idx)) begin
                        rs_valid[issue_idx0] <= 1'b0;
                        issue_cnt = issue_cnt + 1;
                    end
                end else if (!(rs_fu_sel[issue_idx0]==`FU_DEC_MULDIV && rs_muldiv_op[issue_idx0][`ALU_OP_DIV])) begin
                    rs_valid[issue_idx0] <= 1'b0;
                    issue_cnt = issue_cnt + 1;
                end else if (!div_busy) begin
                    // Start divider
                    div_busy <= 1'b1;
                    div_counter <= 4'd8;
                    div_dest_tag <= rs_rd_tag[issue_idx0];
                    div_dest_valid <= (rs_rd_tag[issue_idx0] != {`PREG_IDX_WIDTH{1'b0}});
                    div_dest_is_fp <= rs_rd_is_fp[issue_idx0];
                    div_rob_idx <= rs_rob_idx[issue_idx0];
                    div_rob_gen <= rs_rob_gen[issue_idx0];
                    div_exception <= rs_illegal[issue_idx0];
                    // Simple signed/unsigned division/remainder
                    if (rs_div_is_rem[issue_idx0]) begin
                        if (rs_rs2_val[issue_idx0]==0) div_value <= rs_rs1_val[issue_idx0];
                        else if (rs_div_signed[issue_idx0])
                            div_value <= $signed(rs_rs1_val[issue_idx0]) % $signed(rs_rs2_val[issue_idx0]);
                        else
                            div_value <= $unsigned(rs_rs1_val[issue_idx0]) % $unsigned(rs_rs2_val[issue_idx0]);
                    end else begin
                        if (rs_rs2_val[issue_idx0]==0) div_value <= {`DATA_WIDTH{1'b0}};
                        else if (rs_div_signed[issue_idx0])
                            div_value <= $signed(rs_rs1_val[issue_idx0]) / $signed(rs_rs2_val[issue_idx0]);
                        else
                            div_value <= $unsigned(rs_rs1_val[issue_idx0]) / $unsigned(rs_rs2_val[issue_idx0]);
                    end
                    rs_valid[issue_idx0] <= 1'b0;
                    issue_cnt = issue_cnt + 1;
                end
            end
            if (issue_idx1 != -1 && rs_valid[issue_idx1]) begin
                if (rs_fu_sel[issue_idx1] == `FU_DEC_LSU) begin
                    if (lsu_can_issue && (issue_idx1 == mem_issue_idx)) begin
                        rs_valid[issue_idx1] <= 1'b0;
                        issue_cnt = issue_cnt + 1;
                    end
                end else if (!(rs_fu_sel[issue_idx1]==`FU_DEC_MULDIV && rs_muldiv_op[issue_idx1][`ALU_OP_DIV])) begin
                    rs_valid[issue_idx1] <= 1'b0;
                    issue_cnt = issue_cnt + 1;
                end else if (!div_busy) begin
                    div_busy <= 1'b1;
                    div_counter <= 4'd8;
                    div_dest_tag <= rs_rd_tag[issue_idx1];
                    div_dest_valid <= (rs_rd_tag[issue_idx1] != {`PREG_IDX_WIDTH{1'b0}});
                    div_dest_is_fp <= rs_rd_is_fp[issue_idx1];
                    div_rob_idx <= rs_rob_idx[issue_idx1];
                    div_rob_gen <= rs_rob_gen[issue_idx1];
                    div_exception <= rs_illegal[issue_idx1];
                    if (rs_div_is_rem[issue_idx1]) begin
                        if (rs_rs2_val[issue_idx1]==0) div_value <= rs_rs1_val[issue_idx1];
                        else if (rs_div_signed[issue_idx1])
                            div_value <= $signed(rs_rs1_val[issue_idx1]) % $signed(rs_rs2_val[issue_idx1]);
                        else
                            div_value <= $unsigned(rs_rs1_val[issue_idx1]) % $unsigned(rs_rs2_val[issue_idx1]);
                    end else begin
                        if (rs_rs2_val[issue_idx1]==0) div_value <= {`DATA_WIDTH{1'b0}};
                        else if (rs_div_signed[issue_idx1])
                            div_value <= $signed(rs_rs1_val[issue_idx1]) / $signed(rs_rs2_val[issue_idx1]);
                        else
                            div_value <= $unsigned(rs_rs1_val[issue_idx1]) / $unsigned(rs_rs2_val[issue_idx1]);
                    end
                    rs_valid[issue_idx1] <= 1'b0;
                    issue_cnt = issue_cnt + 1;
                end
            end

            // Divider countdown
            div_done <= 1'b0;
            if (div_busy) begin
                if (div_counter != 0) begin
                    div_counter <= div_counter - 1'b1;
                    if (div_counter == 1) begin
                        div_done <= 1'b1;
                        div_busy <= 1'b0;
                    end
                end
            end

            // Wakeup with CDB
            for (i_idx = 0; i_idx < RS_DEPTH; i_idx = i_idx + 1) begin
                if (rs_valid[i_idx]) begin
                    if (!rs_rs1_ready[i_idx] && cdb0_valid && (rs_rs1_tag[i_idx] == cdb0_tag)) begin
                        rs_rs1_ready[i_idx] <= 1'b1;
                        rs_rs1_val[i_idx]   <= cdb0_value;
                    end
                    if (!rs_rs1_ready[i_idx] && cdb1_valid && (rs_rs1_tag[i_idx] == cdb1_tag)) begin
                        rs_rs1_ready[i_idx] <= 1'b1;
                        rs_rs1_val[i_idx]   <= cdb1_value;
                    end
                    if (!rs_rs2_ready[i_idx] && cdb0_valid && (rs_rs2_tag[i_idx] == cdb0_tag)) begin
                        rs_rs2_ready[i_idx] <= 1'b1;
                        rs_rs2_val[i_idx]   <= cdb0_value;
                    end
                    if (!rs_rs2_ready[i_idx] && cdb1_valid && (rs_rs2_tag[i_idx] == cdb1_tag)) begin
                        rs_rs2_ready[i_idx] <= 1'b1;
                        rs_rs2_val[i_idx]   <= cdb1_value;
                    end
                end
            end

            // Enqueue
            found_idx0 = -1;
            found_idx1 = -1;
            enq_cnt = 0;
            if (!stall_dispatch) begin
                if (in_inst_valid[0]) begin
                    for (i_idx = 0; i_idx < RS_DEPTH; i_idx = i_idx + 1) begin
                        if (!rs_valid[i_idx] && found_idx0 == -1) found_idx0 = i_idx;
                    end
                end
                if (in_inst_valid[1]) begin
                    for (i_idx = 0; i_idx < RS_DEPTH; i_idx = i_idx + 1) begin
                        if (!rs_valid[i_idx] && (i_idx != found_idx0) && found_idx1 == -1) found_idx1 = i_idx;
                    end
                end
                if (found_idx0 != -1) begin
                    rs_valid[found_idx0]      <= 1'b1;
                    rs_age[found_idx0]        <= age_counter;
                    rs_fu_sel[found_idx0]     <= in_fu_sel_0;
                    rs_int_op[found_idx0]     <= in_int_op_0;
                    rs_int_sub[found_idx0]    <= in_int_is_sub_0;
                    rs_cmp_signed[found_idx0] <= in_cmp_signed_0;
                    rs_muldiv_op[found_idx0]  <= in_muldiv_op_0;
                    rs_mul_high[found_idx0]   <= in_mul_high_0;
                    rs_mul_signed_rs1[found_idx0] <= in_mul_signed_rs1_0;
                    rs_mul_signed_rs2[found_idx0] <= in_mul_signed_rs2_0;
                    rs_div_signed[found_idx0] <= in_div_signed_0;
                    rs_div_is_rem[found_idx0] <= in_div_is_rem_0;
                    rs_branch_op[found_idx0]  <= in_branch_op_0;
                    rs_mem_op[found_idx0]     <= in_mem_op_0;
                    rs_mem_is_load[found_idx0]<= in_mem_is_load_0;
                    rs_mem_unsigned[found_idx0]<= in_mem_unsigned_0;
                    rs_csr_op[found_idx0]     <= in_csr_op_0;
                    rs_csr_addr[found_idx0]   <= in_csr_addr_0;
                    rs_fp_op[found_idx0]      <= in_fp_op_0;
                    rs_illegal[found_idx0]    <= in_illegal_0;
                    rs_inst[found_idx0]       <= in_inst_0;
                    rs_pc[found_idx0]         <= in_pc_0;
                    rs_imm[found_idx0]        <= in_imm_0;
                    rs_use_imm[found_idx0]    <= in_use_imm_0;
                    rs_pred_taken[found_idx0] <= in_pred_taken_0;
                    rs_pred_target[found_idx0]<= in_pred_target_0;
                    rs_pred_hist[found_idx0]  <= in_pred_hist_0;
                    rs_arch_rd[found_idx0]    <= in_rd_0;
                    rs_arch_rs1[found_idx0]   <= in_rs1_0;
                    rs_rs1_ready[found_idx0]  <= rs1_ready_now_0;
                    rs_rs2_ready[found_idx0]  <= rs2_ready_now_0;
                    rs_rs1_tag[found_idx0]    <= in_rs1_preg_0;
                    rs_rs2_tag[found_idx0]    <= in_rs2_preg_0;
                    rs_rs1_val[found_idx0]    <= rs1_is_x0_0 ? {`DATA_WIDTH{1'b0}} :
                                                rs1_cdb1_hit_0 ? cdb1_value :
                                                rs1_cdb0_hit_0 ? cdb0_value :
                                                in_rs1_ready_now_0 ? (in_rs1_is_fp_0 ? fp_rs1_data0 : int_rs1_data0) :
                                                {`DATA_WIDTH{1'b0}};
                    rs_rs2_val[found_idx0]    <= rs2_is_x0_0 ? {`DATA_WIDTH{1'b0}} :
                                                rs2_cdb1_hit_0 ? cdb1_value :
                                                rs2_cdb0_hit_0 ? cdb0_value :
                                                in_rs2_ready_now_0 ? (in_rs2_is_fp_0 ? fp_rs2_data0 : int_rs2_data0) :
                                                {`DATA_WIDTH{1'b0}};
                    rs_rd_tag[found_idx0]     <= in_rd_preg_0;
                    rs_rd_is_fp[found_idx0]   <= in_rd_is_fp_0;
                    rs_rob_idx[found_idx0]    <= in_rob_idx_0;
                    rs_rob_gen[found_idx0]    <= in_rob_gen_0;
                    enq_cnt = enq_cnt + 1;
                    age_counter <= age_counter + 1'b1;
                end
                if (found_idx1 != -1) begin
                    rs_valid[found_idx1]      <= 1'b1;
                    rs_age[found_idx1]        <= age_counter;
                    rs_fu_sel[found_idx1]     <= in_fu_sel_1;
                    rs_int_op[found_idx1]     <= in_int_op_1;
                    rs_int_sub[found_idx1]    <= in_int_is_sub_1;
                    rs_cmp_signed[found_idx1] <= in_cmp_signed_1;
                    rs_muldiv_op[found_idx1]  <= in_muldiv_op_1;
                    rs_mul_high[found_idx1]   <= in_mul_high_1;
                    rs_mul_signed_rs1[found_idx1] <= in_mul_signed_rs1_1;
                    rs_mul_signed_rs2[found_idx1] <= in_mul_signed_rs2_1;
                    rs_div_signed[found_idx1] <= in_div_signed_1;
                    rs_div_is_rem[found_idx1] <= in_div_is_rem_1;
                    rs_branch_op[found_idx1]  <= in_branch_op_1;
                    rs_mem_op[found_idx1]     <= in_mem_op_1;
                    rs_mem_is_load[found_idx1]<= in_mem_is_load_1;
                    rs_mem_unsigned[found_idx1]<= in_mem_unsigned_1;
                    rs_csr_op[found_idx1]     <= in_csr_op_1;
                    rs_csr_addr[found_idx1]   <= in_csr_addr_1;
                    rs_fp_op[found_idx1]      <= in_fp_op_1;
                    rs_illegal[found_idx1]    <= in_illegal_1;
                    rs_inst[found_idx1]       <= in_inst_1;
                    rs_pc[found_idx1]         <= in_pc_1;
                    rs_imm[found_idx1]        <= in_imm_1;
                    rs_use_imm[found_idx1]    <= in_use_imm_1;
                    rs_pred_taken[found_idx1] <= in_pred_taken_1;
                    rs_pred_target[found_idx1]<= in_pred_target_1;
                    rs_pred_hist[found_idx1]  <= in_pred_hist_1;
                    rs_arch_rd[found_idx1]    <= in_rd_1;
                    rs_arch_rs1[found_idx1]   <= in_rs1_1;
                    rs_rs1_ready[found_idx1]  <= rs1_ready_now_1;
                    rs_rs2_ready[found_idx1]  <= rs2_ready_now_1;
                    rs_rs1_tag[found_idx1]    <= in_rs1_preg_1;
                    rs_rs2_tag[found_idx1]    <= in_rs2_preg_1;
                    rs_rs1_val[found_idx1]    <= rs1_is_x0_1 ? {`DATA_WIDTH{1'b0}} :
                                                rs1_cdb1_hit_1 ? cdb1_value :
                                                rs1_cdb0_hit_1 ? cdb0_value :
                                                in_rs1_ready_now_1 ? (in_rs1_is_fp_1 ? fp_rs1_data1 : int_rs1_data1) :
                                                {`DATA_WIDTH{1'b0}};
                    rs_rs2_val[found_idx1]    <= rs2_is_x0_1 ? {`DATA_WIDTH{1'b0}} :
                                                rs2_cdb1_hit_1 ? cdb1_value :
                                                rs2_cdb0_hit_1 ? cdb0_value :
                                                in_rs2_ready_now_1 ? (in_rs2_is_fp_1 ? fp_rs2_data1 : int_rs2_data1) :
                                                {`DATA_WIDTH{1'b0}};
                    rs_rd_tag[found_idx1]     <= in_rd_preg_1;
                    rs_rd_is_fp[found_idx1]   <= in_rd_is_fp_1;
                    rs_rob_idx[found_idx1]    <= in_rob_idx_1;
                    rs_rob_gen[found_idx1]    <= in_rob_gen_1;
                    enq_cnt = enq_cnt + 1;
                    age_counter <= age_counter + 1'b1;
                end
            end

            rs_count <= rs_count - issue_cnt + enq_cnt;

            // Redirect pulse and predictor updates
            redirect_valid <= 1'b0;
            redirect_target <= {`INST_ADDR_WIDTH{1'b0}};
            redirect_rob_idx <= {`ROB_IDX_WIDTH{1'b0}};
            bp_update0_valid <= 1'b0; bp_update1_valid <= 1'b0;
            bp_update0_pc <= {`INST_ADDR_WIDTH{1'b0}}; bp_update1_pc <= {`INST_ADDR_WIDTH{1'b0}};
            bp_update0_taken <= 1'b0; bp_update1_taken <= 1'b0;
            bp_update0_target <= {`INST_ADDR_WIDTH{1'b0}}; bp_update1_target <= {`INST_ADDR_WIDTH{1'b0}};
            bp_update0_hist <= {`BP_GHR_BITS{1'b0}}; bp_update1_hist <= {`BP_GHR_BITS{1'b0}};
            bp_update0_is_call <= 1'b0; bp_update1_is_call <= 1'b0;
            bp_update0_is_return <= 1'b0; bp_update1_is_return <= 1'b0;

            if (br0_is_branch) begin
                bp_update0_valid   <= 1'b1;
                bp_update0_pc      <= rs_pc[issue_idx0];
                bp_update0_taken   <= br0_taken;
                bp_update0_target  <= br0_target;
                bp_update0_hist    <= rs_pred_hist[issue_idx0];
                bp_update0_is_call <= br0_is_call;
                bp_update0_is_return <= br0_is_return;
            end
            if (br1_is_branch) begin
                bp_update1_valid   <= 1'b1;
                bp_update1_pc      <= rs_pc[issue_idx1];
                bp_update1_taken   <= br1_taken;
                bp_update1_target  <= br1_target;
                bp_update1_hist    <= rs_pred_hist[issue_idx1];
                bp_update1_is_call <= br1_is_call;
                bp_update1_is_return <= br1_is_return;
            end

            if (br0_mispredict) begin
                redirect_valid  <= 1'b1;
                redirect_target <= br0_taken ? br0_target : br0_seq_next;
                redirect_rob_idx <= rs_rob_idx[issue_idx0];
            end else if (br1_mispredict) begin
                redirect_valid  <= 1'b1;
                redirect_target <= br1_taken ? br1_target : br1_seq_next;
                redirect_rob_idx <= rs_rob_idx[issue_idx1];
            end
        end
    end

endmodule
