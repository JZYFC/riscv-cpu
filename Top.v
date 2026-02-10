`include "riscv_define.v"

// Top-level pipeline: IF -> InstructionBuffer -> PreDecode -> RegRename -> PostDecode -> IssueBuffer.
// Memory/cache/FP execution are still stubbed; commit-side exceptions trigger a flush to a trap vector.
module Top (
    input clk,
    input rst_n
);

// ----------------------------
// Global control / flush
// ----------------------------
wire redirect_valid;
wire [`INST_ADDR_WIDTH-1:0] redirect_target;
wire [`ROB_IDX_WIDTH-1:0] redirect_rob_idx;

// Commit/ROB visibility
wire commit0_valid;
wire commit1_valid;
wire [`ROB_IDX_WIDTH-1:0] commit0_rob_idx;
wire [`ROB_IDX_WIDTH-1:0] commit1_rob_idx;
wire [`INST_ADDR_WIDTH-1:0] commit0_pc;
wire [`INST_ADDR_WIDTH-1:0] commit1_pc;
wire commit0_has_dest;
wire commit1_has_dest;
wire commit0_is_float;
wire commit1_is_float;
wire [`REG_ADDR_WIDTH-1:0] commit0_arch_rd;
wire [`REG_ADDR_WIDTH-1:0] commit1_arch_rd;
wire [`PREG_IDX_WIDTH-1:0] commit0_new_preg;
wire [`PREG_IDX_WIDTH-1:0] commit1_new_preg;
wire [`PREG_IDX_WIDTH-1:0] commit0_old_preg;
wire [`PREG_IDX_WIDTH-1:0] commit1_old_preg;
wire [`DATA_WIDTH-1:0] commit0_value;
wire [`DATA_WIDTH-1:0] commit1_value;
wire commit0_exception;
wire commit1_exception;
wire rob_empty;
wire [`ROB_IDX_WIDTH-1:0] rob_head;
wire rn_stall;

wire commit_exc = commit0_exception;
wire global_flush = redirect_valid | commit_exc;
wire [`INST_ADDR_WIDTH-1:0] flush_target = redirect_valid ? redirect_target : `TRAP_VECTOR;

// ----------------------------
// Shared PRF (int/fp)
// ----------------------------
wire [`DATA_WIDTH-1:0] prf_int_rs1_data0, prf_int_rs2_data0, prf_int_rs1_data1, prf_int_rs2_data1;
wire [`DATA_WIDTH-1:0] prf_fp_rs1_data0, prf_fp_rs2_data0, prf_fp_rs1_data1, prf_fp_rs2_data1;
wire [`PREG_IDX_WIDTH-1:0] prf_i_wr_addr0, prf_i_wr_addr1, prf_f_wr_addr0, prf_f_wr_addr1;
wire [`DATA_WIDTH-1:0] prf_i_wr_data0, prf_i_wr_data1, prf_f_wr_data0, prf_f_wr_data1;
wire prf_i_wr_we0, prf_i_wr_we1, prf_f_wr_we0, prf_f_wr_we1;

// ----------------------------
// IF stage
// ----------------------------
wire [`INST_WIDTH-1:0] if_inst_addr_0;
wire [`INST_WIDTH-1:0] if_inst_addr_1;
wire [`INST_WIDTH-1:0] if_inst_0;
wire [`INST_WIDTH-1:0] if_inst_1;
wire [`IF_BATCH_SIZE-1:0] if_inst_valid;
wire if_pred_taken_0, if_pred_taken_1;
wire [`INST_ADDR_WIDTH-1:0] if_pred_target_0, if_pred_target_1;
wire [`BP_GHR_BITS-1:0] if_pred_hist_0, if_pred_hist_1;
wire ib_stall_if;
wire issue_stall;
wire dispatch_pipe_stall;
wire dispatch_ready = ~rn_stall & ~dispatch_pipe_stall;
wire ic_req;
wire [`INST_ADDR_WIDTH-1:0] ic_paddr;
wire [127:0] ic_rdata_line;
wire ic_valid;
wire ic_stall;
wire issue_bp_update0_valid, issue_bp_update1_valid;
wire [`INST_ADDR_WIDTH-1:0] issue_bp_update0_pc, issue_bp_update1_pc;
wire issue_bp_update0_taken, issue_bp_update1_taken;
wire [`INST_ADDR_WIDTH-1:0] issue_bp_update0_target, issue_bp_update1_target;
wire [`BP_GHR_BITS-1:0] issue_bp_update0_hist, issue_bp_update1_hist;
wire issue_bp_update0_is_call, issue_bp_update1_is_call;
wire issue_bp_update0_is_return, issue_bp_update1_is_return;

IF u_if (
    .clk(clk),
    .rst_n(rst_n),
    .stall(ib_stall_if),
    .flush(global_flush),
    .redirect_valid(global_flush),
    .redirect_pc(flush_target),
    .bp_update0_valid(issue_bp_update0_valid),
    .bp_update0_pc(issue_bp_update0_pc),
    .bp_update0_taken(issue_bp_update0_taken),
    .bp_update0_target(issue_bp_update0_target),
    .bp_update0_hist(issue_bp_update0_hist),
    .bp_update0_is_call(issue_bp_update0_is_call),
    .bp_update0_is_return(issue_bp_update0_is_return),
    .bp_update1_valid(issue_bp_update1_valid),
    .bp_update1_pc(issue_bp_update1_pc),
    .bp_update1_taken(issue_bp_update1_taken),
    .bp_update1_target(issue_bp_update1_target),
    .bp_update1_hist(issue_bp_update1_hist),
    .bp_update1_is_call(issue_bp_update1_is_call),
    .bp_update1_is_return(issue_bp_update1_is_return),
    .ic_req(ic_req),
    .ic_paddr(ic_paddr),
    .ic_rdata_line(ic_rdata_line),
    .ic_valid(ic_valid),
    .ic_stall(ic_stall),
    .out_inst_addr_0(if_inst_addr_0),
    .out_inst_addr_1(if_inst_addr_1),
    .out_inst_0(if_inst_0),
    .out_inst_1(if_inst_1),
    .out_inst_valid(if_inst_valid),
    .out_pred_taken_0(if_pred_taken_0),
    .out_pred_target_0(if_pred_target_0),
    .out_pred_hist_0(if_pred_hist_0),
    .out_pred_taken_1(if_pred_taken_1),
    .out_pred_target_1(if_pred_target_1),
    .out_pred_hist_1(if_pred_hist_1)
);

// ----------------------------
// I-Cache + Memory (D side stubbed for now)
// ----------------------------
wire         ic_mem_req;
wire         ic_mem_we;
wire [31:0]  ic_mem_addr;
wire [127:0] ic_mem_wdata;
wire [127:0] ic_mem_rdata;
wire         ic_mem_ready;

wire         d_mem_req;
wire         d_mem_we;
wire [31:0]  d_mem_addr;
wire [127:0] d_mem_wdata;
wire [127:0] d_mem_rdata;
wire         d_mem_ready;

wire         mem_req;
wire         mem_we;
wire [31:0]  mem_addr;
wire [127:0] mem_wdata;
wire [127:0] mem_rdata;
wire         mem_ready;

ICache u_icache (
    .clk(clk),
    .rst_n(rst_n),
    .paddr(ic_paddr),
    .req(ic_req),
    .rdata_line(ic_rdata_line),
    .valid_out(ic_valid),
    .stall_cpu(ic_stall),
    .mem_req(ic_mem_req),
    .mem_we(ic_mem_we),
    .mem_addr(ic_mem_addr),
    .mem_wdata(ic_mem_wdata),
    .mem_rdata(ic_mem_rdata),
    .mem_ready(ic_mem_ready)
);

MemArbiter u_memarb (
    .clk(clk),
    .rst_n(rst_n),
    .i_req(ic_mem_req),
    .i_we(ic_mem_we),
    .i_addr(ic_mem_addr),
    .i_wdata(ic_mem_wdata),
    .i_rdata(ic_mem_rdata),
    .i_ready(ic_mem_ready),
    .d_req(d_mem_req),
    .d_we(d_mem_we),
    .d_addr(d_mem_addr),
    .d_wdata(d_mem_wdata),
    .d_rdata(d_mem_rdata),
    .d_ready(d_mem_ready),
    .mem_req(mem_req),
    .mem_we(mem_we),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_rdata(mem_rdata),
    .mem_ready(mem_ready)
);

MainMemory u_mainmem (
    .clk(clk),
    .rst_n(rst_n),
    .mem_req(mem_req),
    .mem_we(mem_we),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_rdata(mem_rdata),
    .mem_ready(mem_ready)
);

// ----------------------------
// Instruction buffer
// ----------------------------
wire [`IF_BATCH_SIZE-1:0] ib_valid;
wire [`INST_WIDTH-1:0] ib_inst_0;
wire [`INST_WIDTH-1:0] ib_inst_1;
wire [`INST_ADDR_WIDTH-1:0] ib_pc_0;
wire [`INST_ADDR_WIDTH-1:0] ib_pc_1;
wire ib_pred_taken_0, ib_pred_taken_1;
wire [`INST_ADDR_WIDTH-1:0] ib_pred_target_0, ib_pred_target_1;
wire [`BP_GHR_BITS-1:0] ib_pred_hist_0, ib_pred_hist_1;

InstructionBuffer u_ibuf (
    .clk(clk),
    .rst_n(rst_n),
    .flush(global_flush),
    .in_valid(if_inst_valid),
    .in_inst_0(if_inst_0),
    .in_inst_1(if_inst_1),
    .in_pc_0(if_inst_addr_0),
    .in_pc_1(if_inst_addr_1),
    .in_pred_taken_0(if_pred_taken_0),
    .in_pred_target_0(if_pred_target_0),
    .in_pred_hist_0(if_pred_hist_0),
    .in_pred_taken_1(if_pred_taken_1),
    .in_pred_target_1(if_pred_target_1),
    .in_pred_hist_1(if_pred_hist_1),
    .out_ready(dispatch_ready),
    .out_valid(ib_valid),
    .out_inst_0(ib_inst_0),
    .out_inst_1(ib_inst_1),
    .out_pc_0(ib_pc_0),
    .out_pc_1(ib_pc_1),
    .out_pred_taken_0(ib_pred_taken_0),
    .out_pred_target_0(ib_pred_target_0),
    .out_pred_hist_0(ib_pred_hist_0),
    .out_pred_taken_1(ib_pred_taken_1),
    .out_pred_target_1(ib_pred_target_1),
    .out_pred_hist_1(ib_pred_hist_1),
    .stall_if(ib_stall_if)
);

// ----------------------------
// Pre-decode
// ----------------------------
wire [`IF_BATCH_SIZE-1:0] pre_valid;
wire [`INST_WIDTH-1:0] pre_inst_0;
wire [`INST_WIDTH-1:0] pre_inst_1;
wire [`INST_ADDR_WIDTH-1:0] pre_pc_0;
wire [`INST_ADDR_WIDTH-1:0] pre_pc_1;
wire [1:0] pre_fu_type_0;
wire [1:0] pre_fu_type_1;
wire [`REG_ADDR_WIDTH-1:0] pre_rs1_0;
wire [`REG_ADDR_WIDTH-1:0] pre_rs2_0;
wire [`REG_ADDR_WIDTH-1:0] pre_rd_0;
wire [`DATA_WIDTH-1:0] pre_imm_0;
wire pre_use_imm_0;
wire pre_rs1_is_fp_0;
wire pre_rs2_is_fp_0;
wire pre_rd_is_fp_0;
wire [`REG_ADDR_WIDTH-1:0] pre_rs1_1;
wire [`REG_ADDR_WIDTH-1:0] pre_rs2_1;
wire [`REG_ADDR_WIDTH-1:0] pre_rd_1;
wire [`DATA_WIDTH-1:0] pre_imm_1;
wire pre_use_imm_1;
wire pre_rs1_is_fp_1;
wire pre_rs2_is_fp_1;
wire pre_rd_is_fp_1;
wire pre_pred_taken_0, pre_pred_taken_1;
wire [`INST_ADDR_WIDTH-1:0] pre_pred_target_0, pre_pred_target_1;
wire [`BP_GHR_BITS-1:0] pre_pred_hist_0, pre_pred_hist_1;

PreDecode u_predecode (
    .clk(clk),
    .rst_n(rst_n),
    .stall(dispatch_pipe_stall),
    .in_inst_0(ib_inst_0),
    .in_inst_1(ib_inst_1),
    .in_pc_0(ib_pc_0),
    .in_pc_1(ib_pc_1),
    .in_inst_valid(dispatch_ready ? ib_valid : {`IF_BATCH_SIZE{1'b0}}),
    .in_pred_taken_0(ib_pred_taken_0),
    .in_pred_target_0(ib_pred_target_0),
    .in_pred_hist_0(ib_pred_hist_0),
    .in_pred_taken_1(ib_pred_taken_1),
    .in_pred_target_1(ib_pred_target_1),
    .in_pred_hist_1(ib_pred_hist_1),
    .out_inst_valid(pre_valid),
    .out_inst_0(pre_inst_0),
    .out_inst_1(pre_inst_1),
    .out_pc_0(pre_pc_0),
    .out_pc_1(pre_pc_1),
    .out_pred_taken_0(pre_pred_taken_0),
    .out_pred_target_0(pre_pred_target_0),
    .out_pred_hist_0(pre_pred_hist_0),
    .out_pred_taken_1(pre_pred_taken_1),
    .out_pred_target_1(pre_pred_target_1),
    .out_pred_hist_1(pre_pred_hist_1),
    .out_fu_type_0(pre_fu_type_0),
    .out_rs1_0(pre_rs1_0),
    .out_rs2_0(pre_rs2_0),
    .out_rd_0(pre_rd_0),
    .out_imm_0(pre_imm_0),
    .out_use_imm_0(pre_use_imm_0),
    .out_rs1_is_fp_0(pre_rs1_is_fp_0),
    .out_rs2_is_fp_0(pre_rs2_is_fp_0),
    .out_rd_is_fp_0(pre_rd_is_fp_0),
    .out_fu_type_1(pre_fu_type_1),
    .out_rs1_1(pre_rs1_1),
    .out_rs2_1(pre_rs2_1),
    .out_rd_1(pre_rd_1),
    .out_imm_1(pre_imm_1),
    .out_use_imm_1(pre_use_imm_1),
    .out_rs1_is_fp_1(pre_rs1_is_fp_1),
    .out_rs2_is_fp_1(pre_rs2_is_fp_1),
    .out_rd_is_fp_1(pre_rd_is_fp_1)
);

// ----------------------------
// Register rename
// ----------------------------
wire [`IF_BATCH_SIZE-1:0] rn_valid;
wire [`INST_WIDTH-1:0] rn_inst_0;
wire [`INST_WIDTH-1:0] rn_inst_1;
wire [1:0] rn_fu_type_0;
wire [1:0] rn_fu_type_1;
wire [`INST_ADDR_WIDTH-1:0] rn_pc_0;
wire [`INST_ADDR_WIDTH-1:0] rn_pc_1;
wire rn_pred_taken_0, rn_pred_taken_1;
wire [`INST_ADDR_WIDTH-1:0] rn_pred_target_0, rn_pred_target_1;
wire [`BP_GHR_BITS-1:0] rn_pred_hist_0, rn_pred_hist_1;
wire [`REG_ADDR_WIDTH-1:0] rn_rs1_0;
wire [`REG_ADDR_WIDTH-1:0] rn_rs2_0;
wire [`REG_ADDR_WIDTH-1:0] rn_rd_0;
wire [`DATA_WIDTH-1:0] rn_imm_0;
wire rn_use_imm_0;
wire rn_rs1_is_fp_0;
wire rn_rs2_is_fp_0;
wire rn_rd_is_fp_0;
wire [`ROB_IDX_WIDTH-1:0] rn_rob_idx_0;
wire rn_rob_idx_valid_0;
wire [`ROB_GEN_WIDTH-1:0] rn_rob_gen_0;
wire [`PREG_IDX_WIDTH-1:0] rn_rs1_preg_0;
wire rn_rs1_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] rn_rs2_preg_0;
wire rn_rs2_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] rn_rd_preg_0;
wire rn_rd_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] rn_old_rd_preg_0;
wire rn_old_rd_preg_valid_0;

wire [`REG_ADDR_WIDTH-1:0] rn_rs1_1;
wire [`REG_ADDR_WIDTH-1:0] rn_rs2_1;
wire [`REG_ADDR_WIDTH-1:0] rn_rd_1;
wire [`DATA_WIDTH-1:0] rn_imm_1;
wire rn_use_imm_1;
wire rn_rs1_is_fp_1;
wire rn_rs2_is_fp_1;
wire rn_rd_is_fp_1;
wire [`ROB_IDX_WIDTH-1:0] rn_rob_idx_1;
wire rn_rob_idx_valid_1;
wire [`ROB_GEN_WIDTH-1:0] rn_rob_gen_1;
wire [`PREG_IDX_WIDTH-1:0] rn_rs1_preg_1;
wire rn_rs1_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] rn_rs2_preg_1;
wire rn_rs2_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] rn_rd_preg_1;
wire rn_rd_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] rn_old_rd_preg_1;
wire rn_old_rd_preg_valid_1;

// Writeback wires from execute/issue
wire wb0_valid_exe;
wire wb1_valid_exe;
wire wb2_valid_exe;
wire [`ROB_IDX_WIDTH-1:0] wb0_rob_idx_exe;
wire [`ROB_IDX_WIDTH-1:0] wb1_rob_idx_exe;
wire [`ROB_IDX_WIDTH-1:0] wb2_rob_idx_exe;
wire [`ROB_GEN_WIDTH-1:0] wb0_rob_gen_exe;
wire [`ROB_GEN_WIDTH-1:0] wb1_rob_gen_exe;
wire [`ROB_GEN_WIDTH-1:0] wb2_rob_gen_exe;
wire [`DATA_WIDTH-1:0] wb0_value_exe;
wire [`DATA_WIDTH-1:0] wb1_value_exe;
wire [`DATA_WIDTH-1:0] wb2_value_exe;
wire wb0_exception_exe;
wire wb1_exception_exe;
wire wb2_exception_exe;
wire lsu_valid;
wire [31:0] lsu_addr;
wire [31:0] lsu_wdata;
wire [`MEM_OP_WIDTH-1:0] lsu_mem_op;
wire lsu_mem_is_load;
wire lsu_mem_unsigned;
wire [`ROB_IDX_WIDTH-1:0] lsu_rob_idx;
wire [`ROB_GEN_WIDTH-1:0] lsu_rob_gen;
wire [`PREG_IDX_WIDTH-1:0] lsu_rd_tag;
wire lsu_rd_is_fp;
wire lsu_busy;
wire lsu_wb_valid;
wire [`DATA_WIDTH-1:0] lsu_wb_value;
wire [`ROB_IDX_WIDTH-1:0] lsu_wb_rob_idx;
wire [`ROB_GEN_WIDTH-1:0] lsu_wb_rob_gen;
wire [`PREG_IDX_WIDTH-1:0] lsu_wb_dest_tag;
wire lsu_wb_dest_is_fp;
wire lsu_wb_exception;

// Dispatch queue -> issue payload
wire dq_stall;
wire [`IF_BATCH_SIZE-1:0] dq_valid;
wire [`INST_WIDTH-1:0] dq_inst_0;
wire [`INST_WIDTH-1:0] dq_inst_1;
wire [`INST_ADDR_WIDTH-1:0] dq_pc_0;
wire [`INST_ADDR_WIDTH-1:0] dq_pc_1;
wire dq_pred_taken_0;
wire dq_pred_taken_1;
wire [`INST_ADDR_WIDTH-1:0] dq_pred_target_0;
wire [`INST_ADDR_WIDTH-1:0] dq_pred_target_1;
wire [`BP_GHR_BITS-1:0] dq_pred_hist_0;
wire [`BP_GHR_BITS-1:0] dq_pred_hist_1;
wire [`REG_ADDR_WIDTH-1:0] dq_rs1_0;
wire [`REG_ADDR_WIDTH-1:0] dq_rs2_0;
wire [`REG_ADDR_WIDTH-1:0] dq_rd_0;
wire [`DATA_WIDTH-1:0] dq_imm_0;
wire dq_use_imm_0;
wire dq_rs1_is_fp_0;
wire dq_rs2_is_fp_0;
wire dq_rd_is_fp_0;
wire [`ROB_IDX_WIDTH-1:0] dq_rob_idx_0;
wire dq_rob_idx_valid_0;
wire [`ROB_GEN_WIDTH-1:0] dq_rob_gen_0;
wire [`PREG_IDX_WIDTH-1:0] dq_rs1_preg_0;
wire dq_rs1_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] dq_rs2_preg_0;
wire dq_rs2_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] dq_rd_preg_0;
wire dq_rd_preg_valid_0;
wire [`FU_DEC_WIDTH-1:0] dq_fu_sel_0;
wire [`ALU_OP_WIDTH-1:0] dq_int_op_0;
wire dq_int_is_sub_0;
wire dq_cmp_signed_0;
wire [`ALU_OP_WIDTH-1:0] dq_muldiv_op_0;
wire dq_mul_high_0;
wire dq_mul_signed_rs1_0;
wire dq_mul_signed_rs2_0;
wire dq_div_signed_0;
wire dq_div_is_rem_0;
wire [`BR_OP_WIDTH-1:0] dq_branch_op_0;
wire [`MEM_OP_WIDTH-1:0] dq_mem_op_0;
wire dq_mem_is_load_0;
wire dq_mem_unsigned_0;
wire [`CSR_OP_WIDTH-1:0] dq_csr_op_0;
wire [11:0] dq_csr_addr_0;
wire [`FP_OP_WIDTH-1:0] dq_fp_op_0;
wire dq_illegal_0;
wire [`REG_ADDR_WIDTH-1:0] dq_rs1_1;
wire [`REG_ADDR_WIDTH-1:0] dq_rs2_1;
wire [`REG_ADDR_WIDTH-1:0] dq_rd_1;
wire [`DATA_WIDTH-1:0] dq_imm_1;
wire dq_use_imm_1;
wire dq_rs1_is_fp_1;
wire dq_rs2_is_fp_1;
wire dq_rd_is_fp_1;
wire [`ROB_IDX_WIDTH-1:0] dq_rob_idx_1;
wire dq_rob_idx_valid_1;
wire [`ROB_GEN_WIDTH-1:0] dq_rob_gen_1;
wire [`PREG_IDX_WIDTH-1:0] dq_rs1_preg_1;
wire dq_rs1_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] dq_rs2_preg_1;
wire dq_rs2_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] dq_rd_preg_1;
wire dq_rd_preg_valid_1;
wire [`FU_DEC_WIDTH-1:0] dq_fu_sel_1;
wire [`ALU_OP_WIDTH-1:0] dq_int_op_1;
wire dq_int_is_sub_1;
wire dq_cmp_signed_1;
wire [`ALU_OP_WIDTH-1:0] dq_muldiv_op_1;
wire dq_mul_high_1;
wire dq_mul_signed_rs1_1;
wire dq_mul_signed_rs2_1;
wire dq_div_signed_1;
wire dq_div_is_rem_1;
wire [`BR_OP_WIDTH-1:0] dq_branch_op_1;
wire [`MEM_OP_WIDTH-1:0] dq_mem_op_1;
wire dq_mem_is_load_1;
wire dq_mem_unsigned_1;
wire [`CSR_OP_WIDTH-1:0] dq_csr_op_1;
wire [11:0] dq_csr_addr_1;
wire [`FP_OP_WIDTH-1:0] dq_fp_op_1;
wire dq_illegal_1;

// Live ready lookups from rename scoreboard
wire dq_rs1_ready_now_0;
wire dq_rs2_ready_now_0;
wire dq_rs1_ready_now_1;
wire dq_rs2_ready_now_1;

assign dispatch_pipe_stall = dq_stall;

RegRename u_regrename (
    .clk(clk),
    .rst_n(rst_n),
    .flush(global_flush),
    .flush_is_redirect(redirect_valid),
    .flush_rob_idx(redirect_valid ? redirect_rob_idx :
                               (commit_exc ? commit0_rob_idx : {`ROB_IDX_WIDTH{1'b0}})),
    .stall(dispatch_pipe_stall),
    .in_inst_valid(pre_valid & {`IF_BATCH_SIZE{~dispatch_pipe_stall}}),
    .in_pc_0(pre_pc_0),
    .in_pc_1(pre_pc_1),
    .in_inst_0(pre_inst_0),
    .in_inst_1(pre_inst_1),
    .in_fu_type_0(pre_fu_type_0),
    .in_fu_type_1(pre_fu_type_1),
    .in_pred_taken_0(pre_pred_taken_0),
    .in_pred_target_0(pre_pred_target_0),
    .in_pred_hist_0(pre_pred_hist_0),
    .in_pred_taken_1(pre_pred_taken_1),
    .in_pred_target_1(pre_pred_target_1),
    .in_pred_hist_1(pre_pred_hist_1),
    .in_rs1_0(pre_rs1_0),
    .in_rs2_0(pre_rs2_0),
    .in_rd_0(pre_rd_0),
    .in_imm_0(pre_imm_0),
    .in_use_imm_0(pre_use_imm_0),
    .in_rs1_is_fp_0(pre_rs1_is_fp_0),
    .in_rs2_is_fp_0(pre_rs2_is_fp_0),
    .in_rd_is_fp_0(pre_rd_is_fp_0),
    .in_rs1_1(pre_rs1_1),
    .in_rs2_1(pre_rs2_1),
    .in_rd_1(pre_rd_1),
    .in_imm_1(pre_imm_1),
    .in_use_imm_1(pre_use_imm_1),
    .in_rs1_is_fp_1(pre_rs1_is_fp_1),
    .in_rs2_is_fp_1(pre_rs2_is_fp_1),
    .in_rd_is_fp_1(pre_rd_is_fp_1),
    .wb0_valid(wb0_valid_exe),
    .wb0_rob_idx(wb0_rob_idx_exe),
    .wb0_rob_gen(wb0_rob_gen_exe),
    .wb0_value(wb0_value_exe),
    .wb0_exception(wb0_exception_exe),
    .wb1_valid(wb1_valid_exe),
    .wb1_rob_idx(wb1_rob_idx_exe),
    .wb1_rob_gen(wb1_rob_gen_exe),
    .wb1_value(wb1_value_exe),
    .wb1_exception(wb1_exception_exe),
    .wb2_valid(wb2_valid_exe),
    .wb2_rob_idx(wb2_rob_idx_exe),
    .wb2_rob_gen(wb2_rob_gen_exe),
    .wb2_value(wb2_value_exe),
    .wb2_exception(wb2_exception_exe),
    .out_inst_valid(rn_valid),
    .out_inst_0(rn_inst_0),
    .out_inst_1(rn_inst_1),
    .out_fu_type_0(rn_fu_type_0),
    .out_fu_type_1(rn_fu_type_1),
    .out_pc_0(rn_pc_0),
    .out_pc_1(rn_pc_1),
    .out_pred_taken_0(rn_pred_taken_0),
    .out_pred_target_0(rn_pred_target_0),
    .out_pred_hist_0(rn_pred_hist_0),
    .out_pred_taken_1(rn_pred_taken_1),
    .out_pred_target_1(rn_pred_target_1),
    .out_pred_hist_1(rn_pred_hist_1),
    .out_rs1_0(rn_rs1_0),
    .out_rs2_0(rn_rs2_0),
    .out_rd_0(rn_rd_0),
    .out_imm_0(rn_imm_0),
    .out_use_imm_0(rn_use_imm_0),
    .out_rs1_is_fp_0(rn_rs1_is_fp_0),
    .out_rs2_is_fp_0(rn_rs2_is_fp_0),
    .out_rd_is_fp_0(rn_rd_is_fp_0),
    .out_rob_idx_0(rn_rob_idx_0),
    .out_rob_idx_valid_0(rn_rob_idx_valid_0),
    .out_rob_gen_0(rn_rob_gen_0),
    .out_rs1_preg_0(rn_rs1_preg_0),
    .out_rs1_preg_valid_0(rn_rs1_preg_valid_0),
    .out_rs2_preg_0(rn_rs2_preg_0),
    .out_rs2_preg_valid_0(rn_rs2_preg_valid_0),
    .out_rd_preg_0(rn_rd_preg_0),
    .out_rd_preg_valid_0(rn_rd_preg_valid_0),
    .out_old_rd_preg_0(rn_old_rd_preg_0),
    .out_old_rd_preg_valid_0(rn_old_rd_preg_valid_0),
    .out_rs1_1(rn_rs1_1),
    .out_rs2_1(rn_rs2_1),
    .out_rd_1(rn_rd_1),
    .out_imm_1(rn_imm_1),
    .out_use_imm_1(rn_use_imm_1),
    .out_rs1_is_fp_1(rn_rs1_is_fp_1),
    .out_rs2_is_fp_1(rn_rs2_is_fp_1),
    .out_rd_is_fp_1(rn_rd_is_fp_1),
    .out_rob_idx_1(rn_rob_idx_1),
    .out_rob_idx_valid_1(rn_rob_idx_valid_1),
    .out_rob_gen_1(rn_rob_gen_1),
    .out_rs1_preg_1(rn_rs1_preg_1),
    .out_rs1_preg_valid_1(rn_rs1_preg_valid_1),
    .out_rs2_preg_1(rn_rs2_preg_1),
    .out_rs2_preg_valid_1(rn_rs2_preg_valid_1),
    .out_rd_preg_1(rn_rd_preg_1),
    .out_rd_preg_valid_1(rn_rd_preg_valid_1),
    .out_old_rd_preg_1(rn_old_rd_preg_1),
    .out_old_rd_preg_valid_1(rn_old_rd_preg_valid_1),
    .commit0_valid(commit0_valid),
    .commit0_rob_idx(commit0_rob_idx),
    .commit0_pc(commit0_pc),
    .commit0_has_dest(commit0_has_dest),
    .commit0_is_float(commit0_is_float),
    .commit0_arch_rd(commit0_arch_rd),
    .commit0_new_preg(commit0_new_preg),
    .commit0_old_preg(commit0_old_preg),
    .commit0_value(commit0_value),
    .commit0_exception(commit0_exception),
    .commit1_valid(commit1_valid),
    .commit1_rob_idx(commit1_rob_idx),
    .commit1_pc(commit1_pc),
    .commit1_has_dest(commit1_has_dest),
    .commit1_is_float(commit1_is_float),
    .commit1_arch_rd(commit1_arch_rd),
    .commit1_new_preg(commit1_new_preg),
    .commit1_old_preg(commit1_old_preg),
    .commit1_value(commit1_value),
    .commit1_exception(commit1_exception),
    .rob_empty(rob_empty),
    .rob_head(rob_head),
    .rename_stall(rn_stall),
    .sb_q0_is_fp(dq_rs1_is_fp_0),
    .sb_q0_tag(dq_rs1_preg_0),
    .sb_q0_ready(dq_rs1_ready_now_0),
    .sb_q1_is_fp(dq_rs2_is_fp_0),
    .sb_q1_tag(dq_rs2_preg_0),
    .sb_q1_ready(dq_rs2_ready_now_0),
    .sb_q2_is_fp(dq_rs1_is_fp_1),
    .sb_q2_tag(dq_rs1_preg_1),
    .sb_q2_ready(dq_rs1_ready_now_1),
    .sb_q3_is_fp(dq_rs2_is_fp_1),
    .sb_q3_tag(dq_rs2_preg_1),
    .sb_q3_ready(dq_rs2_ready_now_1)
);

// ----------------------------
// Post-decode
// ----------------------------
wire [`IF_BATCH_SIZE-1:0] post_valid;
wire [`INST_WIDTH-1:0] post_inst_0;
wire [`INST_WIDTH-1:0] post_inst_1;
wire [1:0] post_fu_type_0;
wire [1:0] post_fu_type_1;
wire [`REG_ADDR_WIDTH-1:0] post_rs1_0;
wire [`REG_ADDR_WIDTH-1:0] post_rs2_0;
wire [`REG_ADDR_WIDTH-1:0] post_rd_0;
wire [`DATA_WIDTH-1:0] post_imm_0;
wire post_use_imm_0;
wire post_rs1_is_fp_0;
wire post_rs2_is_fp_0;
wire post_rd_is_fp_0;
wire post_pred_taken_0, post_pred_taken_1;
wire [`INST_ADDR_WIDTH-1:0] post_pred_target_0, post_pred_target_1;
wire [`BP_GHR_BITS-1:0] post_pred_hist_0, post_pred_hist_1;
wire [`ROB_IDX_WIDTH-1:0] post_rob_idx_0;
wire post_rob_idx_valid_0;
wire [`ROB_GEN_WIDTH-1:0] post_rob_gen_0;
wire [`PREG_IDX_WIDTH-1:0] post_rs1_preg_0;
wire post_rs1_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] post_rs2_preg_0;
wire post_rs2_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] post_rd_preg_0;
wire post_rd_preg_valid_0;
wire [`PREG_IDX_WIDTH-1:0] post_old_rd_preg_0;
wire post_old_rd_preg_valid_0;

wire [`REG_ADDR_WIDTH-1:0] post_rs1_1;
wire [`REG_ADDR_WIDTH-1:0] post_rs2_1;
wire [`REG_ADDR_WIDTH-1:0] post_rd_1;
wire [`DATA_WIDTH-1:0] post_imm_1;
wire post_use_imm_1;
wire post_rs1_is_fp_1;
wire post_rs2_is_fp_1;
wire post_rd_is_fp_1;
wire [`ROB_IDX_WIDTH-1:0] post_rob_idx_1;
wire post_rob_idx_valid_1;
wire [`ROB_GEN_WIDTH-1:0] post_rob_gen_1;
wire [`INST_ADDR_WIDTH-1:0] post_pc_0;
wire [`INST_ADDR_WIDTH-1:0] post_pc_1;
wire [`PREG_IDX_WIDTH-1:0] post_rs1_preg_1;
wire post_rs1_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] post_rs2_preg_1;
wire post_rs2_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] post_rd_preg_1;
wire post_rd_preg_valid_1;
wire [`PREG_IDX_WIDTH-1:0] post_old_rd_preg_1;
wire post_old_rd_preg_valid_1;

wire [`FU_DEC_WIDTH-1:0] post_fu_sel_0;
wire [`FU_DEC_WIDTH-1:0] post_fu_sel_1;
wire [`ALU_OP_WIDTH-1:0] post_int_op_0;
wire [`ALU_OP_WIDTH-1:0] post_int_op_1;
wire post_int_is_sub_0;
wire post_int_is_sub_1;
wire post_cmp_signed_0;
wire post_cmp_signed_1;
wire [`ALU_OP_WIDTH-1:0] post_muldiv_op_0;
wire [`ALU_OP_WIDTH-1:0] post_muldiv_op_1;
wire post_mul_high_0;
wire post_mul_high_1;
wire post_mul_signed_rs1_0;
wire post_mul_signed_rs1_1;
wire post_mul_signed_rs2_0;
wire post_mul_signed_rs2_1;
wire post_div_signed_0;
wire post_div_signed_1;
wire post_div_is_rem_0;
wire post_div_is_rem_1;
wire [`BR_OP_WIDTH-1:0] post_branch_op_0;
wire [`BR_OP_WIDTH-1:0] post_branch_op_1;
wire [`MEM_OP_WIDTH-1:0] post_mem_op_0;
wire [`MEM_OP_WIDTH-1:0] post_mem_op_1;
wire post_mem_is_load_0;
wire post_mem_is_load_1;
wire post_mem_unsigned_0;
wire post_mem_unsigned_1;
wire [`CSR_OP_WIDTH-1:0] post_csr_op_0;
wire [`CSR_OP_WIDTH-1:0] post_csr_op_1;
wire [11:0] post_csr_addr_0;
wire [11:0] post_csr_addr_1;
wire [`FP_OP_WIDTH-1:0] post_fp_op_0;
wire [`FP_OP_WIDTH-1:0] post_fp_op_1;
wire post_illegal_0;
wire post_illegal_1;

PostDecode u_postdecode (
    .clk(clk),
    .rst_n(rst_n),
    .stall(dispatch_pipe_stall),
    .in_inst_valid(rn_valid),
    .in_inst_0(rn_inst_0),
    .in_inst_1(rn_inst_1),
    .in_fu_type_0(rn_fu_type_0),
    .in_fu_type_1(rn_fu_type_1),
    .in_pred_taken_0(rn_pred_taken_0),
    .in_pred_target_0(rn_pred_target_0),
    .in_pred_hist_0(rn_pred_hist_0),
    .in_pred_taken_1(rn_pred_taken_1),
    .in_pred_target_1(rn_pred_target_1),
    .in_pred_hist_1(rn_pred_hist_1),
    .in_pc_0(rn_pc_0),
    .in_pc_1(rn_pc_1),
    .in_rs1_0(rn_rs1_0),
    .in_rs2_0(rn_rs2_0),
    .in_rd_0(rn_rd_0),
    .in_imm_0(rn_imm_0),
    .in_use_imm_0(rn_use_imm_0),
    .in_rs1_is_fp_0(rn_rs1_is_fp_0),
    .in_rs2_is_fp_0(rn_rs2_is_fp_0),
    .in_rd_is_fp_0(rn_rd_is_fp_0),
    .in_rob_idx_0(rn_rob_idx_0),
    .in_rob_idx_valid_0(rn_rob_idx_valid_0),
    .in_rob_gen_0(rn_rob_gen_0),
    .in_rs1_preg_0(rn_rs1_preg_0),
    .in_rs1_preg_valid_0(rn_rs1_preg_valid_0),
    .in_rs2_preg_0(rn_rs2_preg_0),
    .in_rs2_preg_valid_0(rn_rs2_preg_valid_0),
    .in_rd_preg_0(rn_rd_preg_0),
    .in_rd_preg_valid_0(rn_rd_preg_valid_0),
    .in_old_rd_preg_0(rn_old_rd_preg_0),
    .in_old_rd_preg_valid_0(rn_old_rd_preg_valid_0),
    .in_rs1_1(rn_rs1_1),
    .in_rs2_1(rn_rs2_1),
    .in_rd_1(rn_rd_1),
    .in_imm_1(rn_imm_1),
    .in_use_imm_1(rn_use_imm_1),
    .in_rs1_is_fp_1(rn_rs1_is_fp_1),
    .in_rs2_is_fp_1(rn_rs2_is_fp_1),
    .in_rd_is_fp_1(rn_rd_is_fp_1),
    .in_rob_idx_1(rn_rob_idx_1),
    .in_rob_idx_valid_1(rn_rob_idx_valid_1),
    .in_rob_gen_1(rn_rob_gen_1),
    .in_rs1_preg_1(rn_rs1_preg_1),
    .in_rs1_preg_valid_1(rn_rs1_preg_valid_1),
    .in_rs2_preg_1(rn_rs2_preg_1),
    .in_rs2_preg_valid_1(rn_rs2_preg_valid_1),
    .in_rd_preg_1(rn_rd_preg_1),
    .in_rd_preg_valid_1(rn_rd_preg_valid_1),
    .in_old_rd_preg_1(rn_old_rd_preg_1),
    .in_old_rd_preg_valid_1(rn_old_rd_preg_valid_1),
    .out_inst_valid(post_valid),
    .out_inst_0(post_inst_0),
    .out_inst_1(post_inst_1),
    .out_fu_type_0(post_fu_type_0),
    .out_fu_type_1(post_fu_type_1),
    .out_pc_0(post_pc_0),
    .out_pc_1(post_pc_1),
    .out_pred_taken_0(post_pred_taken_0),
    .out_pred_target_0(post_pred_target_0),
    .out_pred_hist_0(post_pred_hist_0),
    .out_pred_taken_1(post_pred_taken_1),
    .out_pred_target_1(post_pred_target_1),
    .out_pred_hist_1(post_pred_hist_1),
    .out_rs1_0(post_rs1_0),
    .out_rs2_0(post_rs2_0),
    .out_rd_0(post_rd_0),
    .out_imm_0(post_imm_0),
    .out_use_imm_0(post_use_imm_0),
    .out_rs1_is_fp_0(post_rs1_is_fp_0),
    .out_rs2_is_fp_0(post_rs2_is_fp_0),
    .out_rd_is_fp_0(post_rd_is_fp_0),
    .out_rob_idx_0(post_rob_idx_0),
    .out_rob_idx_valid_0(post_rob_idx_valid_0),
    .out_rob_gen_0(post_rob_gen_0),
    .out_rs1_preg_0(post_rs1_preg_0),
    .out_rs1_preg_valid_0(post_rs1_preg_valid_0),
    .out_rs2_preg_0(post_rs2_preg_0),
    .out_rs2_preg_valid_0(post_rs2_preg_valid_0),
    .out_rd_preg_0(post_rd_preg_0),
    .out_rd_preg_valid_0(post_rd_preg_valid_0),
    .out_old_rd_preg_0(post_old_rd_preg_0),
    .out_old_rd_preg_valid_0(post_old_rd_preg_valid_0),
    .out_rs1_1(post_rs1_1),
    .out_rs2_1(post_rs2_1),
    .out_rd_1(post_rd_1),
    .out_imm_1(post_imm_1),
    .out_use_imm_1(post_use_imm_1),
    .out_rs1_is_fp_1(post_rs1_is_fp_1),
    .out_rs2_is_fp_1(post_rs2_is_fp_1),
    .out_rd_is_fp_1(post_rd_is_fp_1),
    .out_rob_idx_1(post_rob_idx_1),
    .out_rob_idx_valid_1(post_rob_idx_valid_1),
    .out_rob_gen_1(post_rob_gen_1),
    .out_rs1_preg_1(post_rs1_preg_1),
    .out_rs1_preg_valid_1(post_rs1_preg_valid_1),
    .out_rs2_preg_1(post_rs2_preg_1),
    .out_rs2_preg_valid_1(post_rs2_preg_valid_1),
    .out_rd_preg_1(post_rd_preg_1),
    .out_rd_preg_valid_1(post_rd_preg_valid_1),
    .out_old_rd_preg_1(post_old_rd_preg_1),
    .out_old_rd_preg_valid_1(post_old_rd_preg_valid_1),
    .out_fu_sel_0(post_fu_sel_0),
    .out_fu_sel_1(post_fu_sel_1),
    .out_int_op_0(post_int_op_0),
    .out_int_op_1(post_int_op_1),
    .out_int_is_sub_0(post_int_is_sub_0),
    .out_int_is_sub_1(post_int_is_sub_1),
    .out_cmp_signed_0(post_cmp_signed_0),
    .out_cmp_signed_1(post_cmp_signed_1),
    .out_muldiv_op_0(post_muldiv_op_0),
    .out_muldiv_op_1(post_muldiv_op_1),
    .out_mul_high_0(post_mul_high_0),
    .out_mul_high_1(post_mul_high_1),
    .out_mul_signed_rs1_0(post_mul_signed_rs1_0),
    .out_mul_signed_rs1_1(post_mul_signed_rs1_1),
    .out_mul_signed_rs2_0(post_mul_signed_rs2_0),
    .out_mul_signed_rs2_1(post_mul_signed_rs2_1),
    .out_div_signed_0(post_div_signed_0),
    .out_div_signed_1(post_div_signed_1),
    .out_div_is_rem_0(post_div_is_rem_0),
    .out_div_is_rem_1(post_div_is_rem_1),
    .out_branch_op_0(post_branch_op_0),
    .out_branch_op_1(post_branch_op_1),
    .out_mem_op_0(post_mem_op_0),
    .out_mem_op_1(post_mem_op_1),
    .out_mem_is_load_0(post_mem_is_load_0),
    .out_mem_is_load_1(post_mem_is_load_1),
    .out_mem_unsigned_0(post_mem_unsigned_0),
    .out_mem_unsigned_1(post_mem_unsigned_1),
    .out_csr_op_0(post_csr_op_0),
    .out_csr_op_1(post_csr_op_1),
    .out_csr_addr_0(post_csr_addr_0),
    .out_csr_addr_1(post_csr_addr_1),
    .out_fp_op_0(post_fp_op_0),
    .out_fp_op_1(post_fp_op_1),
    .out_illegal_0(post_illegal_0),
    .out_illegal_1(post_illegal_1)
);

DispatchQueue #(.DEPTH(8)) u_dispatchq (
    .clk(clk),
    .rst_n(rst_n),
    .flush(global_flush),
    .in_inst_valid(post_valid),
    .in_inst_0(post_inst_0),
    .in_inst_1(post_inst_1),
    .in_pc_0(post_pc_0),
    .in_pc_1(post_pc_1),
    .in_pred_taken_0(post_pred_taken_0),
    .in_pred_target_0(post_pred_target_0),
    .in_pred_hist_0(post_pred_hist_0),
    .in_pred_taken_1(post_pred_taken_1),
    .in_pred_target_1(post_pred_target_1),
    .in_pred_hist_1(post_pred_hist_1),
    .in_rs1_0(post_rs1_0),
    .in_rs2_0(post_rs2_0),
    .in_rd_0(post_rd_0),
    .in_imm_0(post_imm_0),
    .in_use_imm_0(post_use_imm_0),
    .in_rs1_is_fp_0(post_rs1_is_fp_0),
    .in_rs2_is_fp_0(post_rs2_is_fp_0),
    .in_rd_is_fp_0(post_rd_is_fp_0),
    .in_rob_idx_0(post_rob_idx_0),
    .in_rob_idx_valid_0(post_rob_idx_valid_0),
    .in_rob_gen_0(post_rob_gen_0),
    .in_rs1_preg_0(post_rs1_preg_0),
    .in_rs1_preg_valid_0(post_rs1_preg_valid_0),
    .in_rs2_preg_0(post_rs2_preg_0),
    .in_rs2_preg_valid_0(post_rs2_preg_valid_0),
    .in_rd_preg_0(post_rd_preg_0),
    .in_rd_preg_valid_0(post_rd_preg_valid_0),
    .in_fu_sel_0(post_fu_sel_0),
    .in_int_op_0(post_int_op_0),
    .in_int_is_sub_0(post_int_is_sub_0),
    .in_cmp_signed_0(post_cmp_signed_0),
    .in_muldiv_op_0(post_muldiv_op_0),
    .in_mul_high_0(post_mul_high_0),
    .in_mul_signed_rs1_0(post_mul_signed_rs1_0),
    .in_mul_signed_rs2_0(post_mul_signed_rs2_0),
    .in_div_signed_0(post_div_signed_0),
    .in_div_is_rem_0(post_div_is_rem_0),
    .in_branch_op_0(post_branch_op_0),
    .in_mem_op_0(post_mem_op_0),
    .in_mem_is_load_0(post_mem_is_load_0),
    .in_mem_unsigned_0(post_mem_unsigned_0),
    .in_csr_op_0(post_csr_op_0),
    .in_csr_addr_0(post_csr_addr_0),
    .in_fp_op_0(post_fp_op_0),
    .in_illegal_0(post_illegal_0),
    .in_rs1_1(post_rs1_1),
    .in_rs2_1(post_rs2_1),
    .in_rd_1(post_rd_1),
    .in_imm_1(post_imm_1),
    .in_use_imm_1(post_use_imm_1),
    .in_rs1_is_fp_1(post_rs1_is_fp_1),
    .in_rs2_is_fp_1(post_rs2_is_fp_1),
    .in_rd_is_fp_1(post_rd_is_fp_1),
    .in_rob_idx_1(post_rob_idx_1),
    .in_rob_idx_valid_1(post_rob_idx_valid_1),
    .in_rob_gen_1(post_rob_gen_1),
    .in_rs1_preg_1(post_rs1_preg_1),
    .in_rs1_preg_valid_1(post_rs1_preg_valid_1),
    .in_rs2_preg_1(post_rs2_preg_1),
    .in_rs2_preg_valid_1(post_rs2_preg_valid_1),
    .in_rd_preg_1(post_rd_preg_1),
    .in_rd_preg_valid_1(post_rd_preg_valid_1),
    .in_fu_sel_1(post_fu_sel_1),
    .in_int_op_1(post_int_op_1),
    .in_int_is_sub_1(post_int_is_sub_1),
    .in_cmp_signed_1(post_cmp_signed_1),
    .in_muldiv_op_1(post_muldiv_op_1),
    .in_mul_high_1(post_mul_high_1),
    .in_mul_signed_rs1_1(post_mul_signed_rs1_1),
    .in_mul_signed_rs2_1(post_mul_signed_rs2_1),
    .in_div_signed_1(post_div_signed_1),
    .in_div_is_rem_1(post_div_is_rem_1),
    .in_branch_op_1(post_branch_op_1),
    .in_mem_op_1(post_mem_op_1),
    .in_mem_is_load_1(post_mem_is_load_1),
    .in_mem_unsigned_1(post_mem_unsigned_1),
    .in_csr_op_1(post_csr_op_1),
    .in_csr_addr_1(post_csr_addr_1),
    .in_fp_op_1(post_fp_op_1),
    .in_illegal_1(post_illegal_1),
    .out_ready(!issue_stall),
    .out_inst_valid(dq_valid),
    .out_inst_0(dq_inst_0),
    .out_inst_1(dq_inst_1),
    .out_pc_0(dq_pc_0),
    .out_pc_1(dq_pc_1),
    .out_pred_taken_0(dq_pred_taken_0),
    .out_pred_target_0(dq_pred_target_0),
    .out_pred_hist_0(dq_pred_hist_0),
    .out_pred_taken_1(dq_pred_taken_1),
    .out_pred_target_1(dq_pred_target_1),
    .out_pred_hist_1(dq_pred_hist_1),
    .out_rs1_0(dq_rs1_0),
    .out_rs2_0(dq_rs2_0),
    .out_rd_0(dq_rd_0),
    .out_imm_0(dq_imm_0),
    .out_use_imm_0(dq_use_imm_0),
    .out_rs1_is_fp_0(dq_rs1_is_fp_0),
    .out_rs2_is_fp_0(dq_rs2_is_fp_0),
    .out_rd_is_fp_0(dq_rd_is_fp_0),
    .out_rob_idx_0(dq_rob_idx_0),
    .out_rob_idx_valid_0(dq_rob_idx_valid_0),
    .out_rob_gen_0(dq_rob_gen_0),
    .out_rs1_preg_0(dq_rs1_preg_0),
    .out_rs1_preg_valid_0(dq_rs1_preg_valid_0),
    .out_rs2_preg_0(dq_rs2_preg_0),
    .out_rs2_preg_valid_0(dq_rs2_preg_valid_0),
    .out_rd_preg_0(dq_rd_preg_0),
    .out_rd_preg_valid_0(dq_rd_preg_valid_0),
    .out_fu_sel_0(dq_fu_sel_0),
    .out_int_op_0(dq_int_op_0),
    .out_int_is_sub_0(dq_int_is_sub_0),
    .out_cmp_signed_0(dq_cmp_signed_0),
    .out_muldiv_op_0(dq_muldiv_op_0),
    .out_mul_high_0(dq_mul_high_0),
    .out_mul_signed_rs1_0(dq_mul_signed_rs1_0),
    .out_mul_signed_rs2_0(dq_mul_signed_rs2_0),
    .out_div_signed_0(dq_div_signed_0),
    .out_div_is_rem_0(dq_div_is_rem_0),
    .out_branch_op_0(dq_branch_op_0),
    .out_mem_op_0(dq_mem_op_0),
    .out_mem_is_load_0(dq_mem_is_load_0),
    .out_mem_unsigned_0(dq_mem_unsigned_0),
    .out_csr_op_0(dq_csr_op_0),
    .out_csr_addr_0(dq_csr_addr_0),
    .out_fp_op_0(dq_fp_op_0),
    .out_illegal_0(dq_illegal_0),
    .out_rs1_1(dq_rs1_1),
    .out_rs2_1(dq_rs2_1),
    .out_rd_1(dq_rd_1),
    .out_imm_1(dq_imm_1),
    .out_use_imm_1(dq_use_imm_1),
    .out_rs1_is_fp_1(dq_rs1_is_fp_1),
    .out_rs2_is_fp_1(dq_rs2_is_fp_1),
    .out_rd_is_fp_1(dq_rd_is_fp_1),
    .out_rob_idx_1(dq_rob_idx_1),
    .out_rob_idx_valid_1(dq_rob_idx_valid_1),
    .out_rob_gen_1(dq_rob_gen_1),
    .out_rs1_preg_1(dq_rs1_preg_1),
    .out_rs1_preg_valid_1(dq_rs1_preg_valid_1),
    .out_rs2_preg_1(dq_rs2_preg_1),
    .out_rs2_preg_valid_1(dq_rs2_preg_valid_1),
    .out_rd_preg_1(dq_rd_preg_1),
    .out_rd_preg_valid_1(dq_rd_preg_valid_1),
    .out_fu_sel_1(dq_fu_sel_1),
    .out_int_op_1(dq_int_op_1),
    .out_int_is_sub_1(dq_int_is_sub_1),
    .out_cmp_signed_1(dq_cmp_signed_1),
    .out_muldiv_op_1(dq_muldiv_op_1),
    .out_mul_high_1(dq_mul_high_1),
    .out_mul_signed_rs1_1(dq_mul_signed_rs1_1),
    .out_mul_signed_rs2_1(dq_mul_signed_rs2_1),
    .out_div_signed_1(dq_div_signed_1),
    .out_div_is_rem_1(dq_div_is_rem_1),
    .out_branch_op_1(dq_branch_op_1),
    .out_mem_op_1(dq_mem_op_1),
    .out_mem_is_load_1(dq_mem_is_load_1),
    .out_mem_unsigned_1(dq_mem_unsigned_1),
    .out_csr_op_1(dq_csr_op_1),
    .out_csr_addr_1(dq_csr_addr_1),
    .out_fp_op_1(dq_fp_op_1),
    .out_illegal_1(dq_illegal_1),
    .stall_upstream(dq_stall)
);

// ----------------------------
// Shared PRF instance
// ----------------------------
PhysicalRegFileShared u_prf (
    .clk(clk),
    .rst_n(rst_n),
    // Integer reads
    .i_rs1_addr0(dq_rs1_preg_0),
    .i_rs2_addr0(dq_rs2_preg_0),
    .i_rs1_addr1(dq_rs1_preg_1),
    .i_rs2_addr1(dq_rs2_preg_1),
    .i_rs1_data0(prf_int_rs1_data0),
    .i_rs2_data0(prf_int_rs2_data0),
    .i_rs1_data1(prf_int_rs1_data1),
    .i_rs2_data1(prf_int_rs2_data1),
    // Integer writes
    .i_rd0_addr(prf_i_wr_addr0),
    .i_rd0_data(prf_i_wr_data0),
    .i_rd0_we(prf_i_wr_we0),
    .i_rd1_addr(prf_i_wr_addr1),
    .i_rd1_data(prf_i_wr_data1),
    .i_rd1_we(prf_i_wr_we1),
    // FP reads
    .f_rs1_addr0(dq_rs1_preg_0),
    .f_rs2_addr0(dq_rs2_preg_0),
    .f_rs1_addr1(dq_rs1_preg_1),
    .f_rs2_addr1(dq_rs2_preg_1),
    .f_rs1_data0(prf_fp_rs1_data0),
    .f_rs2_data0(prf_fp_rs2_data0),
    .f_rs1_data1(prf_fp_rs1_data1),
    .f_rs2_data1(prf_fp_rs2_data1),
    // FP writes
    .f_rd0_addr(prf_f_wr_addr0),
    .f_rd0_data(prf_f_wr_data0),
    .f_rd0_we(prf_f_wr_we0),
    .f_rd1_addr(prf_f_wr_addr1),
    .f_rd1_data(prf_f_wr_data1),
    .f_rd1_we(prf_f_wr_we1)
);

// ----------------------------
// Issue / Execute (simplified)
// ----------------------------
IssueBuffer u_issue (
    .clk(clk),
    .rst_n(rst_n),
    .flush(global_flush),
    .flush_is_redirect(redirect_valid),
    .flush_rob_idx(redirect_rob_idx),
    .in_inst_valid(dq_valid),
    .in_pred_taken_0(dq_pred_taken_0),
    .in_pred_target_0(dq_pred_target_0),
    .in_pred_hist_0(dq_pred_hist_0),
    .in_pred_taken_1(dq_pred_taken_1),
    .in_pred_target_1(dq_pred_target_1),
    .in_pred_hist_1(dq_pred_hist_1),
    .in_inst_0(dq_inst_0),
    .in_inst_1(dq_inst_1),
    .in_pc_0(dq_pc_0),
    .in_pc_1(dq_pc_1),
    .in_rs1_0(dq_rs1_0),
    .in_rs2_0(dq_rs2_0),
    .in_rd_0(dq_rd_0),
    .in_imm_0(dq_imm_0),
    .in_use_imm_0(dq_use_imm_0),
    .in_rs1_is_fp_0(dq_rs1_is_fp_0),
    .in_rs2_is_fp_0(dq_rs2_is_fp_0),
    .in_rd_is_fp_0(dq_rd_is_fp_0),
    .in_rob_idx_0(dq_rob_idx_0),
    .in_rob_idx_valid_0(dq_rob_idx_valid_0),
    .in_rob_gen_0(dq_rob_gen_0),
    .in_rs1_preg_0(dq_rs1_preg_0),
    .in_rs1_preg_valid_0(dq_rs1_preg_valid_0),
    .in_rs1_ready_now_0(dq_rs1_ready_now_0),
    .in_rs2_preg_0(dq_rs2_preg_0),
    .in_rs2_preg_valid_0(dq_rs2_preg_valid_0),
    .in_rs2_ready_now_0(dq_rs2_ready_now_0),
    .in_rd_preg_0(dq_rd_preg_0),
    .in_rd_preg_valid_0(dq_rd_preg_valid_0),
    .in_fu_sel_0(dq_fu_sel_0),
    .in_int_op_0(dq_int_op_0),
    .in_int_is_sub_0(dq_int_is_sub_0),
    .in_cmp_signed_0(dq_cmp_signed_0),
    .in_muldiv_op_0(dq_muldiv_op_0),
    .in_mul_high_0(dq_mul_high_0),
    .in_mul_signed_rs1_0(dq_mul_signed_rs1_0),
    .in_mul_signed_rs2_0(dq_mul_signed_rs2_0),
    .in_div_signed_0(dq_div_signed_0),
    .in_div_is_rem_0(dq_div_is_rem_0),
    .in_branch_op_0(dq_branch_op_0),
    .in_mem_op_0(dq_mem_op_0),
    .in_mem_is_load_0(dq_mem_is_load_0),
    .in_mem_unsigned_0(dq_mem_unsigned_0),
    .in_csr_op_0(dq_csr_op_0),
    .in_csr_addr_0(dq_csr_addr_0),
    .in_fp_op_0(dq_fp_op_0),
    .in_illegal_0(dq_illegal_0),
    .in_rs1_1(dq_rs1_1),
    .in_rs2_1(dq_rs2_1),
    .in_rd_1(dq_rd_1),
    .in_imm_1(dq_imm_1),
    .in_use_imm_1(dq_use_imm_1),
    .in_rs1_is_fp_1(dq_rs1_is_fp_1),
    .in_rs2_is_fp_1(dq_rs2_is_fp_1),
    .in_rd_is_fp_1(dq_rd_is_fp_1),
    .in_rob_idx_1(dq_rob_idx_1),
    .in_rob_idx_valid_1(dq_rob_idx_valid_1),
    .in_rob_gen_1(dq_rob_gen_1),
    .in_rs1_preg_1(dq_rs1_preg_1),
    .in_rs1_preg_valid_1(dq_rs1_preg_valid_1),
    .in_rs1_ready_now_1(dq_rs1_ready_now_1),
    .in_rs2_preg_1(dq_rs2_preg_1),
    .in_rs2_preg_valid_1(dq_rs2_preg_valid_1),
    .in_rs2_ready_now_1(dq_rs2_ready_now_1),
    .in_rd_preg_1(dq_rd_preg_1),
    .in_rd_preg_valid_1(dq_rd_preg_valid_1),
    .in_fu_sel_1(dq_fu_sel_1),
    .in_int_op_1(dq_int_op_1),
    .in_int_is_sub_1(dq_int_is_sub_1),
    .in_cmp_signed_1(dq_cmp_signed_1),
    .in_muldiv_op_1(dq_muldiv_op_1),
    .in_mul_high_1(dq_mul_high_1),
    .in_mul_signed_rs1_1(dq_mul_signed_rs1_1),
    .in_mul_signed_rs2_1(dq_mul_signed_rs2_1),
    .in_div_signed_1(dq_div_signed_1),
    .in_div_is_rem_1(dq_div_is_rem_1),
    .in_branch_op_1(dq_branch_op_1),
    .in_mem_op_1(dq_mem_op_1),
    .in_mem_is_load_1(dq_mem_is_load_1),
    .in_mem_unsigned_1(dq_mem_unsigned_1),
    .in_csr_op_1(dq_csr_op_1),
    .in_csr_addr_1(dq_csr_addr_1),
    .in_fp_op_1(dq_fp_op_1),
    .in_illegal_1(dq_illegal_1),
    .rob_head(rob_head),
    .stall_dispatch(issue_stall),
    .wb0_valid(wb0_valid_exe),
    .wb0_rob_idx(wb0_rob_idx_exe),
    .wb0_rob_gen(wb0_rob_gen_exe),
    .wb0_value(wb0_value_exe),
    .wb0_exception(wb0_exception_exe),
    .wb1_valid(wb1_valid_exe),
    .wb1_rob_idx(wb1_rob_idx_exe),
    .wb1_rob_gen(wb1_rob_gen_exe),
    .wb1_value(wb1_value_exe),
    .wb1_exception(wb1_exception_exe),
    .wb2_valid(wb2_valid_exe),
    .wb2_rob_idx(wb2_rob_idx_exe),
    .wb2_rob_gen(wb2_rob_gen_exe),
    .wb2_value(wb2_value_exe),
    .wb2_exception(wb2_exception_exe),
    .lsu_valid(lsu_valid),
    .lsu_addr(lsu_addr),
    .lsu_wdata(lsu_wdata),
    .lsu_mem_op(lsu_mem_op),
    .lsu_mem_is_load(lsu_mem_is_load),
    .lsu_mem_unsigned(lsu_mem_unsigned),
    .lsu_rob_idx(lsu_rob_idx),
    .lsu_rob_gen(lsu_rob_gen),
    .lsu_rd_tag(lsu_rd_tag),
    .lsu_rd_is_fp(lsu_rd_is_fp),
    .lsu_busy(lsu_busy),
    .lsu_wb_valid(lsu_wb_valid),
    .lsu_wb_value(lsu_wb_value),
    .lsu_wb_rob_idx(lsu_wb_rob_idx),
    .lsu_wb_rob_gen(lsu_wb_rob_gen),
    .lsu_wb_dest_tag(lsu_wb_dest_tag),
    .lsu_wb_dest_is_fp(lsu_wb_dest_is_fp),
    .lsu_wb_exception(lsu_wb_exception),
    .redirect_valid(redirect_valid),
    .redirect_target(redirect_target),
    .redirect_rob_idx(redirect_rob_idx),
    .bp_update0_valid(issue_bp_update0_valid),
    .bp_update0_pc(issue_bp_update0_pc),
    .bp_update0_taken(issue_bp_update0_taken),
    .bp_update0_target(issue_bp_update0_target),
    .bp_update0_hist(issue_bp_update0_hist),
    .bp_update0_is_call(issue_bp_update0_is_call),
    .bp_update0_is_return(issue_bp_update0_is_return),
    .bp_update1_valid(issue_bp_update1_valid),
    .bp_update1_pc(issue_bp_update1_pc),
    .bp_update1_taken(issue_bp_update1_taken),
    .bp_update1_target(issue_bp_update1_target),
    .bp_update1_hist(issue_bp_update1_hist),
    .bp_update1_is_call(issue_bp_update1_is_call),
    .bp_update1_is_return(issue_bp_update1_is_return),
    .int_rs1_data0(prf_int_rs1_data0),
    .int_rs2_data0(prf_int_rs2_data0),
    .int_rs1_data1(prf_int_rs1_data1),
    .int_rs2_data1(prf_int_rs2_data1),
    .fp_rs1_data0(prf_fp_rs1_data0),
    .fp_rs2_data0(prf_fp_rs2_data0),
    .fp_rs1_data1(prf_fp_rs1_data1),
    .fp_rs2_data1(prf_fp_rs2_data1),
    .i_wr_addr0(prf_i_wr_addr0),
    .i_wr_data0(prf_i_wr_data0),
    .i_wr_we0(prf_i_wr_we0),
    .i_wr_addr1(prf_i_wr_addr1),
    .i_wr_data1(prf_i_wr_data1),
    .i_wr_we1(prf_i_wr_we1),
    .f_wr_addr0(prf_f_wr_addr0),
    .f_wr_data0(prf_f_wr_data0),
    .f_wr_we0(prf_f_wr_we0),
    .f_wr_addr1(prf_f_wr_addr1),
    .f_wr_data1(prf_f_wr_data1),
    .f_wr_we1(prf_f_wr_we1)
);

// ----------------------------
// LSU (D-Cache + TLB)
// ----------------------------
LSU u_lsu (
    .clk(clk),
    .rst_n(rst_n),
    .flush(global_flush),
    .valid_in(lsu_valid),
    .addr(lsu_addr),
    .wdata(lsu_wdata),
    .mem_op(lsu_mem_op),
    .mem_is_load(lsu_mem_is_load),
    .mem_unsigned(lsu_mem_unsigned),
    .rob_idx_in(lsu_rob_idx),
    .rob_gen_in(lsu_rob_gen),
    .rd_tag_in(lsu_rd_tag),
    .rd_is_fp_in(lsu_rd_is_fp),
    .busy(lsu_busy),
    .wb_valid(lsu_wb_valid),
    .wb_value(lsu_wb_value),
    .wb_rob_idx(lsu_wb_rob_idx),
    .wb_rob_gen(lsu_wb_rob_gen),
    .wb_dest_tag(lsu_wb_dest_tag),
    .wb_dest_is_fp(lsu_wb_dest_is_fp),
    .wb_exception(lsu_wb_exception),
    .mem_req(d_mem_req),
    .mem_we(d_mem_we),
    .mem_addr(d_mem_addr),
    .mem_wdata(d_mem_wdata),
    .mem_rdata(d_mem_rdata),
    .mem_ready(d_mem_ready)
);

endmodule

// Bundle-level dispatch FIFO between post-decode and issue.
module DispatchQueue #(
    parameter DEPTH = 8
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        flush,

    input  wire [`IF_BATCH_SIZE-1:0]   in_inst_valid,
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
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs1_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs2_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rd_0,
    input  wire [`DATA_WIDTH-1:0]      in_imm_0,
    input  wire                        in_use_imm_0,
    input  wire                        in_rs1_is_fp_0,
    input  wire                        in_rs2_is_fp_0,
    input  wire                        in_rd_is_fp_0,
    input  wire [`ROB_IDX_WIDTH-1:0]   in_rob_idx_0,
    input  wire                        in_rob_idx_valid_0,
    input  wire [`ROB_GEN_WIDTH-1:0]   in_rob_gen_0,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs1_preg_0,
    input  wire                        in_rs1_preg_valid_0,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs2_preg_0,
    input  wire                        in_rs2_preg_valid_0,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rd_preg_0,
    input  wire                        in_rd_preg_valid_0,
    input  wire [`FU_DEC_WIDTH-1:0]    in_fu_sel_0,
    input  wire [`ALU_OP_WIDTH-1:0]    in_int_op_0,
    input  wire                        in_int_is_sub_0,
    input  wire                        in_cmp_signed_0,
    input  wire [`ALU_OP_WIDTH-1:0]    in_muldiv_op_0,
    input  wire                        in_mul_high_0,
    input  wire                        in_mul_signed_rs1_0,
    input  wire                        in_mul_signed_rs2_0,
    input  wire                        in_div_signed_0,
    input  wire                        in_div_is_rem_0,
    input  wire [`BR_OP_WIDTH-1:0]     in_branch_op_0,
    input  wire [`MEM_OP_WIDTH-1:0]    in_mem_op_0,
    input  wire                        in_mem_is_load_0,
    input  wire                        in_mem_unsigned_0,
    input  wire [`CSR_OP_WIDTH-1:0]    in_csr_op_0,
    input  wire [11:0]                 in_csr_addr_0,
    input  wire [`FP_OP_WIDTH-1:0]     in_fp_op_0,
    input  wire                        in_illegal_0,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs1_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rs2_1,
    input  wire [`REG_ADDR_WIDTH-1:0]  in_rd_1,
    input  wire [`DATA_WIDTH-1:0]      in_imm_1,
    input  wire                        in_use_imm_1,
    input  wire                        in_rs1_is_fp_1,
    input  wire                        in_rs2_is_fp_1,
    input  wire                        in_rd_is_fp_1,
    input  wire [`ROB_IDX_WIDTH-1:0]   in_rob_idx_1,
    input  wire                        in_rob_idx_valid_1,
    input  wire [`ROB_GEN_WIDTH-1:0]   in_rob_gen_1,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs1_preg_1,
    input  wire                        in_rs1_preg_valid_1,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rs2_preg_1,
    input  wire                        in_rs2_preg_valid_1,
    input  wire [`PREG_IDX_WIDTH-1:0]  in_rd_preg_1,
    input  wire                        in_rd_preg_valid_1,
    input  wire [`FU_DEC_WIDTH-1:0]    in_fu_sel_1,
    input  wire [`ALU_OP_WIDTH-1:0]    in_int_op_1,
    input  wire                        in_int_is_sub_1,
    input  wire                        in_cmp_signed_1,
    input  wire [`ALU_OP_WIDTH-1:0]    in_muldiv_op_1,
    input  wire                        in_mul_high_1,
    input  wire                        in_mul_signed_rs1_1,
    input  wire                        in_mul_signed_rs2_1,
    input  wire                        in_div_signed_1,
    input  wire                        in_div_is_rem_1,
    input  wire [`BR_OP_WIDTH-1:0]     in_branch_op_1,
    input  wire [`MEM_OP_WIDTH-1:0]    in_mem_op_1,
    input  wire                        in_mem_is_load_1,
    input  wire                        in_mem_unsigned_1,
    input  wire [`CSR_OP_WIDTH-1:0]    in_csr_op_1,
    input  wire [11:0]                 in_csr_addr_1,
    input  wire [`FP_OP_WIDTH-1:0]     in_fp_op_1,
    input  wire                        in_illegal_1,

    input  wire                        out_ready,

    output wire [`IF_BATCH_SIZE-1:0]   out_inst_valid,
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
    output wire [`REG_ADDR_WIDTH-1:0]  out_rs1_0,
    output wire [`REG_ADDR_WIDTH-1:0]  out_rs2_0,
    output wire [`REG_ADDR_WIDTH-1:0]  out_rd_0,
    output wire [`DATA_WIDTH-1:0]      out_imm_0,
    output wire                        out_use_imm_0,
    output wire                        out_rs1_is_fp_0,
    output wire                        out_rs2_is_fp_0,
    output wire                        out_rd_is_fp_0,
    output wire [`ROB_IDX_WIDTH-1:0]   out_rob_idx_0,
    output wire                        out_rob_idx_valid_0,
    output wire [`ROB_GEN_WIDTH-1:0]   out_rob_gen_0,
    output wire [`PREG_IDX_WIDTH-1:0]  out_rs1_preg_0,
    output wire                        out_rs1_preg_valid_0,
    output wire [`PREG_IDX_WIDTH-1:0]  out_rs2_preg_0,
    output wire                        out_rs2_preg_valid_0,
    output wire [`PREG_IDX_WIDTH-1:0]  out_rd_preg_0,
    output wire                        out_rd_preg_valid_0,
    output wire [`FU_DEC_WIDTH-1:0]    out_fu_sel_0,
    output wire [`ALU_OP_WIDTH-1:0]    out_int_op_0,
    output wire                        out_int_is_sub_0,
    output wire                        out_cmp_signed_0,
    output wire [`ALU_OP_WIDTH-1:0]    out_muldiv_op_0,
    output wire                        out_mul_high_0,
    output wire                        out_mul_signed_rs1_0,
    output wire                        out_mul_signed_rs2_0,
    output wire                        out_div_signed_0,
    output wire                        out_div_is_rem_0,
    output wire [`BR_OP_WIDTH-1:0]     out_branch_op_0,
    output wire [`MEM_OP_WIDTH-1:0]    out_mem_op_0,
    output wire                        out_mem_is_load_0,
    output wire                        out_mem_unsigned_0,
    output wire [`CSR_OP_WIDTH-1:0]    out_csr_op_0,
    output wire [11:0]                 out_csr_addr_0,
    output wire [`FP_OP_WIDTH-1:0]     out_fp_op_0,
    output wire                        out_illegal_0,
    output wire [`REG_ADDR_WIDTH-1:0]  out_rs1_1,
    output wire [`REG_ADDR_WIDTH-1:0]  out_rs2_1,
    output wire [`REG_ADDR_WIDTH-1:0]  out_rd_1,
    output wire [`DATA_WIDTH-1:0]      out_imm_1,
    output wire                        out_use_imm_1,
    output wire                        out_rs1_is_fp_1,
    output wire                        out_rs2_is_fp_1,
    output wire                        out_rd_is_fp_1,
    output wire [`ROB_IDX_WIDTH-1:0]   out_rob_idx_1,
    output wire                        out_rob_idx_valid_1,
    output wire [`ROB_GEN_WIDTH-1:0]   out_rob_gen_1,
    output wire [`PREG_IDX_WIDTH-1:0]  out_rs1_preg_1,
    output wire                        out_rs1_preg_valid_1,
    output wire [`PREG_IDX_WIDTH-1:0]  out_rs2_preg_1,
    output wire                        out_rs2_preg_valid_1,
    output wire [`PREG_IDX_WIDTH-1:0]  out_rd_preg_1,
    output wire                        out_rd_preg_valid_1,
    output wire [`FU_DEC_WIDTH-1:0]    out_fu_sel_1,
    output wire [`ALU_OP_WIDTH-1:0]    out_int_op_1,
    output wire                        out_int_is_sub_1,
    output wire                        out_cmp_signed_1,
    output wire [`ALU_OP_WIDTH-1:0]    out_muldiv_op_1,
    output wire                        out_mul_high_1,
    output wire                        out_mul_signed_rs1_1,
    output wire                        out_mul_signed_rs2_1,
    output wire                        out_div_signed_1,
    output wire                        out_div_is_rem_1,
    output wire [`BR_OP_WIDTH-1:0]     out_branch_op_1,
    output wire [`MEM_OP_WIDTH-1:0]    out_mem_op_1,
    output wire                        out_mem_is_load_1,
    output wire                        out_mem_unsigned_1,
    output wire [`CSR_OP_WIDTH-1:0]    out_csr_op_1,
    output wire [11:0]                 out_csr_addr_1,
    output wire [`FP_OP_WIDTH-1:0]     out_fp_op_1,
    output wire                        out_illegal_1,

    output wire                        stall_upstream
);
    localparam PTR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH);

    reg [`IF_BATCH_SIZE-1:0]   q_inst_valid       [0:DEPTH-1];
    reg [`INST_WIDTH-1:0]      q_inst_0           [0:DEPTH-1];
    reg [`INST_WIDTH-1:0]      q_inst_1           [0:DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] q_pc_0             [0:DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] q_pc_1             [0:DEPTH-1];
    reg                        q_pred_taken_0     [0:DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] q_pred_target_0    [0:DEPTH-1];
    reg [`BP_GHR_BITS-1:0]     q_pred_hist_0      [0:DEPTH-1];
    reg                        q_pred_taken_1     [0:DEPTH-1];
    reg [`INST_ADDR_WIDTH-1:0] q_pred_target_1    [0:DEPTH-1];
    reg [`BP_GHR_BITS-1:0]     q_pred_hist_1      [0:DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  q_rs1_0            [0:DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  q_rs2_0            [0:DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  q_rd_0             [0:DEPTH-1];
    reg [`DATA_WIDTH-1:0]      q_imm_0            [0:DEPTH-1];
    reg                        q_use_imm_0        [0:DEPTH-1];
    reg                        q_rs1_is_fp_0      [0:DEPTH-1];
    reg                        q_rs2_is_fp_0      [0:DEPTH-1];
    reg                        q_rd_is_fp_0       [0:DEPTH-1];
    reg [`ROB_IDX_WIDTH-1:0]   q_rob_idx_0        [0:DEPTH-1];
    reg                        q_rob_idx_valid_0  [0:DEPTH-1];
    reg [`ROB_GEN_WIDTH-1:0]   q_rob_gen_0        [0:DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  q_rs1_preg_0       [0:DEPTH-1];
    reg                        q_rs1_preg_valid_0 [0:DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  q_rs2_preg_0       [0:DEPTH-1];
    reg                        q_rs2_preg_valid_0 [0:DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  q_rd_preg_0        [0:DEPTH-1];
    reg                        q_rd_preg_valid_0  [0:DEPTH-1];
    reg [`FU_DEC_WIDTH-1:0]    q_fu_sel_0         [0:DEPTH-1];
    reg [`ALU_OP_WIDTH-1:0]    q_int_op_0         [0:DEPTH-1];
    reg                        q_int_is_sub_0     [0:DEPTH-1];
    reg                        q_cmp_signed_0     [0:DEPTH-1];
    reg [`ALU_OP_WIDTH-1:0]    q_muldiv_op_0      [0:DEPTH-1];
    reg                        q_mul_high_0       [0:DEPTH-1];
    reg                        q_mul_signed_rs1_0 [0:DEPTH-1];
    reg                        q_mul_signed_rs2_0 [0:DEPTH-1];
    reg                        q_div_signed_0     [0:DEPTH-1];
    reg                        q_div_is_rem_0     [0:DEPTH-1];
    reg [`BR_OP_WIDTH-1:0]     q_branch_op_0      [0:DEPTH-1];
    reg [`MEM_OP_WIDTH-1:0]    q_mem_op_0         [0:DEPTH-1];
    reg                        q_mem_is_load_0    [0:DEPTH-1];
    reg                        q_mem_unsigned_0   [0:DEPTH-1];
    reg [`CSR_OP_WIDTH-1:0]    q_csr_op_0         [0:DEPTH-1];
    reg [11:0]                 q_csr_addr_0       [0:DEPTH-1];
    reg [`FP_OP_WIDTH-1:0]     q_fp_op_0          [0:DEPTH-1];
    reg                        q_illegal_0        [0:DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  q_rs1_1            [0:DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  q_rs2_1            [0:DEPTH-1];
    reg [`REG_ADDR_WIDTH-1:0]  q_rd_1             [0:DEPTH-1];
    reg [`DATA_WIDTH-1:0]      q_imm_1            [0:DEPTH-1];
    reg                        q_use_imm_1        [0:DEPTH-1];
    reg                        q_rs1_is_fp_1      [0:DEPTH-1];
    reg                        q_rs2_is_fp_1      [0:DEPTH-1];
    reg                        q_rd_is_fp_1       [0:DEPTH-1];
    reg [`ROB_IDX_WIDTH-1:0]   q_rob_idx_1        [0:DEPTH-1];
    reg                        q_rob_idx_valid_1  [0:DEPTH-1];
    reg [`ROB_GEN_WIDTH-1:0]   q_rob_gen_1        [0:DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  q_rs1_preg_1       [0:DEPTH-1];
    reg                        q_rs1_preg_valid_1 [0:DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  q_rs2_preg_1       [0:DEPTH-1];
    reg                        q_rs2_preg_valid_1 [0:DEPTH-1];
    reg [`PREG_IDX_WIDTH-1:0]  q_rd_preg_1        [0:DEPTH-1];
    reg                        q_rd_preg_valid_1  [0:DEPTH-1];
    reg [`FU_DEC_WIDTH-1:0]    q_fu_sel_1         [0:DEPTH-1];
    reg [`ALU_OP_WIDTH-1:0]    q_int_op_1         [0:DEPTH-1];
    reg                        q_int_is_sub_1     [0:DEPTH-1];
    reg                        q_cmp_signed_1     [0:DEPTH-1];
    reg [`ALU_OP_WIDTH-1:0]    q_muldiv_op_1      [0:DEPTH-1];
    reg                        q_mul_high_1       [0:DEPTH-1];
    reg                        q_mul_signed_rs1_1 [0:DEPTH-1];
    reg                        q_mul_signed_rs2_1 [0:DEPTH-1];
    reg                        q_div_signed_1     [0:DEPTH-1];
    reg                        q_div_is_rem_1     [0:DEPTH-1];
    reg [`BR_OP_WIDTH-1:0]     q_branch_op_1      [0:DEPTH-1];
    reg [`MEM_OP_WIDTH-1:0]    q_mem_op_1         [0:DEPTH-1];
    reg                        q_mem_is_load_1    [0:DEPTH-1];
    reg                        q_mem_unsigned_1   [0:DEPTH-1];
    reg [`CSR_OP_WIDTH-1:0]    q_csr_op_1         [0:DEPTH-1];
    reg [11:0]                 q_csr_addr_1       [0:DEPTH-1];
    reg [`FP_OP_WIDTH-1:0]     q_fp_op_1          [0:DEPTH-1];
    reg                        q_illegal_1        [0:DEPTH-1];

    reg [PTR_W-1:0] head;
    reg [PTR_W-1:0] tail;
    reg [PTR_W:0]   count;

    wire in_fire = in_inst_valid[0] | in_inst_valid[1];
    wire deq_fire = out_ready && (count != 0);
    wire can_enq = !in_fire || (count < DEPTH) || ((count == DEPTH) && deq_fire);
    wire enq_fire = in_fire && can_enq;
    assign stall_upstream = in_fire && !can_enq;

    function [PTR_W-1:0] ptr_inc;
        input [PTR_W-1:0] ptr;
        begin
            ptr_inc = (ptr == DEPTH-1) ? {PTR_W{1'b0}} : ptr + 1'b1;
        end
    endfunction

    assign out_inst_valid = (count != 0) ? q_inst_valid[head] : {`IF_BATCH_SIZE{1'b0}};
    assign out_inst_0 = (count != 0) ? q_inst_0[head] : {`INST_WIDTH{1'b0}};
    assign out_inst_1 = (count != 0) ? q_inst_1[head] : {`INST_WIDTH{1'b0}};
    assign out_pc_0 = (count != 0) ? q_pc_0[head] : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pc_1 = (count != 0) ? q_pc_1[head] : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pred_taken_0 = (count != 0) ? q_pred_taken_0[head] : 1'b0;
    assign out_pred_target_0 = (count != 0) ? q_pred_target_0[head] : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pred_hist_0 = (count != 0) ? q_pred_hist_0[head] : {`BP_GHR_BITS{1'b0}};
    assign out_pred_taken_1 = (count != 0) ? q_pred_taken_1[head] : 1'b0;
    assign out_pred_target_1 = (count != 0) ? q_pred_target_1[head] : {`INST_ADDR_WIDTH{1'b0}};
    assign out_pred_hist_1 = (count != 0) ? q_pred_hist_1[head] : {`BP_GHR_BITS{1'b0}};
    assign out_rs1_0 = (count != 0) ? q_rs1_0[head] : {`REG_ADDR_WIDTH{1'b0}};
    assign out_rs2_0 = (count != 0) ? q_rs2_0[head] : {`REG_ADDR_WIDTH{1'b0}};
    assign out_rd_0 = (count != 0) ? q_rd_0[head] : {`REG_ADDR_WIDTH{1'b0}};
    assign out_imm_0 = (count != 0) ? q_imm_0[head] : {`DATA_WIDTH{1'b0}};
    assign out_use_imm_0 = (count != 0) ? q_use_imm_0[head] : 1'b0;
    assign out_rs1_is_fp_0 = (count != 0) ? q_rs1_is_fp_0[head] : 1'b0;
    assign out_rs2_is_fp_0 = (count != 0) ? q_rs2_is_fp_0[head] : 1'b0;
    assign out_rd_is_fp_0 = (count != 0) ? q_rd_is_fp_0[head] : 1'b0;
    assign out_rob_idx_0 = (count != 0) ? q_rob_idx_0[head] : {`ROB_IDX_WIDTH{1'b0}};
    assign out_rob_idx_valid_0 = (count != 0) ? q_rob_idx_valid_0[head] : 1'b0;
    assign out_rob_gen_0 = (count != 0) ? q_rob_gen_0[head] : {`ROB_GEN_WIDTH{1'b0}};
    assign out_rs1_preg_0 = (count != 0) ? q_rs1_preg_0[head] : {`PREG_IDX_WIDTH{1'b0}};
    assign out_rs1_preg_valid_0 = (count != 0) ? q_rs1_preg_valid_0[head] : 1'b0;
    assign out_rs2_preg_0 = (count != 0) ? q_rs2_preg_0[head] : {`PREG_IDX_WIDTH{1'b0}};
    assign out_rs2_preg_valid_0 = (count != 0) ? q_rs2_preg_valid_0[head] : 1'b0;
    assign out_rd_preg_0 = (count != 0) ? q_rd_preg_0[head] : {`PREG_IDX_WIDTH{1'b0}};
    assign out_rd_preg_valid_0 = (count != 0) ? q_rd_preg_valid_0[head] : 1'b0;
    assign out_fu_sel_0 = (count != 0) ? q_fu_sel_0[head] : `FU_DEC_DUMMY;
    assign out_int_op_0 = (count != 0) ? q_int_op_0[head] : {`ALU_OP_WIDTH{1'b0}};
    assign out_int_is_sub_0 = (count != 0) ? q_int_is_sub_0[head] : 1'b0;
    assign out_cmp_signed_0 = (count != 0) ? q_cmp_signed_0[head] : 1'b0;
    assign out_muldiv_op_0 = (count != 0) ? q_muldiv_op_0[head] : {`ALU_OP_WIDTH{1'b0}};
    assign out_mul_high_0 = (count != 0) ? q_mul_high_0[head] : 1'b0;
    assign out_mul_signed_rs1_0 = (count != 0) ? q_mul_signed_rs1_0[head] : 1'b0;
    assign out_mul_signed_rs2_0 = (count != 0) ? q_mul_signed_rs2_0[head] : 1'b0;
    assign out_div_signed_0 = (count != 0) ? q_div_signed_0[head] : 1'b0;
    assign out_div_is_rem_0 = (count != 0) ? q_div_is_rem_0[head] : 1'b0;
    assign out_branch_op_0 = (count != 0) ? q_branch_op_0[head] : `BR_OP_NONE;
    assign out_mem_op_0 = (count != 0) ? q_mem_op_0[head] : {`MEM_OP_WIDTH{1'b0}};
    assign out_mem_is_load_0 = (count != 0) ? q_mem_is_load_0[head] : 1'b0;
    assign out_mem_unsigned_0 = (count != 0) ? q_mem_unsigned_0[head] : 1'b0;
    assign out_csr_op_0 = (count != 0) ? q_csr_op_0[head] : `CSR_OP_NONE;
    assign out_csr_addr_0 = (count != 0) ? q_csr_addr_0[head] : 12'b0;
    assign out_fp_op_0 = (count != 0) ? q_fp_op_0[head] : `FP_OP_DUMMY;
    assign out_illegal_0 = (count != 0) ? q_illegal_0[head] : 1'b0;
    assign out_rs1_1 = (count != 0) ? q_rs1_1[head] : {`REG_ADDR_WIDTH{1'b0}};
    assign out_rs2_1 = (count != 0) ? q_rs2_1[head] : {`REG_ADDR_WIDTH{1'b0}};
    assign out_rd_1 = (count != 0) ? q_rd_1[head] : {`REG_ADDR_WIDTH{1'b0}};
    assign out_imm_1 = (count != 0) ? q_imm_1[head] : {`DATA_WIDTH{1'b0}};
    assign out_use_imm_1 = (count != 0) ? q_use_imm_1[head] : 1'b0;
    assign out_rs1_is_fp_1 = (count != 0) ? q_rs1_is_fp_1[head] : 1'b0;
    assign out_rs2_is_fp_1 = (count != 0) ? q_rs2_is_fp_1[head] : 1'b0;
    assign out_rd_is_fp_1 = (count != 0) ? q_rd_is_fp_1[head] : 1'b0;
    assign out_rob_idx_1 = (count != 0) ? q_rob_idx_1[head] : {`ROB_IDX_WIDTH{1'b0}};
    assign out_rob_idx_valid_1 = (count != 0) ? q_rob_idx_valid_1[head] : 1'b0;
    assign out_rob_gen_1 = (count != 0) ? q_rob_gen_1[head] : {`ROB_GEN_WIDTH{1'b0}};
    assign out_rs1_preg_1 = (count != 0) ? q_rs1_preg_1[head] : {`PREG_IDX_WIDTH{1'b0}};
    assign out_rs1_preg_valid_1 = (count != 0) ? q_rs1_preg_valid_1[head] : 1'b0;
    assign out_rs2_preg_1 = (count != 0) ? q_rs2_preg_1[head] : {`PREG_IDX_WIDTH{1'b0}};
    assign out_rs2_preg_valid_1 = (count != 0) ? q_rs2_preg_valid_1[head] : 1'b0;
    assign out_rd_preg_1 = (count != 0) ? q_rd_preg_1[head] : {`PREG_IDX_WIDTH{1'b0}};
    assign out_rd_preg_valid_1 = (count != 0) ? q_rd_preg_valid_1[head] : 1'b0;
    assign out_fu_sel_1 = (count != 0) ? q_fu_sel_1[head] : `FU_DEC_DUMMY;
    assign out_int_op_1 = (count != 0) ? q_int_op_1[head] : {`ALU_OP_WIDTH{1'b0}};
    assign out_int_is_sub_1 = (count != 0) ? q_int_is_sub_1[head] : 1'b0;
    assign out_cmp_signed_1 = (count != 0) ? q_cmp_signed_1[head] : 1'b0;
    assign out_muldiv_op_1 = (count != 0) ? q_muldiv_op_1[head] : {`ALU_OP_WIDTH{1'b0}};
    assign out_mul_high_1 = (count != 0) ? q_mul_high_1[head] : 1'b0;
    assign out_mul_signed_rs1_1 = (count != 0) ? q_mul_signed_rs1_1[head] : 1'b0;
    assign out_mul_signed_rs2_1 = (count != 0) ? q_mul_signed_rs2_1[head] : 1'b0;
    assign out_div_signed_1 = (count != 0) ? q_div_signed_1[head] : 1'b0;
    assign out_div_is_rem_1 = (count != 0) ? q_div_is_rem_1[head] : 1'b0;
    assign out_branch_op_1 = (count != 0) ? q_branch_op_1[head] : `BR_OP_NONE;
    assign out_mem_op_1 = (count != 0) ? q_mem_op_1[head] : {`MEM_OP_WIDTH{1'b0}};
    assign out_mem_is_load_1 = (count != 0) ? q_mem_is_load_1[head] : 1'b0;
    assign out_mem_unsigned_1 = (count != 0) ? q_mem_unsigned_1[head] : 1'b0;
    assign out_csr_op_1 = (count != 0) ? q_csr_op_1[head] : `CSR_OP_NONE;
    assign out_csr_addr_1 = (count != 0) ? q_csr_addr_1[head] : 12'b0;
    assign out_fp_op_1 = (count != 0) ? q_fp_op_1[head] : `FP_OP_DUMMY;
    assign out_illegal_1 = (count != 0) ? q_illegal_1[head] : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head <= {PTR_W{1'b0}};
            tail <= {PTR_W{1'b0}};
            count <= {(PTR_W+1){1'b0}};
        end else if (flush) begin
            head <= {PTR_W{1'b0}};
            tail <= {PTR_W{1'b0}};
            count <= {(PTR_W+1){1'b0}};
        end else begin
            if (enq_fire) begin
                q_inst_valid[tail]       <= in_inst_valid;
                q_inst_0[tail]           <= in_inst_0;
                q_inst_1[tail]           <= in_inst_1;
                q_pc_0[tail]             <= in_pc_0;
                q_pc_1[tail]             <= in_pc_1;
                q_pred_taken_0[tail]     <= in_pred_taken_0;
                q_pred_target_0[tail]    <= in_pred_target_0;
                q_pred_hist_0[tail]      <= in_pred_hist_0;
                q_pred_taken_1[tail]     <= in_pred_taken_1;
                q_pred_target_1[tail]    <= in_pred_target_1;
                q_pred_hist_1[tail]      <= in_pred_hist_1;
                q_rs1_0[tail]            <= in_rs1_0;
                q_rs2_0[tail]            <= in_rs2_0;
                q_rd_0[tail]             <= in_rd_0;
                q_imm_0[tail]            <= in_imm_0;
                q_use_imm_0[tail]        <= in_use_imm_0;
                q_rs1_is_fp_0[tail]      <= in_rs1_is_fp_0;
                q_rs2_is_fp_0[tail]      <= in_rs2_is_fp_0;
                q_rd_is_fp_0[tail]       <= in_rd_is_fp_0;
                q_rob_idx_0[tail]        <= in_rob_idx_0;
                q_rob_idx_valid_0[tail]  <= in_rob_idx_valid_0;
                q_rob_gen_0[tail]        <= in_rob_gen_0;
                q_rs1_preg_0[tail]       <= in_rs1_preg_0;
                q_rs1_preg_valid_0[tail] <= in_rs1_preg_valid_0;
                q_rs2_preg_0[tail]       <= in_rs2_preg_0;
                q_rs2_preg_valid_0[tail] <= in_rs2_preg_valid_0;
                q_rd_preg_0[tail]        <= in_rd_preg_0;
                q_rd_preg_valid_0[tail]  <= in_rd_preg_valid_0;
                q_fu_sel_0[tail]         <= in_fu_sel_0;
                q_int_op_0[tail]         <= in_int_op_0;
                q_int_is_sub_0[tail]     <= in_int_is_sub_0;
                q_cmp_signed_0[tail]     <= in_cmp_signed_0;
                q_muldiv_op_0[tail]      <= in_muldiv_op_0;
                q_mul_high_0[tail]       <= in_mul_high_0;
                q_mul_signed_rs1_0[tail] <= in_mul_signed_rs1_0;
                q_mul_signed_rs2_0[tail] <= in_mul_signed_rs2_0;
                q_div_signed_0[tail]     <= in_div_signed_0;
                q_div_is_rem_0[tail]     <= in_div_is_rem_0;
                q_branch_op_0[tail]      <= in_branch_op_0;
                q_mem_op_0[tail]         <= in_mem_op_0;
                q_mem_is_load_0[tail]    <= in_mem_is_load_0;
                q_mem_unsigned_0[tail]   <= in_mem_unsigned_0;
                q_csr_op_0[tail]         <= in_csr_op_0;
                q_csr_addr_0[tail]       <= in_csr_addr_0;
                q_fp_op_0[tail]          <= in_fp_op_0;
                q_illegal_0[tail]        <= in_illegal_0;
                q_rs1_1[tail]            <= in_rs1_1;
                q_rs2_1[tail]            <= in_rs2_1;
                q_rd_1[tail]             <= in_rd_1;
                q_imm_1[tail]            <= in_imm_1;
                q_use_imm_1[tail]        <= in_use_imm_1;
                q_rs1_is_fp_1[tail]      <= in_rs1_is_fp_1;
                q_rs2_is_fp_1[tail]      <= in_rs2_is_fp_1;
                q_rd_is_fp_1[tail]       <= in_rd_is_fp_1;
                q_rob_idx_1[tail]        <= in_rob_idx_1;
                q_rob_idx_valid_1[tail]  <= in_rob_idx_valid_1;
                q_rob_gen_1[tail]        <= in_rob_gen_1;
                q_rs1_preg_1[tail]       <= in_rs1_preg_1;
                q_rs1_preg_valid_1[tail] <= in_rs1_preg_valid_1;
                q_rs2_preg_1[tail]       <= in_rs2_preg_1;
                q_rs2_preg_valid_1[tail] <= in_rs2_preg_valid_1;
                q_rd_preg_1[tail]        <= in_rd_preg_1;
                q_rd_preg_valid_1[tail]  <= in_rd_preg_valid_1;
                q_fu_sel_1[tail]         <= in_fu_sel_1;
                q_int_op_1[tail]         <= in_int_op_1;
                q_int_is_sub_1[tail]     <= in_int_is_sub_1;
                q_cmp_signed_1[tail]     <= in_cmp_signed_1;
                q_muldiv_op_1[tail]      <= in_muldiv_op_1;
                q_mul_high_1[tail]       <= in_mul_high_1;
                q_mul_signed_rs1_1[tail] <= in_mul_signed_rs1_1;
                q_mul_signed_rs2_1[tail] <= in_mul_signed_rs2_1;
                q_div_signed_1[tail]     <= in_div_signed_1;
                q_div_is_rem_1[tail]     <= in_div_is_rem_1;
                q_branch_op_1[tail]      <= in_branch_op_1;
                q_mem_op_1[tail]         <= in_mem_op_1;
                q_mem_is_load_1[tail]    <= in_mem_is_load_1;
                q_mem_unsigned_1[tail]   <= in_mem_unsigned_1;
                q_csr_op_1[tail]         <= in_csr_op_1;
                q_csr_addr_1[tail]       <= in_csr_addr_1;
                q_fp_op_1[tail]          <= in_fp_op_1;
                q_illegal_1[tail]        <= in_illegal_1;
                tail <= ptr_inc(tail);
            end

            if (deq_fire) begin
                head <= ptr_inc(head);
            end

            case ({enq_fire, deq_fire})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule
