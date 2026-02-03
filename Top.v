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
wire dispatch_ready = ~rn_stall & ~issue_stall;
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
    .stall(issue_stall),
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
wire [`PREG_IDX_WIDTH-1:0] lsu_rd_tag;
wire lsu_rd_is_fp;
wire lsu_busy;
wire lsu_wb_valid;
wire [`DATA_WIDTH-1:0] lsu_wb_value;
wire [`ROB_IDX_WIDTH-1:0] lsu_wb_rob_idx;
wire [`PREG_IDX_WIDTH-1:0] lsu_wb_dest_tag;
wire lsu_wb_dest_is_fp;
wire lsu_wb_exception;

RegRename u_regrename (
    .clk(clk),
    .rst_n(rst_n),
    .flush(global_flush),
    .flush_is_redirect(redirect_valid),
    .flush_rob_idx(redirect_valid ? redirect_rob_idx :
                               (commit_exc ? commit0_rob_idx : {`ROB_IDX_WIDTH{1'b0}})),
    .stall(issue_stall),
    .in_inst_valid(pre_valid & {`IF_BATCH_SIZE{~issue_stall}}),
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
    .wb0_value(wb0_value_exe),
    .wb0_exception(wb0_exception_exe),
    .wb1_valid(wb1_valid_exe),
    .wb1_rob_idx(wb1_rob_idx_exe),
    .wb1_value(wb1_value_exe),
    .wb1_exception(wb1_exception_exe),
    .wb2_valid(wb2_valid_exe),
    .wb2_rob_idx(wb2_rob_idx_exe),
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
    .rename_stall(rn_stall)
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
    .stall(issue_stall),
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

// ----------------------------
// Shared PRF instance
// ----------------------------
PhysicalRegFileShared u_prf (
    .clk(clk),
    .rst_n(rst_n),
    // Integer reads
    .i_rs1_addr0(post_rs1_preg_0),
    .i_rs2_addr0(post_rs2_preg_0),
    .i_rs1_addr1(post_rs1_preg_1),
    .i_rs2_addr1(post_rs2_preg_1),
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
    .f_rs1_addr0(post_rs1_preg_0),
    .f_rs2_addr0(post_rs2_preg_0),
    .f_rs1_addr1(post_rs1_preg_1),
    .f_rs2_addr1(post_rs2_preg_1),
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
    .in_inst_valid(post_valid),
    .in_pred_taken_0(post_pred_taken_0),
    .in_pred_target_0(post_pred_target_0),
    .in_pred_hist_0(post_pred_hist_0),
    .in_pred_taken_1(post_pred_taken_1),
    .in_pred_target_1(post_pred_target_1),
    .in_pred_hist_1(post_pred_hist_1),
    .in_inst_0(post_inst_0),
    .in_inst_1(post_inst_1),
    .in_pc_0(post_pc_0),
    .in_pc_1(post_pc_1),
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
    .rob_head(rob_head),
    .stall_dispatch(issue_stall),
    .wb0_valid(wb0_valid_exe),
    .wb0_rob_idx(wb0_rob_idx_exe),
    .wb0_value(wb0_value_exe),
    .wb0_exception(wb0_exception_exe),
    .wb1_valid(wb1_valid_exe),
    .wb1_rob_idx(wb1_rob_idx_exe),
    .wb1_value(wb1_value_exe),
    .wb1_exception(wb1_exception_exe),
    .wb2_valid(wb2_valid_exe),
    .wb2_rob_idx(wb2_rob_idx_exe),
    .wb2_value(wb2_value_exe),
    .wb2_exception(wb2_exception_exe),
    .lsu_valid(lsu_valid),
    .lsu_addr(lsu_addr),
    .lsu_wdata(lsu_wdata),
    .lsu_mem_op(lsu_mem_op),
    .lsu_mem_is_load(lsu_mem_is_load),
    .lsu_mem_unsigned(lsu_mem_unsigned),
    .lsu_rob_idx(lsu_rob_idx),
    .lsu_rd_tag(lsu_rd_tag),
    .lsu_rd_is_fp(lsu_rd_is_fp),
    .lsu_busy(lsu_busy),
    .lsu_wb_valid(lsu_wb_valid),
    .lsu_wb_value(lsu_wb_value),
    .lsu_wb_rob_idx(lsu_wb_rob_idx),
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
    .rd_tag_in(lsu_rd_tag),
    .rd_is_fp_in(lsu_rd_is_fp),
    .busy(lsu_busy),
    .wb_valid(lsu_wb_valid),
    .wb_value(lsu_wb_value),
    .wb_rob_idx(lsu_wb_rob_idx),
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
