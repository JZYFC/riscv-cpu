`timescale 1ns/1ps

module Top_tb;
    reg clk;
    reg rst_n;

    Top dut (
        .clk(clk),
        .rst_n(rst_n)
    );

    localparam [6:0] OPCODE_OP     = 7'b0110011;
    localparam [6:0] OPCODE_OP_IMM = 7'b0010011;
    localparam [6:0] OPCODE_BRANCH = 7'b1100011;
    localparam [6:0] OPCODE_JAL    = 7'b1101111;
    localparam [6:0] OPCODE_JALR   = 7'b1100111;
    localparam [6:0] OPCODE_LUI    = 7'b0110111;
    localparam [6:0] OPCODE_AUIPC  = 7'b0010111;
    localparam [6:0] OPCODE_MISC_MEM = 7'b0001111;
    localparam [6:0] OPCODE_SYSTEM   = 7'b1110011;

    localparam [2:0] F3_ADD_SUB = 3'b000;
    localparam [2:0] F3_SLL     = 3'b001;
    localparam [2:0] F3_SLT     = 3'b010;
    localparam [2:0] F3_SLTU    = 3'b011;
    localparam [2:0] F3_XOR     = 3'b100;
    localparam [2:0] F3_SRL_SRA = 3'b101;
    localparam [2:0] F3_OR      = 3'b110;
    localparam [2:0] F3_AND     = 3'b111;
    localparam [2:0] F3_BEQ     = 3'b000;
    localparam [2:0] F3_BNE     = 3'b001;
    localparam [2:0] F3_BLT     = 3'b100;
    localparam [2:0] F3_BGE     = 3'b101;
    localparam [2:0] F3_BLTU    = 3'b110;
    localparam [2:0] F3_BGEU    = 3'b111;

    function [31:0] enc_r;
        input [6:0] funct7;
        input [4:0] rs2;
        input [4:0] rs1;
        input [2:0] funct3;
        input [4:0] rd;
        input [6:0] opcode;
        begin
            enc_r = {funct7, rs2, rs1, funct3, rd, opcode};
        end
    endfunction

    function [31:0] enc_b;
        input [12:0] imm;
        input [4:0]  rs2;
        input [4:0]  rs1;
        input [2:0]  funct3;
        input [6:0]  opcode;
        begin
            enc_b = {imm[12], imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], opcode};
        end
    endfunction

    function [31:0] enc_j;
        input [20:0] imm;
        input [4:0]  rd;
        input [6:0]  opcode;
        begin
            enc_j = {imm[20], imm[10:1], imm[11], imm[19:12], rd, opcode};
        end
    endfunction

    function [31:0] enc_i;
        input [11:0] imm;
        input [4:0]  rs1;
        input [2:0]  funct3;
        input [4:0]  rd;
        input [6:0]  opcode;
        begin
            enc_i = {imm, rs1, funct3, rd, opcode};
        end
    endfunction

    function [31:0] enc_u;
        input [19:0] imm;
        input [4:0]  rd;
        input [6:0]  opcode;
        begin
            enc_u = {imm, rd, opcode};
        end
    endfunction

    task write_prf;
        input [4:0] regnum;
        input [31:0] value;
        begin
            if (regnum != 5'd0) begin
                dut.u_prf.int_prf[regnum] = value;
            end
        end
    endtask

    task run_test;
        input integer idx;
        input [31:0] inst;
        input [31:0] pc;
        input [4:0]  rd;
        input [4:0]  rs1;
        input [31:0] rs1_val;
        input [4:0]  rs2;
        input [31:0] rs2_val;
        input [31:0] exp_val;
        integer cycles;
        begin
            rst_n = 1'b0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
            write_prf(rs1, rs1_val);
            write_prf(rs2, rs2_val);

            force dut.u_if.out_inst_0 = inst;
            force dut.u_if.out_inst_1 = 32'h0000_0013;
            force dut.u_if.out_inst_valid = 2'b01;
            force dut.u_if.out_inst_addr_0 = pc;
            force dut.u_if.out_inst_addr_1 = pc + 32'd4;

            begin : wait_commit
                cycles = 0;
                while (cycles < 200) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (dut.commit0_valid) begin
                        if ((dut.commit0_arch_rd !== rd) || (dut.commit0_value !== exp_val)) begin
                            $display("FAIL[%0d] inst=%h rd=%0d exp=%h got_rd=%0d got=%h",
                                     idx, inst, rd, exp_val, dut.commit0_arch_rd, dut.commit0_value);
                        end else begin
                            $display("PASS[%0d] inst=%h rd=%0d val=%h",
                                     idx, inst, rd, dut.commit0_value);
                        end
                        disable wait_commit;
                    end
                end
                $display("TIMEOUT[%0d] inst=%h", idx, inst);
            end

            release dut.u_if.out_inst_0;
            release dut.u_if.out_inst_1;
            release dut.u_if.out_inst_valid;
            release dut.u_if.out_inst_addr_0;
            release dut.u_if.out_inst_addr_1;
        end
    endtask

    task run_redirect_test;
        input integer idx;
        input [31:0] inst;
        input [31:0] pc;
        input [4:0]  rs1;
        input [31:0] rs1_val;
        input [4:0]  rs2;
        input [31:0] rs2_val;
        input [31:0] exp_target;
        integer cycles;
        begin
            rst_n = 1'b0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
            write_prf(rs1, rs1_val);
            write_prf(rs2, rs2_val);

            force dut.u_if.out_inst_0 = inst;
            force dut.u_if.out_inst_1 = 32'h0000_0013;
            force dut.u_if.out_inst_valid = 2'b01;
            force dut.u_if.out_inst_addr_0 = pc;
            force dut.u_if.out_inst_addr_1 = pc + 32'd4;

            begin : wait_redirect
                cycles = 0;
                while (cycles < 200) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (dut.u_issue.redirect_valid) begin
                        if (dut.u_issue.redirect_target !== exp_target) begin
                            $display("FAIL_REDIRECT[%0d] inst=%h exp=%h got=%h",
                                     idx, inst, exp_target, dut.u_issue.redirect_target);
                        end else begin
                            $display("PASS_REDIRECT[%0d] inst=%h target=%h",
                                     idx, inst, dut.u_issue.redirect_target);
                        end
                        disable wait_redirect;
                    end
                end
                $display("TIMEOUT_REDIRECT[%0d] inst=%h", idx, inst);
            end

            release dut.u_if.out_inst_0;
            release dut.u_if.out_inst_1;
            release dut.u_if.out_inst_valid;
            release dut.u_if.out_inst_addr_0;
            release dut.u_if.out_inst_addr_1;
        end
    endtask

    task run_redirect_link_test;
        input integer idx;
        input [31:0] inst;
        input [31:0] pc;
        input [4:0]  rs1;
        input [31:0] rs1_val;
        input [4:0]  rs2;
        input [31:0] rs2_val;
        input [31:0] exp_target;
        input [31:0] exp_link;
        integer cycles;
        reg redirect_seen;
        reg link_seen;
        reg redirect_ok;
        reg link_ok;
        reg [31:0] got_link;
        reg [31:0] got_target;
        begin
            rst_n = 1'b0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
            write_prf(rs1, rs1_val);
            write_prf(rs2, rs2_val);

            force dut.u_if.out_inst_0 = inst;
            force dut.u_if.out_inst_1 = 32'h0000_0013;
            force dut.u_if.out_inst_valid = 2'b01;
            force dut.u_if.out_inst_addr_0 = pc;
            force dut.u_if.out_inst_addr_1 = pc + 32'd4;

            redirect_seen = 1'b0;
            link_seen = 1'b0;
            redirect_ok = 1'b1;
            link_ok = 1'b1;
            got_link = 32'h0;
            got_target = 32'h0;

            begin : wait_redirect_link
                cycles = 0;
                while (cycles < 200) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (!redirect_seen && dut.u_issue.redirect_valid) begin
                        redirect_seen = 1'b1;
                        got_target = dut.u_issue.redirect_target;
                        if (got_target !== exp_target) begin
                            redirect_ok = 1'b0;
                        end
                    end
                    if (!link_seen && (dut.prf_i_wr_we0 || dut.prf_i_wr_we1)) begin
                        link_seen = 1'b1;
                        if (dut.prf_i_wr_we0) begin
                            got_link = dut.prf_i_wr_data0;
                        end else begin
                            got_link = dut.prf_i_wr_data1;
                        end
                        if (got_link !== exp_link) begin
                            link_ok = 1'b0;
                        end
                    end
                    if (redirect_seen && link_seen) begin
                        if (redirect_ok && link_ok) begin
                            $display("PASS_REDIRECT_LINK[%0d] inst=%h target=%h link=%h",
                                     idx, inst, got_target, got_link);
                        end else begin
                            $display("FAIL_REDIRECT_LINK[%0d] inst=%h exp_target=%h got_target=%h exp_link=%h got_link=%h",
                                     idx, inst, exp_target, got_target, exp_link, got_link);
                        end
                        disable wait_redirect_link;
                    end
                end
                $display("TIMEOUT_REDIRECT_LINK[%0d] inst=%h", idx, inst);
            end

            release dut.u_if.out_inst_0;
            release dut.u_if.out_inst_1;
            release dut.u_if.out_inst_valid;
            release dut.u_if.out_inst_addr_0;
            release dut.u_if.out_inst_addr_1;
        end
    endtask

    task run_noeffect_test;
        input integer idx;
        input [31:0] inst;
        input [31:0] pc;
        integer cycles;
        reg seen_enq;
        reg seen_done;
        reg bad_redirect;
        reg bad_write;
        begin
            rst_n = 1'b0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);

            force dut.u_if.out_inst_0 = inst;
            force dut.u_if.out_inst_1 = 32'h0000_0013;
            force dut.u_if.out_inst_valid = 2'b01;
            force dut.u_if.out_inst_addr_0 = pc;
            force dut.u_if.out_inst_addr_1 = pc + 32'd4;
            @(posedge clk);
            force dut.u_if.out_inst_valid = 2'b00;

            seen_enq = 1'b0;
            seen_done = 1'b0;
            bad_redirect = 1'b0;
            bad_write = 1'b0;

            begin : wait_noeffect
                cycles = 0;
                while (cycles < 200) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (!seen_enq && (dut.u_issue.rs_count != 0)) begin
                        seen_enq = 1'b1;
                    end
                    if (seen_enq && (dut.u_issue.rs_count == 0)) begin
                        seen_done = 1'b1;
                    end
                    if (dut.u_issue.redirect_valid) begin
                        bad_redirect = 1'b1;
                    end
                    if (dut.prf_i_wr_we0 || dut.prf_i_wr_we1) begin
                        bad_write = 1'b1;
                    end
                    if (seen_done) begin
                        if (!bad_redirect && !bad_write) begin
                            $display("PASS_NOEFFECT[%0d] inst=%h", idx, inst);
                        end else begin
                            $display("FAIL_NOEFFECT[%0d] inst=%h redirect=%b write=%b",
                                     idx, inst, bad_redirect, bad_write);
                        end
                        disable wait_noeffect;
                    end
                end
                $display("TIMEOUT_NOEFFECT[%0d] inst=%h", idx, inst);
            end

            release dut.u_if.out_inst_0;
            release dut.u_if.out_inst_1;
            release dut.u_if.out_inst_valid;
            release dut.u_if.out_inst_addr_0;
            release dut.u_if.out_inst_addr_1;
        end
    endtask

    task run_dual_test;
        input integer idx;
        input [31:0] inst0;
        input [31:0] inst1;
        input [31:0] pc;
        input [4:0]  rd0;
        input [4:0]  rs10;
        input [31:0] rs10_val;
        input [4:0]  rs20;
        input [31:0] rs20_val;
        input [31:0] exp0;
        input [4:0]  rd1;
        input [4:0]  rs11;
        input [31:0] rs11_val;
        input [4:0]  rs21;
        input [31:0] rs21_val;
        input [31:0] exp1;
        integer cycles;
        integer count;
        integer inject_wait;
        reg pass;
        reg [4:0] got_rd0;
        reg [4:0] got_rd1;
        reg [31:0] got_val0;
        reg [31:0] got_val1;
        begin
            rst_n = 1'b0;
            repeat (5) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
            write_prf(rs10, rs10_val);
            write_prf(rs20, rs20_val);
            write_prf(rs11, rs11_val);
            write_prf(rs21, rs21_val);

            force dut.u_if.out_inst_0 = inst0;
            force dut.u_if.out_inst_1 = inst1;
            force dut.u_if.out_inst_valid = 2'b11;
            force dut.u_if.out_inst_addr_0 = pc;
            force dut.u_if.out_inst_addr_1 = pc + 32'd4;
            inject_wait = 0;
            while (inject_wait < 10 && dut.ib_valid == 2'b00) begin
                @(posedge clk);
                #1;
                inject_wait = inject_wait + 1;
            end
            force dut.u_if.out_inst_valid = 2'b00;

            pass = 1'b1;
            count = 0;
            got_rd0 = 5'd0;
            got_rd1 = 5'd0;
            got_val0 = 32'h0;
            got_val1 = 32'h0;

            begin : wait_dual_commit
                cycles = 0;
                while (cycles < 200) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (dut.commit0_valid) begin
                        if (count == 0) begin
                            got_rd0 = dut.commit0_arch_rd;
                            got_val0 = dut.commit0_value;
                            if ((dut.commit0_arch_rd !== rd0) || (dut.commit0_value !== exp0)) begin
                                pass = 1'b0;
                            end
                        end else if (count == 1) begin
                            got_rd1 = dut.commit0_arch_rd;
                            got_val1 = dut.commit0_value;
                            if ((dut.commit0_arch_rd !== rd1) || (dut.commit0_value !== exp1)) begin
                                pass = 1'b0;
                            end
                        end
                        count = count + 1;
                    end
                    if (dut.commit1_valid) begin
                        if (count == 0) begin
                            got_rd0 = dut.commit1_arch_rd;
                            got_val0 = dut.commit1_value;
                            if ((dut.commit1_arch_rd !== rd0) || (dut.commit1_value !== exp0)) begin
                                pass = 1'b0;
                            end
                        end else if (count == 1) begin
                            got_rd1 = dut.commit1_arch_rd;
                            got_val1 = dut.commit1_value;
                            if ((dut.commit1_arch_rd !== rd1) || (dut.commit1_value !== exp1)) begin
                                pass = 1'b0;
                            end
                        end
                        count = count + 1;
                    end
                    if (!pass) begin
                        $display("DBG_DUAL[%0d] cyc=%0d if=%b ib_stall=%b ib=%b pre=%b rn=%b post=%b post0=%h post1=%h il0=%b il1=%b rs_cnt=%0d idx0=%0d idx1=%0d wb0=%b wb1=%b wb_exc0=%b wb_exc1=%b c0=%b c1=%b c0_exc=%b c1_exc=%b rn_stall=%b issue_stall=%b disp_ready=%b flush=%b redirect=%b",
                                 idx, cycles,
                                 dut.if_inst_valid, dut.ib_stall_if, dut.ib_valid, dut.pre_valid, dut.rn_valid, dut.post_valid,
                                 dut.post_inst_0, dut.post_inst_1, dut.post_illegal_0, dut.post_illegal_1,
                                 dut.u_issue.rs_count, dut.u_issue.issue_idx0, dut.u_issue.issue_idx1,
                                 dut.u_issue.wb0_valid, dut.u_issue.wb1_valid,
                                 dut.wb0_exception_exe, dut.wb1_exception_exe,
                                 dut.commit0_valid, dut.commit1_valid,
                                 dut.commit0_exception, dut.commit1_exception,
                                 dut.rn_stall, dut.issue_stall, dut.dispatch_ready,
                                 dut.global_flush, dut.redirect_valid);
                    end
                    if (count >= 2) begin
                        if (pass) begin
                            $display("PASS_DUAL[%0d] inst0=%h inst1=%h", idx, inst0, inst1);
                        end else begin
                            $display("FAIL_DUAL[%0d] inst0=%h inst1=%h exp0(rd=%0d val=%h) got0(rd=%0d val=%h) exp1(rd=%0d val=%h) got1(rd=%0d val=%h)",
                                     idx, inst0, inst1,
                                     rd0, exp0, got_rd0, got_val0,
                                     rd1, exp1, got_rd1, got_val1);
                        end
                        disable wait_dual_commit;
                    end
                end
                if (!pass) begin
                    $display("DBG_DUAL_TIMEOUT[%0d] if=%b ib_stall=%b ib=%b pre=%b rn=%b post=%b post0=%h post1=%h il0=%b il1=%b rs_cnt=%0d idx0=%0d idx1=%0d wb0=%b wb1=%b wb_exc0=%b wb_exc1=%b c0=%b c1=%b c0_exc=%b c1_exc=%b rn_stall=%b issue_stall=%b disp_ready=%b flush=%b redirect=%b",
                             idx,
                             dut.if_inst_valid, dut.ib_stall_if, dut.ib_valid, dut.pre_valid, dut.rn_valid, dut.post_valid,
                             dut.post_inst_0, dut.post_inst_1, dut.post_illegal_0, dut.post_illegal_1,
                             dut.u_issue.rs_count, dut.u_issue.issue_idx0, dut.u_issue.issue_idx1,
                             dut.u_issue.wb0_valid, dut.u_issue.wb1_valid,
                             dut.wb0_exception_exe, dut.wb1_exception_exe,
                             dut.commit0_valid, dut.commit1_valid,
                             dut.commit0_exception, dut.commit1_exception,
                             dut.rn_stall, dut.issue_stall, dut.dispatch_ready,
                             dut.global_flush, dut.redirect_valid);
                end
                $display("TIMEOUT_DUAL[%0d] inst0=%h inst1=%h", idx, inst0, inst1);
            end

            release dut.u_if.out_inst_0;
            release dut.u_if.out_inst_1;
            release dut.u_if.out_inst_valid;
            release dut.u_if.out_inst_addr_0;
            release dut.u_if.out_inst_addr_1;
        end
    endtask

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        // ADD
        run_test(0,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_ADD_SUB, 5'd3, OPCODE_OP),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'd7, 5'd2, 32'd3, 32'd10);

        // SUB (5 - 7 = -2)
        run_test(1,
                 enc_r(7'b0100000, 5'd2, 5'd1, F3_ADD_SUB, 5'd4, OPCODE_OP),
                 32'h0000_1000,
                 5'd4, 5'd1, 32'd5, 5'd2, 32'd7, 32'hFFFF_FFFE);

        // AND
        run_test(2,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_AND, 5'd5, OPCODE_OP),
                 32'h0000_1000,
                 5'd5, 5'd1, 32'hA5A5_A5A5, 5'd2, 32'h0F0F_0F0F, 32'h0505_0505);

        // OR
        run_test(3,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_OR, 5'd6, OPCODE_OP),
                 32'h0000_1000,
                 5'd6, 5'd1, 32'hA5A5_A5A5, 5'd2, 32'h0F0F_0F0F, 32'hAFAF_AFAF);

        // XOR
        run_test(4,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_XOR, 5'd7, OPCODE_OP),
                 32'h0000_1000,
                 5'd7, 5'd1, 32'hA5A5_A5A5, 5'd2, 32'h0F0F_0F0F, 32'hAAAA_AAAA);

        // SLL
        run_test(5,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_SLL, 5'd8, OPCODE_OP),
                 32'h0000_1000,
                 5'd8, 5'd1, 32'h0000_0001, 5'd2, 32'd3, 32'h0000_0008);

        // SRL
        run_test(6,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_SRL_SRA, 5'd9, OPCODE_OP),
                 32'h0000_1000,
                 5'd9, 5'd1, 32'h8000_0000, 5'd2, 32'd1, 32'h4000_0000);

        // SRA
        run_test(7,
                 enc_r(7'b0100000, 5'd2, 5'd1, F3_SRL_SRA, 5'd10, OPCODE_OP),
                 32'h0000_1000,
                 5'd10, 5'd1, 32'h8000_0000, 5'd2, 32'd1, 32'hC000_0000);

        // SLT (signed)
        run_test(8,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_SLT, 5'd11, OPCODE_OP),
                 32'h0000_1000,
                 5'd11, 5'd1, 32'hFFFF_FFF0, 5'd2, 32'h0000_0001, 32'h0000_0001);

        // SLTU (unsigned)
        run_test(9,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_SLTU, 5'd12, OPCODE_OP),
                 32'h0000_1000,
                 5'd12, 5'd1, 32'hFFFF_FFF0, 5'd2, 32'h0000_0001, 32'h0000_0000);

        // ADDI
        run_test(10,
                 enc_i(12'h00A, 5'd1, F3_ADD_SUB, 5'd13, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd13, 5'd1, 32'd5, 5'd0, 32'd0, 32'd15);

        // SLTI
        run_test(11,
                 enc_i(12'd1, 5'd1, 3'b010, 5'd14, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd14, 5'd1, 32'hFFFF_FFFF, 5'd0, 32'd0, 32'd1);

        // SLTIU
        run_test(12,
                 enc_i(12'd1, 5'd1, 3'b011, 5'd15, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd15, 5'd1, 32'hFFFF_FFFF, 5'd0, 32'd0, 32'd0);

        // XORI
        run_test(13,
                 enc_i(12'h0F0, 5'd1, 3'b100, 5'd16, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd16, 5'd1, 32'h0000_0055, 5'd0, 32'd0, 32'h0000_00A5);

        // ORI
        run_test(14,
                 enc_i(12'h0F0, 5'd1, 3'b110, 5'd17, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd17, 5'd1, 32'h0000_0055, 5'd0, 32'd0, 32'h0000_00F5);

        // ANDI
        run_test(15,
                 enc_i(12'h0F0, 5'd1, 3'b111, 5'd18, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd18, 5'd1, 32'h0000_0055, 5'd0, 32'd0, 32'h0000_0050);

        // SLLI
        run_test(16,
                 enc_i(12'h003, 5'd1, 3'b001, 5'd19, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd19, 5'd1, 32'h0000_0001, 5'd0, 32'd0, 32'h0000_0008);

        // SRLI
        run_test(17,
                 enc_i(12'h001, 5'd1, 3'b101, 5'd20, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd20, 5'd1, 32'h8000_0000, 5'd0, 32'd0, 32'h4000_0000);

        // SRAI
        run_test(18,
                 enc_i(12'h401, 5'd1, 3'b101, 5'd21, OPCODE_OP_IMM),
                 32'h0000_1000,
                 5'd21, 5'd1, 32'h8000_0000, 5'd0, 32'd0, 32'hC000_0000);

        // LUI
        run_test(19,
                 enc_u(20'h12345, 5'd22, OPCODE_LUI),
                 32'h0000_1000,
                 5'd22, 5'd0, 32'd0, 5'd0, 32'd0, 32'h1234_5000);

        // AUIPC
        run_test(20,
                 enc_u(20'h00012, 5'd23, OPCODE_AUIPC),
                 32'h0000_1000,
                 5'd23, 5'd0, 32'd0, 5'd0, 32'd0, 32'h0001_3000);

        // BEQ taken -> redirect to PC+8
        run_redirect_test(21,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BEQ, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd5, 5'd2, 32'd5, 32'h0000_1008);

        // BNE taken
        run_redirect_test(22,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BNE, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd5, 5'd2, 32'd7, 32'h0000_1008);

        // BLT taken
        run_redirect_test(23,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BLT, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'hFFFF_FFFF, 5'd2, 32'd1, 32'h0000_1008);

        // BGE taken
        run_redirect_test(24,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BGE, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd1, 5'd2, 32'hFFFF_FFFF, 32'h0000_1008);

        // BLTU taken
        run_redirect_test(25,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BLTU, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd1, 5'd2, 32'd2, 32'h0000_1008);

        // BGEU taken
        run_redirect_test(26,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BGEU, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'hFFFF_FFFF, 5'd2, 32'd1, 32'h0000_1008);

        // JAL -> redirect to PC+16
        run_redirect_link_test(27,
                 enc_j(21'd16, 5'd1, OPCODE_JAL),
                 32'h0000_1000,
                 5'd0, 32'd0, 5'd0, 32'd0, 32'h0000_1010, 32'h0000_1004);

        // JALR -> redirect to (rs1+4)&~1
        run_redirect_link_test(28,
                 enc_i(12'd4, 5'd5, F3_ADD_SUB, 5'd1, OPCODE_JALR),
                 32'h0000_1000,
                 5'd5, 32'h0000_2000, 5'd0, 32'd0, 32'h0000_2004, 32'h0000_1004);

        // FENCE
        run_noeffect_test(29,
                 enc_i(12'd0, 5'd0, 3'b000, 5'd0, OPCODE_MISC_MEM),
                 32'h0000_1000);

        // FENCE.I
        run_noeffect_test(30,
                 enc_i(12'd0, 5'd0, 3'b001, 5'd0, OPCODE_MISC_MEM),
                 32'h0000_1000);

        // ECALL
        run_noeffect_test(31,
                 enc_i(12'd0, 5'd0, 3'b000, 5'd0, OPCODE_SYSTEM),
                 32'h0000_1000);

        // EBREAK
        run_noeffect_test(32,
                 enc_i(12'd1, 5'd0, 3'b000, 5'd0, OPCODE_SYSTEM),
                 32'h0000_1000);

        // Dual-issue: independent adds
        run_dual_test(33,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_ADD_SUB, 5'd3, OPCODE_OP),
                 enc_r(7'b0000000, 5'd6, 5'd5, F3_ADD_SUB, 5'd4, OPCODE_OP),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'd7, 5'd2, 32'd3, 32'd10,
                 5'd4, 5'd5, 32'd11, 5'd6, 32'd22, 32'd33);

        // Dual-issue: RAW dependency (rd0 -> rs1 of inst1)
        run_dual_test(34,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_ADD_SUB, 5'd7, OPCODE_OP),
                 enc_r(7'b0000000, 5'd3, 5'd7, F3_ADD_SUB, 5'd8, OPCODE_OP),
                 32'h0000_1000,
                 5'd7, 5'd1, 32'd5, 5'd2, 32'd6, 32'd11,
                 5'd8, 5'd7, 32'd0, 5'd3, 32'd4, 32'd15);

        // Dual-issue: WAW to same rd
        run_dual_test(35,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_ADD_SUB, 5'd9, OPCODE_OP),
                 enc_r(7'b0000000, 5'd4, 5'd3, F3_ADD_SUB, 5'd9, OPCODE_OP),
                 32'h0000_1000,
                 5'd9, 5'd1, 32'd5, 5'd2, 32'd6, 32'd11,
                 5'd9, 5'd3, 32'd2, 5'd4, 32'd3, 32'd5);

        $finish;
    end
endmodule
