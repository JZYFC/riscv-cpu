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
    localparam [6:0] OPCODE_OP_FP  = 7'b1010011;
    localparam [6:0] OPCODE_LUI    = 7'b0110111;
    localparam [6:0] OPCODE_AUIPC  = 7'b0010111;
    localparam [6:0] OPCODE_LOAD   = 7'b0000011;
    localparam [6:0] OPCODE_STORE  = 7'b0100011;
    localparam [6:0] OPCODE_MISC_MEM = 7'b0001111;
    localparam [6:0] OPCODE_SYSTEM   = 7'b1110011;

    localparam integer CACHE_INDEX_BITS = 6;
    localparam integer CACHE_TAG_BITS = 22;
    localparam integer CACHE_SETS = 1 << CACHE_INDEX_BITS;

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
    localparam [2:0] F3_MUL     = 3'b000;
    localparam [2:0] F3_MULH    = 3'b001;
    localparam [2:0] F3_MULHSU  = 3'b010;
    localparam [2:0] F3_MULHU   = 3'b011;
    localparam [2:0] RM_RNE     = 3'b000;
    localparam [2:0] F3_CSRRW   = 3'b001;
    localparam [2:0] F3_CSRRS   = 3'b010;
    localparam [2:0] F3_CSRRC   = 3'b011;
    localparam [2:0] F3_CSRRWI  = 3'b101;
    localparam [2:0] F3_CSRRSI  = 3'b110;
    localparam [2:0] F3_CSRRCI  = 3'b111;

    localparam integer TASK_MAX_REG_CHECK = 8;
    localparam integer TASK_MAX_MEM_CHECK = 8;
    localparam integer TASK_DEBUG_STRIDE = 1;
    localparam integer DEBUG_TASK_IDX = 70;
    localparam [31:0] DEBUG_PC_LO = 32'h0000_1050;
    localparam [31:0] DEBUG_PC_HI = 32'h0000_1090;
    reg [4:0] task_reg_num [0:TASK_MAX_REG_CHECK-1];
    reg [31:0] task_reg_val [0:TASK_MAX_REG_CHECK-1];
    integer task_reg_count;
    reg [31:0] task_mem_addr [0:TASK_MAX_MEM_CHECK-1];
    reg [31:0] task_mem_val [0:TASK_MAX_MEM_CHECK-1];
    integer task_mem_count;

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

    function [31:0] enc_s;
        input [11:0] imm;
        input [4:0]  rs2;
        input [4:0]  rs1;
        input [2:0]  funct3;
        input [6:0]  opcode;
        begin
            enc_s = {imm[11:5], rs2, rs1, funct3, imm[4:0], opcode};
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

    function is_debug_pc;
        input [31:0] pc;
        begin
            is_debug_pc = (pc >= DEBUG_PC_LO) && (pc <= DEBUG_PC_HI);
        end
    endfunction

    task clear_mainmem;
        integer i;
        begin
            for (i = 0; i < 256; i = i + 1) begin
                dut.u_mainmem.ram[i] = 128'b0;
            end
        end
    endtask

    task clear_caches;
        integer i;
        begin
            for (i = 0; i < CACHE_SETS; i = i + 1) begin
                dut.u_icache.tag_way0[i] = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_icache.tag_way1[i] = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_icache.data_way0[i] = 128'b0;
                dut.u_icache.data_way1[i] = 128'b0;
                dut.u_icache.lru[i] = 1'b0;
                dut.u_lsu.u_cache.tag_way0[i] = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_lsu.u_cache.tag_way1[i] = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_lsu.u_cache.data_way0[i] = 128'b0;
                dut.u_lsu.u_cache.data_way1[i] = 128'b0;
                dut.u_lsu.u_cache.lru[i] = 1'b0;
            end
        end
    endtask

    task write_mem_line;
        input [31:0] addr;
        input [127:0] line;
        begin
            dut.u_mainmem.ram[addr[11:4]] = line;
        end
    endtask

    task write_mem_word;
        input [31:0] addr;
        input [31:0] value;
        reg [7:0] idx;
        reg [1:0] word_idx;
        reg [127:0] line;
        begin
            idx = addr[11:4];
            word_idx = addr[3:2];
            line = dut.u_mainmem.ram[idx];
            case (word_idx)
                2'b00: line[31:0] = value;
                2'b01: line[63:32] = value;
                2'b10: line[95:64] = value;
                2'b11: line[127:96] = value;
            endcase
            dut.u_mainmem.ram[idx] = line;
        end
    endtask

    function [31:0] peek_arch_reg;
        input [4:0] regnum;
        reg [5:0] preg;
        begin
            if (regnum == 5'd0) begin
                peek_arch_reg = 32'b0;
            end else begin
                preg = dut.u_regrename.int_map[regnum];
                peek_arch_reg = dut.u_prf.int_prf[preg];
            end
        end
    endfunction

    function [31:0] peek_imem_word;
        input [31:0] addr;
        reg [127:0] line;
        begin
            line = dut.u_mainmem.ram[addr[11:4]];
            case (addr[3:2])
                2'b00: peek_imem_word = line[31:0];
                2'b01: peek_imem_word = line[63:32];
                2'b10: peek_imem_word = line[95:64];
                2'b11: peek_imem_word = line[127:96];
            endcase
        end
    endfunction

    function [31:0] peek_mem_word;
        input [31:0] addr;
        reg [CACHE_TAG_BITS-1:0] tag;
        reg [CACHE_INDEX_BITS-1:0] index;
        reg [CACHE_TAG_BITS+1:0] raw0;
        reg [CACHE_TAG_BITS+1:0] raw1;
        reg hit0;
        reg hit1;
        reg [127:0] line;
        begin
            tag = addr[31:32-CACHE_TAG_BITS];
            index = addr[4 + CACHE_INDEX_BITS - 1 : 4];
            raw0 = dut.u_lsu.u_cache.tag_way0[index];
            raw1 = dut.u_lsu.u_cache.tag_way1[index];
            hit0 = raw0[CACHE_TAG_BITS] && (raw0[CACHE_TAG_BITS-1:0] == tag);
            hit1 = raw1[CACHE_TAG_BITS] && (raw1[CACHE_TAG_BITS-1:0] == tag);

            if (hit1) begin
                line = dut.u_lsu.u_cache.data_way1[index];
            end else if (hit0) begin
                line = dut.u_lsu.u_cache.data_way0[index];
            end else begin
                line = dut.u_mainmem.ram[addr[11:4]];
            end

            case (addr[3:2])
                2'b00: peek_mem_word = line[31:0];
                2'b01: peek_mem_word = line[63:32];
                2'b10: peek_mem_word = line[95:64];
                2'b11: peek_mem_word = line[127:96];
            endcase
        end
    endfunction

    task load_sum_program;
        input [31:0] base_pc;
        begin
            // main
            write_mem_word(base_pc + 32'd0,  enc_i(12'hFE0, 5'd2, F3_ADD_SUB, 5'd2, OPCODE_OP_IMM)); // addi sp,sp,-32
            write_mem_word(base_pc + 32'd4,  enc_s(12'h01C, 5'd1, 5'd2, 3'b010, OPCODE_STORE));     // sw ra,28(sp)
            write_mem_word(base_pc + 32'd8,  enc_s(12'h018, 5'd8, 5'd2, 3'b010, OPCODE_STORE));     // sw s0,24(sp)
            write_mem_word(base_pc + 32'd12, enc_i(12'h020, 5'd2, F3_ADD_SUB, 5'd8, OPCODE_OP_IMM)); // addi s0,sp,32
            write_mem_word(base_pc + 32'd16, enc_i(12'h00A, 5'd0, F3_ADD_SUB, 5'd15, OPCODE_OP_IMM)); // li a5,10
            write_mem_word(base_pc + 32'd20, enc_s(12'hFEC, 5'd15, 5'd8, 3'b010, OPCODE_STORE));     // sw a5,-20(s0)
            write_mem_word(base_pc + 32'd24, enc_i(12'h014, 5'd0, F3_ADD_SUB, 5'd15, OPCODE_OP_IMM)); // li a5,20
            write_mem_word(base_pc + 32'd28, enc_s(12'hFE8, 5'd15, 5'd8, 3'b010, OPCODE_STORE));     // sw a5,-24(s0)
            write_mem_word(base_pc + 32'd32, enc_i(12'h014, 5'd0, F3_ADD_SUB, 5'd11, OPCODE_OP_IMM)); // li a1,20
            write_mem_word(base_pc + 32'd36, enc_i(12'h00A, 5'd0, F3_ADD_SUB, 5'd10, OPCODE_OP_IMM)); // li a0,10
            write_mem_word(base_pc + 32'd40, enc_j(21'd28, 5'd1, OPCODE_JAL));                        // call sum
            write_mem_word(base_pc + 32'd44, enc_s(12'hFE4, 5'd10, 5'd8, 3'b010, OPCODE_STORE));     // sw a0,-28(s0)
            write_mem_word(base_pc + 32'd48, enc_i(12'h000, 5'd0, F3_ADD_SUB, 5'd0, OPCODE_OP_IMM)); // nop
            write_mem_word(base_pc + 32'd52, enc_i(12'h01C, 5'd2, 3'b010, 5'd1, OPCODE_LOAD));       // lw ra,28(sp)
            write_mem_word(base_pc + 32'd56, enc_i(12'h018, 5'd2, 3'b010, 5'd8, OPCODE_LOAD));       // lw s0,24(sp)
            write_mem_word(base_pc + 32'd60, enc_i(12'h020, 5'd2, F3_ADD_SUB, 5'd2, OPCODE_OP_IMM)); // addi sp,sp,32
            write_mem_word(base_pc + 32'd64, enc_i(12'h000, 5'd1, F3_ADD_SUB, 5'd0, OPCODE_JALR));   // jr ra

            // sum (base_pc + 0x44)
            write_mem_word(base_pc + 32'h44, enc_i(12'hFD0, 5'd2, F3_ADD_SUB, 5'd2, OPCODE_OP_IMM)); // addi sp,sp,-48
            write_mem_word(base_pc + 32'h48, enc_s(12'h02C, 5'd1, 5'd2, 3'b010, OPCODE_STORE));     // sw ra,44(sp)
            write_mem_word(base_pc + 32'h4C, enc_s(12'h028, 5'd8, 5'd2, 3'b010, OPCODE_STORE));     // sw s0,40(sp)
            write_mem_word(base_pc + 32'h50, enc_i(12'h030, 5'd2, F3_ADD_SUB, 5'd8, OPCODE_OP_IMM)); // addi s0,sp,48
            write_mem_word(base_pc + 32'h54, enc_s(12'hFDC, 5'd10, 5'd8, 3'b010, OPCODE_STORE));    // sw a0,-36(s0)
            write_mem_word(base_pc + 32'h58, enc_s(12'hFD8, 5'd11, 5'd8, 3'b010, OPCODE_STORE));    // sw a1,-40(s0)
            write_mem_word(base_pc + 32'h5C, enc_s(12'hFEC, 5'd0, 5'd8, 3'b010, OPCODE_STORE));     // sw zero,-20(s0)
            write_mem_word(base_pc + 32'h60, enc_i(12'hFD8, 5'd8, 3'b010, 5'd14, OPCODE_LOAD));     // lw a4,-40(s0)
            write_mem_word(base_pc + 32'h64, enc_i(12'hFDC, 5'd8, 3'b010, 5'd15, OPCODE_LOAD));     // lw a5,-36(s0)
            write_mem_word(base_pc + 32'h68, enc_r(7'b0100000, 5'd15, 5'd14, F3_ADD_SUB, 5'd15, OPCODE_OP)); // sub a5,a4,a5
            write_mem_word(base_pc + 32'h6C, enc_s(12'hFE4, 5'd15, 5'd8, 3'b010, OPCODE_STORE));    // sw a5,-28(s0)
            write_mem_word(base_pc + 32'h70, enc_s(12'hFE8, 5'd0, 5'd8, 3'b010, OPCODE_STORE));     // sw zero,-24(s0)
            write_mem_word(base_pc + 32'h74, enc_j(21'd40, 5'd0, OPCODE_JAL));                       // j .L2
            write_mem_word(base_pc + 32'h78, enc_i(12'hFDC, 5'd8, 3'b010, 5'd14, OPCODE_LOAD));     // lw a4,-36(s0)
            write_mem_word(base_pc + 32'h7C, enc_i(12'hFEC, 5'd8, 3'b010, 5'd15, OPCODE_LOAD));     // lw a5,-20(s0)
            write_mem_word(base_pc + 32'h80, enc_r(7'b0000000, 5'd15, 5'd14, F3_ADD_SUB, 5'd15, OPCODE_OP)); // add a5,a4,a5
            write_mem_word(base_pc + 32'h84, enc_i(12'hFE8, 5'd8, 3'b010, 5'd14, OPCODE_LOAD));     // lw a4,-24(s0)
            write_mem_word(base_pc + 32'h88, enc_r(7'b0000000, 5'd15, 5'd14, F3_ADD_SUB, 5'd15, OPCODE_OP)); // add a5,a4,a5
            write_mem_word(base_pc + 32'h8C, enc_s(12'hFE8, 5'd15, 5'd8, 3'b010, OPCODE_STORE));    // sw a5,-24(s0)
            write_mem_word(base_pc + 32'h90, enc_i(12'hFEC, 5'd8, 3'b010, 5'd15, OPCODE_LOAD));     // lw a5,-20(s0)
            write_mem_word(base_pc + 32'h94, enc_i(12'h001, 5'd15, F3_ADD_SUB, 5'd15, OPCODE_OP_IMM)); // addi a5,a5,1
            write_mem_word(base_pc + 32'h98, enc_s(12'hFEC, 5'd15, 5'd8, 3'b010, OPCODE_STORE));    // sw a5,-20(s0)
            write_mem_word(base_pc + 32'h9C, enc_i(12'hFEC, 5'd8, 3'b010, 5'd14, OPCODE_LOAD));     // lw a4,-20(s0)
            write_mem_word(base_pc + 32'hA0, enc_i(12'hFE4, 5'd8, 3'b010, 5'd15, OPCODE_LOAD));     // lw a5,-28(s0)
            write_mem_word(base_pc + 32'hA4, enc_b(13'h1FD4, 5'd15, 5'd14, F3_BLT, OPCODE_BRANCH)); // blt a4,a5,.L3
            write_mem_word(base_pc + 32'hA8, enc_i(12'hFE8, 5'd8, 3'b010, 5'd15, OPCODE_LOAD));     // lw a5,-24(s0)
            write_mem_word(base_pc + 32'hAC, enc_i(12'h000, 5'd15, F3_ADD_SUB, 5'd10, OPCODE_OP_IMM)); // mv a0,a5
            write_mem_word(base_pc + 32'hB0, enc_i(12'h02C, 5'd2, 3'b010, 5'd1, OPCODE_LOAD));       // lw ra,44(sp)
            write_mem_word(base_pc + 32'hB4, enc_i(12'h028, 5'd2, 3'b010, 5'd8, OPCODE_LOAD));       // lw s0,40(sp)
            write_mem_word(base_pc + 32'hB8, enc_i(12'h030, 5'd2, F3_ADD_SUB, 5'd2, OPCODE_OP_IMM)); // addi sp,sp,48
            write_mem_word(base_pc + 32'hBC, enc_i(12'h000, 5'd1, F3_ADD_SUB, 5'd0, OPCODE_JALR));   // jr ra

            // halt loop (base_pc + 0xC0)
            write_mem_word(base_pc + 32'hC0, enc_j(21'd0, 5'd0, OPCODE_JAL));
        end
    endtask


    task write_prf;
        input [4:0] regnum;
        input [31:0] value;
        begin
            if (regnum != 5'd0) begin
                dut.u_prf.int_prf[regnum] = value;
            end
        end
    endtask

    task write_fprf;
        input [4:0] regnum;
        input [31:0] value;
        begin
            dut.u_prf.float_prf[regnum] = value;
        end
    endtask

    task write_csr;
        input [11:0] addr;
        input [31:0] value;
        begin
            dut.u_issue.csr_file[addr] = value;
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

    task run_fp_test;
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
            write_fprf(rs1, rs1_val);
            write_fprf(rs2, rs2_val);

            force dut.u_if.out_inst_0 = inst;
            force dut.u_if.out_inst_1 = 32'h0000_0013;
            force dut.u_if.out_inst_valid = 2'b01;
            force dut.u_if.out_inst_addr_0 = pc;
            force dut.u_if.out_inst_addr_1 = pc + 32'd4;

            begin : wait_fp_commit
                cycles = 0;
                while (cycles < 200) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (dut.commit0_valid) begin
                        if (!dut.commit0_is_float ||
                            (dut.commit0_arch_rd !== rd) ||
                            (dut.commit0_value !== exp_val)) begin
                            $display("FAIL_FP[%0d] inst=%h rd=%0d exp=%h got_rd=%0d got=%h is_fp=%b",
                                     idx, inst, rd, exp_val, dut.commit0_arch_rd, dut.commit0_value, dut.commit0_is_float);
                        end else begin
                            $display("PASS_FP[%0d] inst=%h rd=%0d val=%h",
                                     idx, inst, rd, dut.commit0_value);
                        end
                        disable wait_fp_commit;
                    end
                end
                $display("TIMEOUT_FP[%0d] inst=%h", idx, inst);
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

    task run_store_task;
        input integer idx;
        input [31:0] inst;
        input [31:0] pc;
        input [4:0]  rs1;
        input [31:0] rs1_val;
        input [4:0]  rs2;
        input [31:0] rs2_val;
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

            begin : wait_store_commit
                cycles = 0;
                while (cycles < 200) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (dut.commit0_valid) begin
                        $display("PASS_STORE[%0d] inst=%h", idx, inst);
                        disable wait_store_commit;
                    end
                end
                $display("TIMEOUT_STORE[%0d] inst=%h", idx, inst);
            end

            release dut.u_if.out_inst_0;
            release dut.u_if.out_inst_1;
            release dut.u_if.out_inst_valid;
            release dut.u_if.out_inst_addr_0;
            release dut.u_if.out_inst_addr_1;
        end
    endtask

    task run_task_test;
        input integer idx;
        input [31:0] start_pc;
        input integer target_commits;
        input [31:0] init_sp;
        input [31:0] init_ra;
        integer cycles;
        integer commit_count;
        integer i;
        integer drain_cycles;
        integer r;
        integer robd;
        reg pass;
        reg head_in_rs;
        reg [31:0] got_reg;
        reg [31:0] got_mem;
        integer last_commit_cycle;
        begin
            rst_n = 1'b0;
            repeat (5) @(posedge clk);

            clear_caches();
            clear_mainmem();
            load_sum_program(start_pc);
            if (idx == DEBUG_TASK_IDX) begin
                $display("DBG_IMEM_INIT[%0d] start_pc=%h", idx, start_pc);
                $display("DBG_IMEM[%0d] %h = %h", idx, start_pc + 32'h0078, peek_imem_word(start_pc + 32'h0078));
                $display("DBG_IMEM[%0d] %h = %h", idx, start_pc + 32'h009C, peek_imem_word(start_pc + 32'h009C));
                $display("DBG_IMEM[%0d] %h = %h", idx, start_pc + 32'h00A4, peek_imem_word(start_pc + 32'h00A4));
                $display("DBG_IMEM[%0d] %h = %h", idx, start_pc + 32'h00AC, peek_imem_word(start_pc + 32'h00AC));
                $display("DBG_IMEM[%0d] %h = %h", idx, start_pc + 32'h00C0, peek_imem_word(start_pc + 32'h00C0));
            end

            rst_n = 1'b1;
            @(negedge clk);
            write_prf(5'd2, init_sp);
            write_prf(5'd1, init_ra);

            if (start_pc != 32'h0000_1000) begin
                force dut.u_if.reg_PC = start_pc;
                force dut.u_if.line_valid = 1'b0;
                @(posedge clk);
                #1;
                release dut.u_if.reg_PC;
                release dut.u_if.line_valid;
            end

            commit_count = 0;
            cycles = 0;
            pass = 1'b1;
            last_commit_cycle = 0;

            begin : wait_task_commit
                while (cycles < 4000 && commit_count < target_commits) begin
                    @(posedge clk);
                    #1;
                    cycles = cycles + 1;
                    if (dut.commit0_valid) begin
                        commit_count = commit_count + 1;
                        last_commit_cycle = cycles;
                        if ((commit_count % TASK_DEBUG_STRIDE) == 0) begin
                            $display("TASK_COMMIT0[%0d] c=%0d pc=%h inst=%h rd=%0d val=%h exc=%b has_dest=%b new_preg=%0d old_preg=%0d",
                                     idx, commit_count, dut.commit0_pc, peek_imem_word(dut.commit0_pc),
                                     dut.commit0_arch_rd, dut.commit0_value, dut.commit0_exception,
                                     dut.commit0_has_dest, dut.commit0_new_preg, dut.commit0_old_preg);
                        end
                        if (dut.commit0_exception) begin
                            pass = 1'b0;
                        end
                    end
                    if (dut.commit1_valid) begin
                        commit_count = commit_count + 1;
                        last_commit_cycle = cycles;
                        if ((commit_count % TASK_DEBUG_STRIDE) == 0) begin
                            $display("TASK_COMMIT1[%0d] c=%0d pc=%h inst=%h rd=%0d val=%h exc=%b has_dest=%b new_preg=%0d old_preg=%0d",
                                     idx, commit_count, dut.commit1_pc, peek_imem_word(dut.commit1_pc),
                                     dut.commit1_arch_rd, dut.commit1_value, dut.commit1_exception,
                                     dut.commit1_has_dest, dut.commit1_new_preg, dut.commit1_old_preg);
                        end
                        if (dut.commit1_exception) begin
                            pass = 1'b0;
                        end
                    end
                    if (idx == DEBUG_TASK_IDX) begin
                        if (dut.global_flush) begin
                            $display("DBG_FLUSH[%0d] cyc=%0d redirect=%b target=%h rob_idx=%0d rob_head=%0d rob_empty=%b",
                                     idx, cycles, dut.redirect_valid, dut.redirect_target,
                                     dut.redirect_rob_idx, dut.rob_head, dut.rob_empty);
                        end
                        if (dut.lsu_valid && !dut.lsu_busy) begin
                            $display("DBG_LSU_REQ[%0d] cyc=%0d load=%b op=%0d addr=%h wdata=%h rob=%0d rd_tag=%0d rd_fp=%b head=%0d",
                                     idx, cycles, dut.lsu_mem_is_load, dut.lsu_mem_op, dut.lsu_addr,
                                     dut.lsu_wdata, dut.lsu_rob_idx, dut.lsu_rd_tag, dut.lsu_rd_is_fp, dut.rob_head);
                        end
                        if (dut.lsu_wb_valid) begin
                            $display("DBG_LSU_WB[%0d] cyc=%0d load=%b addr=%h rdata=%h rob=%0d rd_tag=%0d rd_fp=%b mem_peek=%h",
                                     idx, cycles, dut.u_lsu.mem_is_load_reg, dut.u_lsu.addr_reg,
                                     dut.lsu_wb_value, dut.lsu_wb_rob_idx, dut.lsu_wb_dest_tag,
                                     dut.lsu_wb_dest_is_fp, peek_mem_word(dut.u_lsu.addr_reg));
                        end
                        if (dut.issue_stall && (dut.post_valid != {`IF_BATCH_SIZE{1'b0}})) begin
                            $display("DBG_POST_STALL[%0d] cyc=%0d rs_cnt=%0d post_valid=%b pc0=%h rob0=%0d pc1=%h rob1=%0d",
                                     idx, cycles, dut.u_issue.rs_count, dut.post_valid,
                                     dut.post_pc_0, dut.post_rob_idx_0, dut.post_pc_1, dut.post_rob_idx_1);
                            if (dut.post_valid[0]) begin
                                $display("DBG_POST0[%0d] inst=%h fu=%0d rs1=P%0d rs2=P%0d rd=P%0d imm=%h",
                                         idx, dut.post_inst_0, dut.post_fu_type_0,
                                         dut.post_rs1_preg_0, dut.post_rs2_preg_0, dut.post_rd_preg_0,
                                         dut.post_imm_0);
                            end
                            if (dut.post_valid[1]) begin
                                $display("DBG_POST1[%0d] inst=%h fu=%0d rs1=P%0d rs2=P%0d rd=P%0d imm=%h",
                                         idx, dut.post_inst_1, dut.post_fu_type_1,
                                         dut.post_rs1_preg_1, dut.post_rs2_preg_1, dut.post_rd_preg_1,
                                         dut.post_imm_1);
                            end
                        end

                        if ((dut.u_if.out_inst_valid != 2'b00) &&
                            (is_debug_pc(dut.u_if.out_inst_addr_0) || is_debug_pc(dut.u_if.out_inst_addr_1))) begin
                            $display("DBG_IF[%0d] cyc=%0d pc0=%h v0=%b inst0=%h pred0=%b tgt0=%h pc1=%h v1=%b inst1=%h pred1=%b tgt1=%h if_pc=%h stall_if=%b ic_stall=%b",
                                     idx, cycles,
                                     dut.u_if.out_inst_addr_0, dut.u_if.out_inst_valid[0], dut.u_if.out_inst_0,
                                     dut.u_if.out_pred_taken_0, dut.u_if.out_pred_target_0,
                                     dut.u_if.out_inst_addr_1, dut.u_if.out_inst_valid[1], dut.u_if.out_inst_1,
                                     dut.u_if.out_pred_taken_1, dut.u_if.out_pred_target_1,
                                     dut.u_if.reg_PC, dut.ib_stall_if, dut.ic_stall);
                        end

                        if ((dut.u_ibuf.out_valid != 2'b00) &&
                            (is_debug_pc(dut.u_ibuf.out_pc_0) || is_debug_pc(dut.u_ibuf.out_pc_1))) begin
                            $display("DBG_IB[%0d] cyc=%0d pc0=%h v0=%b inst0=%h pc1=%h v1=%b inst1=%h out_ready=%b",
                                     idx, cycles,
                                     dut.u_ibuf.out_pc_0, dut.u_ibuf.out_valid[0], dut.u_ibuf.out_inst_0,
                                     dut.u_ibuf.out_pc_1, dut.u_ibuf.out_valid[1], dut.u_ibuf.out_inst_1,
                                     dut.dispatch_ready);
                        end

                        if ((dut.pre_valid != 2'b00) &&
                            (is_debug_pc(dut.pre_pc_0) || is_debug_pc(dut.pre_pc_1))) begin
                            $display("DBG_PRE[%0d] cyc=%0d pc0=%h v0=%b inst0=%h pc1=%h v1=%b inst1=%h",
                                     idx, cycles,
                                     dut.pre_pc_0, dut.pre_valid[0], dut.pre_inst_0,
                                     dut.pre_pc_1, dut.pre_valid[1], dut.pre_inst_1);
                        end

                        if ((dut.rn_valid != 2'b00) &&
                            (is_debug_pc(dut.rn_pc_0) || is_debug_pc(dut.rn_pc_1))) begin
                            $display("DBG_RN[%0d] cyc=%0d pc0=%h v0=%b inst0=%h rob0=%0d pc1=%h v1=%b inst1=%h rob1=%0d rn_stall=%b",
                                     idx, cycles,
                                     dut.rn_pc_0, dut.rn_valid[0], dut.rn_inst_0, dut.rn_rob_idx_0,
                                     dut.rn_pc_1, dut.rn_valid[1], dut.rn_inst_1, dut.rn_rob_idx_1,
                                     dut.rn_stall);
                        end

                        if ((dut.post_valid != 2'b00) &&
                            (is_debug_pc(dut.post_pc_0) || is_debug_pc(dut.post_pc_1))) begin
                            $display("DBG_POST[%0d] cyc=%0d pc0=%h v0=%b inst0=%h fu0=%0d rob0=%0d pc1=%h v1=%b inst1=%h fu1=%0d rob1=%0d issue_stall=%b",
                                     idx, cycles,
                                     dut.post_pc_0, dut.post_valid[0], dut.post_inst_0, dut.post_fu_sel_0, dut.post_rob_idx_0,
                                     dut.post_pc_1, dut.post_valid[1], dut.post_inst_1, dut.post_fu_sel_1, dut.post_rob_idx_1,
                                     dut.issue_stall);
                        end

                        if (dut.u_issue.rs_count != 0) begin
                            for (r = 0; r < dut.u_issue.RS_DEPTH; r = r + 1) begin
                                if (dut.u_issue.rs_valid[r] && is_debug_pc(dut.u_issue.rs_pc[r])) begin
                                    $display("DBG_RS_PC[%0d] cyc=%0d idx=%0d pc=%h rob=%0d fu=%0d rdy1=%b rdy2=%b rs1=P%0d rs2=P%0d rd=P%0d",
                                             idx, cycles, r, dut.u_issue.rs_pc[r], dut.u_issue.rs_rob_idx[r],
                                             dut.u_issue.rs_fu_sel[r], dut.u_issue.rs_rs1_ready[r], dut.u_issue.rs_rs2_ready[r],
                                             dut.u_issue.rs_rs1_tag[r], dut.u_issue.rs_rs2_tag[r], dut.u_issue.rs_rd_tag[r]);
                                end
                            end
                        end

                        if ((cycles >= 70) && (cycles <= 220)) begin
                            if (dut.u_issue.mem_issue_idx != -1) begin
                                $display("DBG_LSU_PICK[%0d] cyc=%0d mem_idx=%0d pc=%h rob=%0d age=%0d ready=%b is_store=%b lsu_can_issue=%b lsu_busy=%b lsu_pending=%b rob_head=%0d head_pc=%h",
                                         idx, cycles, dut.u_issue.mem_issue_idx,
                                         dut.u_issue.rs_pc[dut.u_issue.mem_issue_idx],
                                         dut.u_issue.rs_rob_idx[dut.u_issue.mem_issue_idx],
                                         dut.u_issue.rs_age[dut.u_issue.mem_issue_idx],
                                         (dut.u_issue.rs_rs1_ready[dut.u_issue.mem_issue_idx] && dut.u_issue.rs_rs2_ready[dut.u_issue.mem_issue_idx]),
                                         !dut.u_issue.rs_mem_is_load[dut.u_issue.mem_issue_idx],
                                         dut.u_issue.lsu_can_issue, dut.u_issue.lsu_busy, dut.u_issue.lsu_pending,
                                         dut.rob_head,
                                         dut.u_regrename.u_rob.entry_pc[dut.rob_head]);
                            end else begin
                                $display("DBG_LSU_PICK[%0d] cyc=%0d mem_idx=-1 lsu_can_issue=%b lsu_busy=%b lsu_pending=%b rob_head=%0d head_pc=%h",
                                         idx, cycles, dut.u_issue.lsu_can_issue, dut.u_issue.lsu_busy, dut.u_issue.lsu_pending,
                                         dut.rob_head, dut.u_regrename.u_rob.entry_pc[dut.rob_head]);
                            end
                            if ((dut.u_issue.mem_issue_idx != -1) &&
                                !dut.u_issue.rs_mem_is_load[dut.u_issue.mem_issue_idx] &&
                                !(dut.u_issue.rs_rs1_ready[dut.u_issue.mem_issue_idx] && dut.u_issue.rs_rs2_ready[dut.u_issue.mem_issue_idx]) &&
                                (dut.u_regrename.u_rob.entry_pc[dut.rob_head] == 32'h0000_1060)) begin
                                $display("DBG_LSU_CAND[%0d] cyc=%0d head=%0d head_pc=%h (store picked but not ready)",
                                         idx, cycles, dut.rob_head, dut.u_regrename.u_rob.entry_pc[dut.rob_head]);
                                for (r = 0; r < dut.u_issue.RS_DEPTH; r = r + 1) begin
                                    if (dut.u_issue.rs_valid[r] && (dut.u_issue.rs_fu_sel[r] == `FU_DEC_LSU)) begin
                                        if (dut.u_issue.rs_rob_idx[r] >= dut.rob_head) robd = dut.u_issue.rs_rob_idx[r] - dut.rob_head;
                                        else robd = dut.u_issue.rs_rob_idx[r] + 32 - dut.rob_head;
                                        $display("  LSU_RS idx=%0d pc=%h rob=%0d dist=%0d ready=%b is_load=%b rs1=P%0d rs2=P%0d rd=P%0d",
                                                 r, dut.u_issue.rs_pc[r], dut.u_issue.rs_rob_idx[r], robd,
                                                 (dut.u_issue.rs_rs1_ready[r] && dut.u_issue.rs_rs2_ready[r]),
                                                 dut.u_issue.rs_mem_is_load[r],
                                                 dut.u_issue.rs_rs1_tag[r], dut.u_issue.rs_rs2_tag[r], dut.u_issue.rs_rd_tag[r]);
                                    end
                                end
                            end
                        end
                    end
                    if (dut.redirect_valid) begin
                        $display("TASK_REDIRECT[%0d] cyc=%0d target=%h inst=%h if_pc=%h",
                                 idx, cycles, dut.redirect_target, peek_imem_word(dut.redirect_target),
                                 dut.u_if.reg_PC);
                        if (idx == DEBUG_TASK_IDX) begin
                            $display("DBG_REDIRECT_STATE[%0d] issue0=%0d issue1=%0d br0=%b br1=%b br0_mis=%b br1_mis=%b",
                                     idx, dut.u_issue.issue_idx0, dut.u_issue.issue_idx1,
                                     dut.u_issue.br0_is_branch, dut.u_issue.br1_is_branch,
                                     dut.u_issue.br0_mispredict, dut.u_issue.br1_mispredict);
                            $display("DBG_REDIRECT_ROB[%0d] rob_idx=%0d valid=%b ready=%b pc=%h has_dest=%b arch_rd=%0d new_preg=%0d old_preg=%0d",
                                     idx, dut.redirect_rob_idx,
                                     dut.u_regrename.u_rob.entry_valid[dut.redirect_rob_idx],
                                     dut.u_regrename.u_rob.entry_ready[dut.redirect_rob_idx],
                                     dut.u_regrename.u_rob.entry_pc[dut.redirect_rob_idx],
                                     dut.u_regrename.u_rob.entry_has_dest[dut.redirect_rob_idx],
                                     dut.u_regrename.u_rob.entry_arch_rd[dut.redirect_rob_idx],
                                     dut.u_regrename.u_rob.entry_new_preg[dut.redirect_rob_idx],
                                     dut.u_regrename.u_rob.entry_old_preg[dut.redirect_rob_idx]);
                            if (dut.u_issue.issue_idx0 != -1 && dut.u_issue.br0_is_branch) begin
                                $display("DBG_BR0[%0d] pc=%h inst=%h imm=%h op=%0d rs1=%h rs2=%h taken=%b target=%h pred_taken=%b pred_target=%h",
                                         idx, dut.u_issue.rs_pc[dut.u_issue.issue_idx0],
                                         dut.u_issue.rs_inst[dut.u_issue.issue_idx0],
                                         dut.u_issue.rs_imm[dut.u_issue.issue_idx0],
                                         dut.u_issue.rs_branch_op[dut.u_issue.issue_idx0],
                                         dut.u_issue.rs_rs1_val[dut.u_issue.issue_idx0],
                                         dut.u_issue.rs_rs2_val[dut.u_issue.issue_idx0],
                                         dut.u_issue.br0_taken, dut.u_issue.br0_target,
                                         dut.u_issue.rs_pred_taken[dut.u_issue.issue_idx0],
                                         dut.u_issue.rs_pred_target[dut.u_issue.issue_idx0]);
                            end
                            if (dut.u_issue.issue_idx1 != -1 && dut.u_issue.br1_is_branch) begin
                                $display("DBG_BR1[%0d] pc=%h inst=%h imm=%h op=%0d rs1=%h rs2=%h taken=%b target=%h pred_taken=%b pred_target=%h",
                                         idx, dut.u_issue.rs_pc[dut.u_issue.issue_idx1],
                                         dut.u_issue.rs_inst[dut.u_issue.issue_idx1],
                                         dut.u_issue.rs_imm[dut.u_issue.issue_idx1],
                                         dut.u_issue.rs_branch_op[dut.u_issue.issue_idx1],
                                         dut.u_issue.rs_rs1_val[dut.u_issue.issue_idx1],
                                         dut.u_issue.rs_rs2_val[dut.u_issue.issue_idx1],
                                         dut.u_issue.br1_taken, dut.u_issue.br1_target,
                                         dut.u_issue.rs_pred_taken[dut.u_issue.issue_idx1],
                                         dut.u_issue.rs_pred_target[dut.u_issue.issue_idx1]);
                            end
                            $display("DBG_IMEM_AROUND[%0d] %h=%h %h=%h %h=%h",
                                     idx,
                                     start_pc + 32'h0078, peek_imem_word(start_pc + 32'h0078),
                                     start_pc + 32'h00A4, peek_imem_word(start_pc + 32'h00A4),
                                     start_pc + 32'h00BC, peek_imem_word(start_pc + 32'h00BC));
                            $display("DBG_ARCH_REGS[%0d] ra=%h sp=%h s0=%h a0=%h a4=%h a5=%h",
                                     idx,
                                     peek_arch_reg(5'd1), peek_arch_reg(5'd2), peek_arch_reg(5'd8),
                                     peek_arch_reg(5'd10), peek_arch_reg(5'd14), peek_arch_reg(5'd15));
                        end
                    end
                    if (idx == DEBUG_TASK_IDX && dut.u_issue.lsu_valid && !dut.u_issue.lsu_mem_is_load) begin
                        if (dut.u_issue.lsu_addr >= 32'h0000_1000 && dut.u_issue.lsu_addr < 32'h0000_1100) begin
                            $display("DBG_CODE_STORE[%0d] cyc=%0d addr=%h wdata=%h mem_op=%0d",
                                     idx, cycles, dut.u_issue.lsu_addr, dut.u_issue.lsu_wdata,
                                     dut.u_issue.lsu_mem_op);
                        end
                    end
                end
                if (commit_count < target_commits) begin
                    $display("TIMEOUT_TASK[%0d] commits=%0d/%0d", idx, commit_count, target_commits);
                    $display("DBG_TASK[%0d] cyc=%0d last_commit=%0d if=%b ib=%b pre=%b rn=%b post=%b rs_cnt=%0d rob_empty=%b rn_stall=%b issue_stall=%b disp_ready=%b flush=%b redirect=%b pc=%h",
                             idx, cycles, last_commit_cycle,
                             dut.if_inst_valid, dut.ib_valid, dut.pre_valid, dut.rn_valid, dut.post_valid,
                             dut.u_issue.rs_count, dut.rob_empty, dut.rn_stall, dut.issue_stall, dut.dispatch_ready,
                             dut.global_flush, dut.redirect_valid, dut.u_if.reg_PC);
                    if (idx == DEBUG_TASK_IDX) begin
                        head_in_rs = 1'b0;
                        for (r = 0; r < dut.u_issue.RS_DEPTH; r = r + 1) begin
                            if (dut.u_issue.rs_valid[r] && (dut.u_issue.rs_rob_idx[r] == dut.rob_head)) begin
                                head_in_rs = 1'b1;
                            end
                        end
                        $display("DBG_MAP[%0d] sp=R2->P%0d val=%h ready=%b s0=R8->P%0d val=%h ready=%b a0=R10->P%0d val=%h ready=%b a1=R11->P%0d val=%h ready=%b a4=R14->P%0d val=%h ready=%b a5=R15->P%0d val=%h ready=%b",
                                 idx,
                                 dut.u_regrename.int_map[5'd2], dut.u_prf.int_prf[dut.u_regrename.int_map[5'd2]], dut.u_regrename.int_preg_ready[dut.u_regrename.int_map[5'd2]],
                                 dut.u_regrename.int_map[5'd8], dut.u_prf.int_prf[dut.u_regrename.int_map[5'd8]], dut.u_regrename.int_preg_ready[dut.u_regrename.int_map[5'd8]],
                                 dut.u_regrename.int_map[5'd10], dut.u_prf.int_prf[dut.u_regrename.int_map[5'd10]], dut.u_regrename.int_preg_ready[dut.u_regrename.int_map[5'd10]],
                                 dut.u_regrename.int_map[5'd11], dut.u_prf.int_prf[dut.u_regrename.int_map[5'd11]], dut.u_regrename.int_preg_ready[dut.u_regrename.int_map[5'd11]],
                                 dut.u_regrename.int_map[5'd14], dut.u_prf.int_prf[dut.u_regrename.int_map[5'd14]], dut.u_regrename.int_preg_ready[dut.u_regrename.int_map[5'd14]],
                                 dut.u_regrename.int_map[5'd15], dut.u_prf.int_prf[dut.u_regrename.int_map[5'd15]], dut.u_regrename.int_preg_ready[dut.u_regrename.int_map[5'd15]]);
                        $display("DBG_ROB_HEAD[%0d] head=%0d valid=%b ready=%b pc=%h has_dest=%b arch_rd=%0d new_preg=%0d old_preg=%0d",
                                 idx, dut.rob_head,
                                 dut.u_regrename.u_rob.entry_valid[dut.rob_head],
                                 dut.u_regrename.u_rob.entry_ready[dut.rob_head],
                                 dut.u_regrename.u_rob.entry_pc[dut.rob_head],
                                 dut.u_regrename.u_rob.entry_has_dest[dut.rob_head],
                                 dut.u_regrename.u_rob.entry_arch_rd[dut.rob_head],
                                 dut.u_regrename.u_rob.entry_new_preg[dut.rob_head],
                                 dut.u_regrename.u_rob.entry_old_preg[dut.rob_head]);
                        $display("DBG_HEAD_IN_RS[%0d] head=%0d in_rs=%b", idx, dut.rob_head, head_in_rs);
                        for (r = 0; r < dut.u_issue.RS_DEPTH; r = r + 1) begin
                            if (dut.u_issue.rs_valid[r]) begin
                                $display("DBG_RS[%0d] idx=%0d pc=%h inst=%h rob=%0d fu=%0d rs1=P%0d rdy=%b rs2=P%0d rdy=%b imm=%h",
                                         idx, r, dut.u_issue.rs_pc[r], dut.u_issue.rs_inst[r], dut.u_issue.rs_rob_idx[r],
                                         dut.u_issue.rs_fu_sel[r], dut.u_issue.rs_rs1_tag[r], dut.u_issue.rs_rs1_ready[r],
                                         dut.u_issue.rs_rs2_tag[r], dut.u_issue.rs_rs2_ready[r], dut.u_issue.rs_imm[r]);
                                $display("DBG_RS_RDY[%0d] idx=%0d rs1_preg_ready=%b rs2_preg_ready=%b",
                                         idx, r,
                                         dut.u_regrename.int_preg_ready[dut.u_issue.rs_rs1_tag[r]],
                                         dut.u_regrename.int_preg_ready[dut.u_issue.rs_rs2_tag[r]]);
                            end
                        end
                    end
                    pass = 1'b0;
                end
            end

            force dut.u_if.out_inst_valid = 2'b00;
            force dut.u_if.out_inst_0 = 32'h0000_0013;
            force dut.u_if.out_inst_1 = 32'h0000_0013;

            drain_cycles = 0;
            while (drain_cycles < 200 && !(dut.rob_empty && (dut.u_issue.rs_count == 0) && !dut.u_lsu.busy)) begin
                @(posedge clk);
                #1;
                drain_cycles = drain_cycles + 1;
            end

            for (i = 0; i < task_reg_count; i = i + 1) begin
                got_reg = peek_arch_reg(task_reg_num[i]);
                if (got_reg !== task_reg_val[i]) begin
                    $display("FAIL_TASK_REG[%0d] reg=%0d exp=%h got=%h",
                             idx, task_reg_num[i], task_reg_val[i], got_reg);
                    pass = 1'b0;
                end
            end

            for (i = 0; i < task_mem_count; i = i + 1) begin
                got_mem = peek_mem_word(task_mem_addr[i]);
                if (got_mem !== task_mem_val[i]) begin
                    $display("FAIL_TASK_MEM[%0d] addr=%h exp=%h got=%h",
                             idx, task_mem_addr[i], task_mem_val[i], got_mem);
                    pass = 1'b0;
                end
            end

            if (pass) begin
                $display("PASS_TASK[%0d] commits=%0d", idx, commit_count);
            end else begin
                $display("FAIL_TASK[%0d] commits=%0d", idx, commit_count);
            end

            release dut.u_if.out_inst_valid;
            release dut.u_if.out_inst_0;
            release dut.u_if.out_inst_1;
        end
    endtask

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // TAG: Test case start
    initial begin
        // Initialize a memory line for load test (addr 0x0000_0040 -> idx 0x04)
        dut.u_mainmem.ram[8'h04] = 128'h00000000_00000000_00000000_DEADBEEF;

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

        // LW (from addr 0x0000_0040 -> 0xDEADBEEF)
        run_test(21,
                 enc_i(12'h000, 5'd1, 3'b010, 5'd24, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd24, 5'd1, 32'h0000_0040, 5'd0, 32'd0, 32'hDEAD_BEEF);

        // BEQ taken -> redirect to PC+8
        run_redirect_test(22,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BEQ, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd5, 5'd2, 32'd5, 32'h0000_1008);

        // BNE taken
        run_redirect_test(23,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BNE, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd5, 5'd2, 32'd7, 32'h0000_1008);

        // BLT taken
        run_redirect_test(24,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BLT, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'hFFFF_FFFF, 5'd2, 32'd1, 32'h0000_1008);

        // BGE taken
        run_redirect_test(25,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BGE, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd1, 5'd2, 32'hFFFF_FFFF, 32'h0000_1008);

        // BLTU taken
        run_redirect_test(26,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BLTU, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'd1, 5'd2, 32'd2, 32'h0000_1008);

        // BGEU taken
        run_redirect_test(27,
                 enc_b(13'd8, 5'd2, 5'd1, F3_BGEU, OPCODE_BRANCH),
                 32'h0000_1000,
                 5'd1, 32'hFFFF_FFFF, 5'd2, 32'd1, 32'h0000_1008);

        // JAL -> redirect to PC+16
        run_redirect_link_test(28,
                 enc_j(21'd16, 5'd1, OPCODE_JAL),
                 32'h0000_1000,
                 5'd0, 32'd0, 5'd0, 32'd0, 32'h0000_1010, 32'h0000_1004);

        // JALR -> redirect to (rs1+4)&~1
        run_redirect_link_test(29,
                 enc_i(12'd4, 5'd5, F3_ADD_SUB, 5'd1, OPCODE_JALR),
                 32'h0000_1000,
                 5'd5, 32'h0000_2000, 5'd0, 32'd0, 32'h0000_2004, 32'h0000_1004);

        // FENCE
        run_noeffect_test(30,
                 enc_i(12'd0, 5'd0, 3'b000, 5'd0, OPCODE_MISC_MEM),
                 32'h0000_1000);

        // FENCE.I
        run_noeffect_test(31,
                 enc_i(12'd0, 5'd0, 3'b001, 5'd0, OPCODE_MISC_MEM),
                 32'h0000_1000);

        // ECALL
        run_noeffect_test(32,
                 enc_i(12'd0, 5'd0, 3'b000, 5'd0, OPCODE_SYSTEM),
                 32'h0000_1000);

        // EBREAK
        run_noeffect_test(33,
                 enc_i(12'd1, 5'd0, 3'b000, 5'd0, OPCODE_SYSTEM),
                 32'h0000_1000);

        // Dual-issue: independent adds
        run_dual_test(34,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_ADD_SUB, 5'd3, OPCODE_OP),
                 enc_r(7'b0000000, 5'd6, 5'd5, F3_ADD_SUB, 5'd4, OPCODE_OP),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'd7, 5'd2, 32'd3, 32'd10,
                 5'd4, 5'd5, 32'd11, 5'd6, 32'd22, 32'd33);

        // Dual-issue: RAW dependency (rd0 -> rs1 of inst1)
        run_dual_test(35,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_ADD_SUB, 5'd7, OPCODE_OP),
                 enc_r(7'b0000000, 5'd3, 5'd7, F3_ADD_SUB, 5'd8, OPCODE_OP),
                 32'h0000_1000,
                 5'd7, 5'd1, 32'd5, 5'd2, 32'd6, 32'd11,
                 5'd8, 5'd7, 32'd0, 5'd3, 32'd4, 32'd15);

        // Dual-issue: WAW to same rd
        run_dual_test(36,
                 enc_r(7'b0000000, 5'd2, 5'd1, F3_ADD_SUB, 5'd9, OPCODE_OP),
                 enc_r(7'b0000000, 5'd4, 5'd3, F3_ADD_SUB, 5'd9, OPCODE_OP),
                 32'h0000_1000,
                 5'd9, 5'd1, 32'd5, 5'd2, 32'd6, 32'd11,
                 5'd9, 5'd3, 32'd2, 5'd4, 32'd3, 32'd5);

        // FADD.S (1.0 + 2.0 = 3.0)
        run_fp_test(37,
                 enc_r(7'b0000000, 5'd2, 5'd1, RM_RNE, 5'd3, OPCODE_OP_FP),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'h3F80_0000, 5'd2, 32'h4000_0000, 32'h4040_0000);

        // FSUB.S (4.0 - 1.0 = 3.0)
        run_fp_test(38,
                 enc_r(7'b0000100, 5'd5, 5'd4, RM_RNE, 5'd6, OPCODE_OP_FP),
                 32'h0000_1000,
                 5'd6, 5'd4, 32'h4080_0000, 5'd5, 32'h3F80_0000, 32'h4040_0000);

        // FMUL.S (2.0 * 0.5 = 1.0)
        run_fp_test(39,
                 enc_r(7'b0001000, 5'd8, 5'd7, RM_RNE, 5'd9, OPCODE_OP_FP),
                 32'h0000_1000,
                 5'd9, 5'd7, 32'h4000_0000, 5'd8, 32'h3F00_0000, 32'h3F80_0000);

        // --- Memory Access Extended Tests ---

        // LB (from 0x40 -> 0xEF -> sign extended)
        run_test(40,
                 enc_i(12'h000, 5'd1, 3'b000, 5'd2, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd2, 5'd1, 32'h0000_0040, 5'd0, 32'd0, 32'hFFFF_FFEF);

        // LH (from 0x40 -> 0xBEEF -> sign extended)
        run_test(41,
                 enc_i(12'h000, 5'd1, 3'b001, 5'd3, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'h0000_0040, 5'd0, 32'd0, 32'hFFFF_BEEF);

        // LBU (from 0x40 -> 0xEF -> zero extended)
        run_test(42,
                 enc_i(12'h000, 5'd1, 3'b100, 5'd4, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd4, 5'd1, 32'h0000_0040, 5'd0, 32'd0, 32'h0000_00EF);

        // LHU (from 0x40 -> 0xBEEF -> zero extended)
        run_test(43,
                 enc_i(12'h000, 5'd1, 3'b101, 5'd5, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd5, 5'd1, 32'h0000_0040, 5'd0, 32'd0, 32'h0000_BEEF);

        // Store-Load Tests
        // SB 0x88 at 0x50
        run_store_task(44,
                 enc_s(12'h000, 5'd2, 5'd1, 3'b000, OPCODE_STORE),
                 32'h0000_1000,
                 5'd1, 32'h0000_0050, 5'd2, 32'h0000_0088);
        // LBU 0x50 -> 0x88
        run_test(45,
                 enc_i(12'h000, 5'd1, 3'b100, 5'd3, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'h0000_0050, 5'd0, 32'd0, 32'h0000_0088);

        // SH 0x7766 at 0x52
        run_store_task(46,
                 enc_s(12'h000, 5'd2, 5'd1, 3'b001, OPCODE_STORE),
                 32'h0000_1000,
                 5'd1, 32'h0000_0052, 5'd2, 32'h0000_7766);
        // LHU 0x52 -> 0x7766
        run_test(47,
                 enc_i(12'h000, 5'd1, 3'b101, 5'd3, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'h0000_0052, 5'd0, 32'd0, 32'h0000_7766);

        // SW 0x12345678 at 0x54
        run_store_task(48,
                 enc_s(12'h000, 5'd2, 5'd1, 3'b010, OPCODE_STORE),
                 32'h0000_1000,
                 5'd1, 32'h0000_0054, 5'd2, 32'h1234_5678);
        // LW 0x54 -> 0x12345678
        run_test(49,
                 enc_i(12'h000, 5'd1, 3'b010, 5'd3, OPCODE_LOAD),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'h0000_0054, 5'd0, 32'd0, 32'h1234_5678);

        // --- Zmmul Extension Tests ---

        // MUL (20 * 10 = 200)
        run_test(50,
                 enc_r(7'b0000001, 5'd2, 5'd1, F3_MUL, 5'd3, OPCODE_OP),
                 32'h0000_1000,
                 5'd3, 5'd1, 32'd20, 5'd2, 32'd10, 32'd200);

        // MULH (-2^31 * (2^31 - 1) -> High: 0xC0000000)
        run_test(51,
                 enc_r(7'b0000001, 5'd2, 5'd1, F3_MULH, 5'd4, OPCODE_OP),
                 32'h0000_1000,
                 5'd4, 5'd1, 32'h8000_0000, 5'd2, 32'h7FFF_FFFF, 32'hC000_0000);

        // MULHSU (-1 * 2^31 -> High: 0xFFFFFFFF)
        run_test(52,
                 enc_r(7'b0000001, 5'd2, 5'd1, F3_MULHSU, 5'd5, OPCODE_OP),
                 32'h0000_1000,
                 5'd5, 5'd1, 32'hFFFF_FFFF, 5'd2, 32'h8000_0000, 32'hFFFF_FFFF);

        // MULHU (2^31 * 2^31 -> High: 0x40000000)
        run_test(53,
                 enc_r(7'b0000001, 5'd2, 5'd1, F3_MULHU, 5'd6, OPCODE_OP),
                 32'h0000_1000,
                 5'd6, 5'd1, 32'h8000_0000, 5'd2, 32'h8000_0000, 32'h4000_0000);

        // --- Zicsr Extension Tests ---
        
        // Initialize CSR 0x300 (mstatus) for read-modify-write tests
        write_csr(12'h300, 32'h1234_5678);

        // CSRRW: Write 0xDEADBEEF to 0x300, Read old 0x12345678 to rd=10
        run_test(60,
                 enc_i(12'h300, 5'd1, F3_CSRRW, 5'd10, OPCODE_SYSTEM),
                 32'h0000_1000,
                 5'd10, 5'd1, 32'hDEAD_BEEF, 5'd0, 32'd0, 32'h1234_5678);

        // CSRRS: Read 0x300 (now DEADBEEF), Set 0x0000FFFF. Result in CSR: DEADFFFF. Rd gets old DEADBEEF.
        run_test(61,
                 enc_i(12'h300, 5'd1, F3_CSRRS, 5'd11, OPCODE_SYSTEM),
                 32'h0000_1000,
                 5'd11, 5'd1, 32'h0000_FFFF, 5'd0, 32'd0, 32'hDEAD_BEEF);

        // CSRRC: Read 0x300 (now DEADFFFF), Clear 0x0000000F. Result in CSR: DEADFFF0. Rd gets old DEADFFFF.
        run_test(62,
                 enc_i(12'h300, 5'd1, F3_CSRRC, 5'd12, OPCODE_SYSTEM),
                 32'h0000_1000,
                 5'd12, 5'd1, 32'h0000_000F, 5'd0, 32'd0, 32'hDEAD_FFFF);

        // Check final value of 0x300 via CSRRW (Write 0, Read old)
        run_test(63,
                 enc_i(12'h300, 5'd0, F3_CSRRW, 5'd13, OPCODE_SYSTEM),
                 32'h0000_1000,
                 5'd13, 5'd0, 32'd0, 5'd0, 32'd0, 32'hDEAD_FFF0);

        // Initialize CSR 0x301 for Immediate tests
        write_csr(12'h301, 32'h0000_0010);

        // CSRRWI: Write uimm=5 to 0x301. Rd gets old 0x10.
        // imm field is 0x301. rs1 field holds uimm=5.
        run_test(64,
                 enc_i(12'h301, 5'd5, F3_CSRRWI, 5'd14, OPCODE_SYSTEM),
                 32'h0000_1000,
                 5'd14, 5'd0, 32'd0, 5'd0, 32'd0, 32'h0000_0010);

        // CSRRSI: Read 0x301 (now 5), Set uimm=2. Result 7. Rd gets 5.
        run_test(65,
                 enc_i(12'h301, 5'd2, F3_CSRRSI, 5'd15, OPCODE_SYSTEM),
                 32'h0000_1000,
                 5'd15, 5'd0, 32'd0, 5'd0, 32'd0, 32'd5);

        // CSRRCI: Read 0x301 (now 7), Clear uimm=1. Result 6. Rd gets 7.
        run_test(66,
                 enc_i(12'h301, 5'd1, F3_CSRRCI, 5'd16, OPCODE_SYSTEM),
                 32'h0000_1000,
                 5'd16, 5'd0, 32'd0, 5'd0, 32'd0, 32'd7);

        // --- Task-level Test: sum(int,int) ---
        task_reg_count = 0;
        task_mem_count = 0;
        task_reg_num[task_reg_count] = 5'd10; // a0
        task_reg_val[task_reg_count] = 32'd145;
        task_reg_count = task_reg_count + 1;
        task_reg_num[task_reg_count] = 5'd2; // sp
        task_reg_val[task_reg_count] = 32'h0000_0F00;
        task_reg_count = task_reg_count + 1;
        task_mem_addr[task_mem_count] = 32'h0000_0EE4; // s0-28
        task_mem_val[task_mem_count] = 32'd145;
        task_mem_count = task_mem_count + 1;

        run_task_test(70, 32'h0000_1000, 48, 32'h0000_0F00, 32'h0000_10C0);

        $finish;
    end
endmodule
