`timescale 1ns/1ps

// Top_tb_bin.v — Testbench for loading and running a compiled RISC-V .bin file.
//
// Usage (via xsim plusargs):
//   +BIN=<absolute/path/to/program.bin>   (required)
//   +TIMEOUT=<cycle_count>                 (optional, default 200000)
//
// Exit codes written to tohost (address 0x1FF0 in MainMemory):
//   0        = PASS
//   nonzero  = FAIL (value is the program's exit code)
//
// Address map (must match sw/link.ld):
//   0x1000–0x1FEF  program text/data/bss/stack
//   0x1FE0         putchar — testbench intercepts writes and prints the byte
//   0x1FF0         tohost  — testbench detects write and terminates simulation

module Top_tb_bin;

    // -------------------------------------------------------------------------
    // Clock & reset
    // -------------------------------------------------------------------------
    reg clk;
    reg rst_n;

    initial clk = 0;
    always #5 clk = ~clk;  // 10 ns period = 100 MHz

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    Top dut (
        .clk  (clk),
        .rst_n(rst_n)
    );

    // -------------------------------------------------------------------------
    // Cache geometry (must match Top.v / Cache.v parameters)
    // -------------------------------------------------------------------------
    localparam integer CACHE_INDEX_BITS = 6;
    localparam integer CACHE_TAG_BITS   = 22;
    localparam integer CACHE_SETS       = 1 << CACHE_INDEX_BITS;

    // -------------------------------------------------------------------------
    // tohost / putchar detection
    //   tohost  @ 0x1FF0  →  ram[0xFF], word offset 0  (bits[31:0])
    //   putchar @ 0x1FE0  →  ram[0xFE], word offset 0  (bits[7:0])
    // -------------------------------------------------------------------------
    localparam [7:0] TOHOST_RAM_IDX  = 8'hFF;
    localparam [7:0] PUTCHAR_RAM_IDX = 8'hFE;

    reg        tohost_written;
    reg [31:0] tohost_value;
    reg [31:0] sim_cycles;

    localparam integer TRACE_DEPTH = 24;
    integer trace_wr;
    integer trace_count;
    integer trace_wr_tmp;
    reg [31:0] trace_cycle [0:TRACE_DEPTH-1];
    reg        trace_slot  [0:TRACE_DEPTH-1];
    reg [31:0] trace_pc    [0:TRACE_DEPTH-1];
    reg        trace_has_dest [0:TRACE_DEPTH-1];
    reg [4:0]  trace_arch_rd  [0:TRACE_DEPTH-1];
    reg [31:0] trace_value    [0:TRACE_DEPTH-1];
    reg        trace_exception [0:TRACE_DEPTH-1];

    // Main flow controls are declared here so debug tasks can reference them.
    reg [2047:0] bin_file;
    integer      timeout_cycles;
    integer      cycle_count;

    // Monitor MainMemory write completions for tohost/putchar.
    // IMPORTANT: MemArbiter drives mem_req=0/mem_we=0 while waiting for memory
    // (SERVE_D/SERVE_I states), so we must use MainMemory's internal latched
    // registers txn_we/txn_addr/txn_wdata which stay valid until the next request.
    always @(posedge clk) begin
        if (!rst_n) begin
            sim_cycles = 32'd0;
            trace_wr   = 0;
            trace_count = 0;
        end else begin
            sim_cycles = sim_cycles + 1;

            trace_wr_tmp = trace_wr;
            if (dut.commit0_valid) begin
                trace_cycle[trace_wr_tmp]     = sim_cycles;
                trace_slot[trace_wr_tmp]      = 1'b0;
                trace_pc[trace_wr_tmp]        = dut.commit0_pc;
                trace_has_dest[trace_wr_tmp]  = dut.commit0_has_dest;
                trace_arch_rd[trace_wr_tmp]   = dut.commit0_arch_rd;
                trace_value[trace_wr_tmp]     = dut.commit0_value;
                trace_exception[trace_wr_tmp] = dut.commit0_exception;
                if (trace_wr_tmp == TRACE_DEPTH - 1) trace_wr_tmp = 0;
                else                                 trace_wr_tmp = trace_wr_tmp + 1;
                if (trace_count < TRACE_DEPTH) trace_count = trace_count + 1;
            end
            if (dut.commit1_valid) begin
                trace_cycle[trace_wr_tmp]     = sim_cycles;
                trace_slot[trace_wr_tmp]      = 1'b1;
                trace_pc[trace_wr_tmp]        = dut.commit1_pc;
                trace_has_dest[trace_wr_tmp]  = dut.commit1_has_dest;
                trace_arch_rd[trace_wr_tmp]   = dut.commit1_arch_rd;
                trace_value[trace_wr_tmp]     = dut.commit1_value;
                trace_exception[trace_wr_tmp] = dut.commit1_exception;
                if (trace_wr_tmp == TRACE_DEPTH - 1) trace_wr_tmp = 0;
                else                                 trace_wr_tmp = trace_wr_tmp + 1;
                if (trace_count < TRACE_DEPTH) trace_count = trace_count + 1;
            end
            trace_wr = trace_wr_tmp;

            if (dut.u_mainmem.mem_ready &&   // transaction just completed
                dut.u_mainmem.txn_we)        // it was a write
            begin
                if (dut.u_mainmem.txn_addr[11:4] == TOHOST_RAM_IDX) begin
                    tohost_value   <= dut.u_mainmem.txn_wdata[31:0];
                    tohost_written <= 1'b1;
                end
                if (dut.u_mainmem.txn_addr[11:4] == PUTCHAR_RAM_IDX) begin
                    $write("%c", dut.u_mainmem.txn_wdata[7:0]);
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Helper tasks (reused from Top_tb.v style)
    // -------------------------------------------------------------------------
    task clear_mainmem;
        integer i;
        begin
            for (i = 0; i < 256; i = i + 1)
                dut.u_mainmem.ram[i] = 128'b0;
        end
    endtask

    task clear_caches;
        integer i;
        begin
            for (i = 0; i < CACHE_SETS; i = i + 1) begin
                dut.u_icache.tag_way0[i]          = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_icache.tag_way1[i]          = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_icache.data_way0[i]         = 128'b0;
                dut.u_icache.data_way1[i]         = 128'b0;
                dut.u_icache.lru[i]               = 1'b0;
                dut.u_lsu.u_cache.tag_way0[i]     = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_lsu.u_cache.tag_way1[i]     = {CACHE_TAG_BITS+2{1'b0}};
                dut.u_lsu.u_cache.data_way0[i]    = 128'b0;
                dut.u_lsu.u_cache.data_way1[i]    = 128'b0;
                dut.u_lsu.u_cache.lru[i]          = 1'b0;
            end
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
            tag   = addr[31:32-CACHE_TAG_BITS];
            index = addr[4 + CACHE_INDEX_BITS - 1 : 4];
            raw0  = dut.u_lsu.u_cache.tag_way0[index];
            raw1  = dut.u_lsu.u_cache.tag_way1[index];
            hit0  = raw0[CACHE_TAG_BITS] && (raw0[CACHE_TAG_BITS-1:0] == tag);
            hit1  = raw1[CACHE_TAG_BITS] && (raw1[CACHE_TAG_BITS-1:0] == tag);

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

    task dump_dcache_line;
        input [31:0] addr;
        reg [CACHE_TAG_BITS-1:0] tag;
        reg [CACHE_INDEX_BITS-1:0] index;
        reg [CACHE_TAG_BITS+1:0] raw0;
        reg [CACHE_TAG_BITS+1:0] raw1;
        reg [127:0] line0;
        reg [127:0] line1;
        reg [127:0] line_mem;
        begin
            tag      = addr[31:32-CACHE_TAG_BITS];
            index    = addr[4 + CACHE_INDEX_BITS - 1 : 4];
            raw0     = dut.u_lsu.u_cache.tag_way0[index];
            raw1     = dut.u_lsu.u_cache.tag_way1[index];
            line0    = dut.u_lsu.u_cache.data_way0[index];
            line1    = dut.u_lsu.u_cache.data_way1[index];
            line_mem = dut.u_mainmem.ram[addr[11:4]];
            $display("[DBG_DCACHE] addr=%h idx=%0d tag=%h way0{D,V,tag}={%b,%b,%h} way1{D,V,tag}={%b,%b,%h}",
                     addr, index, tag,
                     raw0[CACHE_TAG_BITS+1], raw0[CACHE_TAG_BITS], raw0[CACHE_TAG_BITS-1:0],
                     raw1[CACHE_TAG_BITS+1], raw1[CACHE_TAG_BITS], raw1[CACHE_TAG_BITS-1:0]);
            $display("[DBG_DCACHE] addr=%h way0_word=%h way1_word=%h mem_word=%h",
                     addr,
                     (addr[3:2] == 2'b00) ? line0[31:0] :
                     (addr[3:2] == 2'b01) ? line0[63:32] :
                     (addr[3:2] == 2'b10) ? line0[95:64] : line0[127:96],
                     (addr[3:2] == 2'b00) ? line1[31:0] :
                     (addr[3:2] == 2'b01) ? line1[63:32] :
                     (addr[3:2] == 2'b10) ? line1[95:64] : line1[127:96],
                     (addr[3:2] == 2'b00) ? line_mem[31:0] :
                     (addr[3:2] == 2'b01) ? line_mem[63:32] :
                     (addr[3:2] == 2'b10) ? line_mem[95:64] : line_mem[127:96]);
        end
    endtask

    task dump_arch_regs;
        begin
            $display("[DBG_REG] ra=%h sp=%h gp=%h tp=%h s0=%h s1=%h a0=%h a1=%h a2=%h a3=%h a4=%h a5=%h",
                     peek_arch_reg(5'd1),  peek_arch_reg(5'd2),
                     peek_arch_reg(5'd3),  peek_arch_reg(5'd4),
                     peek_arch_reg(5'd8),  peek_arch_reg(5'd9),
                     peek_arch_reg(5'd10), peek_arch_reg(5'd11),
                     peek_arch_reg(5'd12), peek_arch_reg(5'd13),
                     peek_arch_reg(5'd14), peek_arch_reg(5'd15));
        end
    endtask

    task dump_recent_commits;
        integer i;
        integer idx;
        begin
            $display("[DBG_TRACE] latest %0d commits (depth=%0d):", trace_count, TRACE_DEPTH);
            for (i = 0; i < TRACE_DEPTH; i = i + 1) begin
                if (i < trace_count) begin
                    idx = trace_wr - 1 - i;
                    if (idx < 0) idx = idx + TRACE_DEPTH;
                    $display("[DBG_TRACE][-%0d] cyc=%0d slot=%0d pc=%h has_dest=%b rd=%0d val=%h exc=%b",
                             i,
                             trace_cycle[idx], trace_slot[idx], trace_pc[idx],
                             trace_has_dest[idx], trace_arch_rd[idx], trace_value[idx], trace_exception[idx]);
                end
            end
        end
    endtask

    task dump_failure_context;
        integer i;
        reg [31:0] sp;
        reg [31:0] stack_base;
        begin
            sp = peek_arch_reg(5'd2);
            stack_base = sp & 32'hFFFF_FFF0;
            $display("[DBG] ---------------- failure context ----------------");
            $display("[DBG_STATE] sim_cyc=%0d wait_cyc=%0d pc=%h rob_empty=%b rob_head=%0d rs_cnt=%0d lsu_busy=%b",
                     sim_cycles, cycle_count, dut.u_if.reg_PC, dut.rob_empty, dut.rob_head,
                     dut.u_issue.rs_count, dut.u_lsu.busy);
            $display("[DBG_STATE] if=%b ib=%b pre=%b rn=%b post=%b rn_stall=%b issue_stall=%b disp_ready=%b flush=%b redirect=%b flush_target=%h",
                     dut.if_inst_valid, dut.ib_valid, dut.pre_valid, dut.rn_valid, dut.post_valid,
                     dut.rn_stall, dut.issue_stall, dut.dispatch_ready,
                     dut.global_flush, dut.redirect_valid, dut.flush_target);
            $display("[DBG_COMMIT_NOW] c0=%b pc0=%h rd0=%0d val0=%h exc0=%b c1=%b pc1=%h rd1=%0d val1=%h exc1=%b",
                     dut.commit0_valid, dut.commit0_pc, dut.commit0_arch_rd, dut.commit0_value, dut.commit0_exception,
                     dut.commit1_valid, dut.commit1_pc, dut.commit1_arch_rd, dut.commit1_value, dut.commit1_exception);
            dump_arch_regs();
            $display("[DBG_MEM] tohost@1FF0=%h putchar@1FE0=%h", peek_mem_word(32'h0000_1FF0), peek_mem_word(32'h0000_1FE0));
            $display("[DBG_STACK] sp=%h aligned_base=%h", sp, stack_base);
            for (i = 0; i < 8; i = i + 1) begin
                $display("[DBG_STACK] %h : %h", stack_base + i*4, peek_mem_word(stack_base + i*4));
            end
            dump_dcache_line(32'h0000_1FF0);
            dump_dcache_line(32'h0000_1FE0);
            dump_dcache_line(sp);
            dump_dcache_line(sp + 32'd4);
            dump_recent_commits();
            $display("[DBG] --------------------------------------------------");
        end
    endtask

    // -------------------------------------------------------------------------
    // load_bin_file — read raw bytes from file into MainMemory
    //   Byte 0 → ram[0][7:0]  (address 0x1000)
    //   Byte 1 → ram[0][15:8] (address 0x1001)
    //   ...little-endian, packed 16 bytes per 128-bit line
    // -------------------------------------------------------------------------
    task load_bin_file;
        input [2047:0] filename;   // 256-char path, padded with zeros
        integer        fd;
        integer        ret;
        reg [7:0]      byte_val;
        reg [127:0]    line_val;
        integer        line_idx;
        integer        byte_pos;
        integer        total_bytes;
        begin
            fd = $fopen(filename, "rb");
            if (fd == 0) begin
                $display("[ERROR] Cannot open BIN file: %s", filename);
                $finish(1);
            end

            line_idx    = 0;
            byte_pos    = 0;
            total_bytes = 0;
            line_val    = 128'b0;

            // Read until $fread returns 0 (EOF).
            // Do NOT use "while (!$feof)" — xsim may lag one call behind,
            // causing an extra $fread that prints a spurious warning.
            ret = $fread(byte_val, fd);
            while (ret > 0) begin
                // Pack byte little-endian into current 128-bit line
                line_val[byte_pos*8 +: 8] = byte_val;
                byte_pos    = byte_pos    + 1;
                total_bytes = total_bytes + 1;

                if (byte_pos == 16) begin
                    dut.u_mainmem.ram[line_idx] = line_val;
                    line_idx = line_idx + 1;
                    line_val = 128'b0;
                    byte_pos = 0;
                end

                ret = $fread(byte_val, fd);
            end

            // Flush final partial line
            if (byte_pos > 0) begin
                dut.u_mainmem.ram[line_idx] = line_val;
                line_idx = line_idx + 1;
            end

            $fclose(fd);
            $display("[INFO] Loaded %0d byte(s) (%0d line(s)) from binary.",
                     total_bytes, line_idx);
        end
    endtask

    initial begin
        // -- Parse plusargs --------------------------------------------------
        if (!$value$plusargs("BIN=%s", bin_file)) begin
            $display("[ERROR] Required plusarg missing.");
            $display("[ERROR] Usage: +BIN=<path/to/program.bin> [+TIMEOUT=<cycles>]");
            $finish(1);
        end
        if (!$value$plusargs("TIMEOUT=%d", timeout_cycles))
            timeout_cycles = 200000;

        $display("[INFO] BIN     = %s", bin_file);
        $display("[INFO] TIMEOUT = %0d cycles", timeout_cycles);

        // -- Initialize state ------------------------------------------------
        rst_n          = 1'b0;
        tohost_written = 1'b0;
        tohost_value   = 32'b0;
        sim_cycles     = 32'd0;
        trace_wr       = 0;
        trace_count    = 0;

        clear_mainmem();
        clear_caches();

        // -- Load binary into MainMemory -------------------------------------
        load_bin_file(bin_file);

        // -- Reset sequence --------------------------------------------------
        @(posedge clk);
        @(posedge clk);
        rst_n = 1'b1;
        $display("[INFO] Reset released, CPU running...");

        // -- Wait for program to write tohost --------------------------------
        cycle_count = 0;
        while (!tohost_written && cycle_count < timeout_cycles) begin
            @(posedge clk);
            cycle_count = cycle_count + 1;
        end

        // -- Report result ---------------------------------------------------
        if (!tohost_written) begin
            $display("[TIMEOUT] %0d cycles elapsed, program did not write tohost.",
                     timeout_cycles);
            dump_failure_context();
            $finish(1);
        end else if (tohost_value == 32'd0) begin
            $display("[PASS] Program exited with code 0 after %0d cycles.", cycle_count);
            $finish(0);
        end else begin
            $display("[FAIL] Program exited with code %0d after %0d cycles.",
                     tohost_value, cycle_count);
            dump_failure_context();
            $finish(1);
        end
    end

endmodule
