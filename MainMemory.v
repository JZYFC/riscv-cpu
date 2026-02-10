`include "riscv_define.v"

module MainMemory (
    input wire clk,
    input wire rst_n,

    input wire         mem_req,
    input wire         mem_we,
    input wire [31:0]  mem_addr,
    input wire [127:0] mem_wdata,

    output reg [127:0] mem_rdata,
    output reg         mem_ready,
    // Address of the completed transaction (line-aligned by requesters).
    // Used by caches to verify refill responses match the outstanding miss.
    output reg [31:0]  mem_resp_addr
);

    reg [127:0] ram [0:255];
    integer delay_cnt;
    localparam LATENCY = 4;
    reg txn_active;
    reg txn_we;
    reg [31:0] txn_addr;
    reg [127:0] txn_wdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_ready <= 0;
            delay_cnt <= 0;
            mem_rdata <= 0;
            mem_resp_addr <= 32'b0;
            txn_active <= 1'b0;
            txn_we <= 1'b0;
            txn_addr <= 32'b0;
            txn_wdata <= 128'b0;
        end else begin
            mem_ready <= 0;

            if (txn_active) begin
                if (delay_cnt < LATENCY) begin
                    delay_cnt <= delay_cnt + 1;
                end else begin
                    mem_ready <= 1;
                    delay_cnt <= 0;
                    txn_active <= 1'b0;
                    mem_resp_addr <= txn_addr;

                    if (txn_we) begin
                        ram[txn_addr[11:4]] <= txn_wdata;
                        if (txn_addr[11:4] == 8'hEB) begin
                            $display("DBG_MEM_WRITE addr=%h line=%h", txn_addr, txn_wdata);
                        end
                    end else begin
                        mem_rdata <= ram[txn_addr[11:4]];
                    end
                end
            end else begin
                delay_cnt <= 0;
                if (mem_req) begin
                    txn_active <= 1'b1;
                    txn_we <= mem_we;
                    txn_addr <= mem_addr;
                    txn_wdata <= mem_wdata;
                    delay_cnt <= 1;
                end
            end
        end
    end

    initial begin
        ram[8'h00] = 128'hDEAD_BEEF_CAFE_BABE_0123_4567_89AB_CDEF;
        ram[8'h01] = 128'h1111_2222_3333_4444_5555_6666_7777_8888;
    end

endmodule
