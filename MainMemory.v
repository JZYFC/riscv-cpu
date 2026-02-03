`include "riscv_define.v"

module MainMemory (
    input wire clk,
    input wire rst_n,
    
    input wire         mem_req,
    input wire         mem_we,
    input wire [31:0]  mem_addr,
    input wire [127:0] mem_wdata,
    
    output reg [127:0] mem_rdata,
    output reg         mem_ready
);

    reg [127:0] ram [0:255]; 
    integer delay_cnt;
    localparam LATENCY = 4;

    // 提取索引的辅助信号
    wire [7:0] idx;
    assign idx = mem_addr[11:4];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_ready <= 0;
            delay_cnt <= 0;
            mem_rdata <= 0;
        end else begin
            mem_ready <= 0;

            if (mem_req) begin
                if (delay_cnt < LATENCY) begin
                    delay_cnt <= delay_cnt + 1;
                end else begin
                    mem_ready <= 1;
                    delay_cnt <= 0;

                    if (mem_we) begin
                        ram[idx] <= mem_wdata; 
                        if (mem_addr[11:4] == 8'hEB) begin
                            $display("DBG_MEM_WRITE addr=%h line=%h", mem_addr, mem_wdata);
                        end
                    end else begin
                        mem_rdata <= ram[idx];
                    end
                end
            end else begin
                delay_cnt <= 0;
            end
        end
    end
    
    initial begin
        ram[8'h00] = 128'hDEAD_BEEF_CAFE_BABE_0123_4567_89AB_CDEF; 
        ram[8'h01] = 128'h1111_2222_3333_4444_5555_6666_7777_8888;
    end

endmodule