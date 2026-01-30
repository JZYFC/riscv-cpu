`include "riscv_define.v"

module MemArbiter (
    input wire clk,
    input wire rst_n,

    // I-Cache 接口 
    input wire         i_req,
    input wire         i_we, 
    input wire [31:0]  i_addr,
    input wire [127:0] i_wdata,
    output reg [127:0] i_rdata,
    output reg         i_ready,

    // D-Cache (LSU) 接口
    input wire         d_req,
    input wire         d_we,
    input wire [31:0]  d_addr,
    input wire [127:0] d_wdata,
    output reg [127:0] d_rdata,
    output reg         d_ready,

    // 主存接口
    output reg         mem_req,
    output reg         mem_we,
    output reg [31:0]  mem_addr,
    output reg [127:0] mem_wdata,
    input  wire [127:0] mem_rdata,
    input  wire        mem_ready
);

    // 状态机：空闲、服务D-Cache、服务I-Cache
    localparam IDLE = 0, SERVE_D = 1, SERVE_I = 2;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    if (d_req) state <= SERVE_D;      // D-Cache 优先
                    else if (i_req) state <= SERVE_I;
                end
                SERVE_D: begin
                    if (mem_ready) state <= IDLE; // 传输完成
                end
                SERVE_I: begin
                    if (mem_ready) state <= IDLE; // 传输完成
                end
            endcase
        end
    end

    // 组合逻辑路由信号
    always @(*) begin
        // 默认值
        mem_req = 0; mem_we = 0; mem_addr = 0; mem_wdata = 0;
        d_ready = 0; d_rdata = 0;
        i_ready = 0; i_rdata = 0;

        case (state)
            IDLE: begin
                if (d_req) begin
                    mem_req = 1;
                    mem_we = d_we;
                    mem_addr = d_addr;
                    mem_wdata = d_wdata;
                end else if (i_req) begin
                    mem_req = 1;
                    mem_we = i_we;
                    mem_addr = i_addr;
                    mem_wdata = i_wdata;
                end
            end
            SERVE_D: begin
                mem_req = 1;
                mem_we = d_we;
                mem_addr = d_addr;
                mem_wdata = d_wdata;
                d_ready = mem_ready;
                d_rdata = mem_rdata;
            end
            SERVE_I: begin
                mem_req = 1;
                mem_we = i_we;
                mem_addr = i_addr;
                mem_wdata = i_wdata;
                i_ready = mem_ready;
                i_rdata = mem_rdata;
            end
        endcase
    end

endmodule