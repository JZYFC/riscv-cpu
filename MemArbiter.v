`include "riscv_define.v"

module MemArbiter (
    input wire clk,
    input wire rst_n,

    // I-Cache interface
    input wire         i_req,
    input wire         i_we,
    input wire [31:0]  i_addr,
    input wire [127:0] i_wdata,
    output reg [127:0] i_rdata,
    output reg         i_ready,

    // D-Cache (LSU) interface
    input wire         d_req,
    input wire         d_we,
    input wire [31:0]  d_addr,
    input wire [127:0] d_wdata,
    output reg [127:0] d_rdata,
    output reg         d_ready,

    // Main memory interface
    output reg         mem_req,
    output reg         mem_we,
    output reg [31:0]  mem_addr,
    output reg [127:0] mem_wdata,
    input  wire [127:0] mem_rdata,
    input  wire        mem_ready,
    input  wire [31:0] mem_resp_addr,
    output reg [31:0]  i_resp_addr,
    output reg [31:0]  d_resp_addr
);

    // State machine: prioritize D-Cache over I-Cache
    localparam IDLE = 0, SERVE_D = 1, SERVE_I = 2;
    reg [1:0] state;
    reg       serving_we;
    reg [31:0]  serving_addr;
    reg [127:0] serving_wdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            serving_we <= 1'b0;
            serving_addr <= 32'b0;
            serving_wdata <= 128'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (d_req) begin
                        serving_we <= d_we;
                        serving_addr <= d_addr;
                        serving_wdata <= d_wdata;
                        state <= SERVE_D;
                    end else if (i_req) begin
                        serving_we <= i_we;
                        serving_addr <= i_addr;
                        serving_wdata <= i_wdata;
                        state <= SERVE_I;
                    end
                end
                SERVE_D: begin
                    if (mem_ready) state <= IDLE;
                end
                SERVE_I: begin
                    if (mem_ready) state <= IDLE;
                end
            endcase
        end
    end

    // Combinational routing
    always @(*) begin
        mem_req = 0; mem_we = 0; mem_addr = 0; mem_wdata = 0;
        d_ready = 0; d_rdata = 0;
        i_ready = 0; i_rdata = 0;
        i_resp_addr = 32'b0;
        d_resp_addr = 32'b0;

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
            // Once a request is launched (accepted by memory), do NOT keep mem_req asserted.
            // MainMemory latches the request on acceptance; holding mem_req high risks
            // a duplicate accept in the cycle after completion (txn_active drops before
            // the arbiter state updates), which can desynchronize I/D responses.
            SERVE_D: begin
                mem_req = 0;
                mem_we = 0;
                mem_addr = 0;
                mem_wdata = 0;
                d_ready = mem_ready;
                d_rdata = mem_rdata;
                d_resp_addr = mem_resp_addr;
            end
            SERVE_I: begin
                mem_req = 0;
                mem_we = 0;
                mem_addr = 0;
                mem_wdata = 0;
                i_ready = mem_ready;
                i_rdata = mem_rdata;
                i_resp_addr = mem_resp_addr;
            end
        endcase
    end

endmodule
