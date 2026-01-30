`include "riscv_define.v"

module ICache (
    input wire clk,
    input wire rst_n,

    // CPU 侧接口
    input wire [31:0]  paddr,
    input wire         req,
    output wire [127:0] rdata_line, // 输出整行数据给 IF
    output reg         valid_out,
    output wire        stall_cpu,

    //主存接口
    output reg         mem_req,
    output reg         mem_we,
    output reg [31:0]  mem_addr,
    output reg [127:0] mem_wdata,
    input  wire [127:0] mem_rdata,
    input  wire        mem_ready
);
    localparam SETS = 1 << `CACHE_INDEX_BITS;
    localparam TAG_BITS = `CACHE_TAG_BITS;

    wire [TAG_BITS-1:0]      tag;
    wire [`CACHE_INDEX_BITS-1:0] index;
    
    assign tag   = paddr[31 : 32-TAG_BITS];
    assign index = paddr[4 + `CACHE_INDEX_BITS - 1 : 4];

    //存储阵列
    reg [127:0] data_way0 [0:SETS-1];
    reg [127:0] data_way1 [0:SETS-1];
    reg [TAG_BITS+1:0] tag_way0 [0:SETS-1];
    reg [TAG_BITS+1:0] tag_way1 [0:SETS-1];
    reg lru [0:SETS-1];

    // 读取 Tag
    wire [TAG_BITS+1:0] raw_tag0 = tag_way0[index];
    wire [TAG_BITS+1:0] raw_tag1 = tag_way1[index];
    wire valid0 = raw_tag0[TAG_BITS];
    wire valid1 = raw_tag1[TAG_BITS];
    wire [TAG_BITS-1:0] saved_tag0 = raw_tag0[TAG_BITS-1:0];
    wire [TAG_BITS-1:0] saved_tag1 = raw_tag1[TAG_BITS-1:0];

    // Hit 逻辑
    wire hit0 = valid0 && (saved_tag0 == tag);
    wire hit1 = valid1 && (saved_tag1 == tag);
    wire hit  = (hit0 || hit1) && req;

    // 直接输出被选中的 Cache Line
    assign rdata_line = hit1 ? data_way1[index] : data_way0[index];
    
    localparam IDLE = 0, REFILL = 1; 
    reg state;
    
    // 替换策略 (Pseudo-LRU)
    wire victim_way = lru[index];
    assign stall_cpu = (req && !hit);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            valid_out <= 0;
            mem_req <= 0;
        end else begin
            valid_out <= 0;
            mem_req <= 0;
            mem_we <= 0; 

            case (state)
                IDLE: begin
                    if (req) begin
                        if (hit) begin
                            valid_out <= 1;
                            lru[index] <= hit0 ? 1'b1 : 1'b0;
                        end else begin
                            state <= REFILL;
                            mem_addr <= {paddr[31:4], 4'b0000};
                            mem_req <= 1;
                        end
                    end
                end
                REFILL: begin
                    if (mem_ready) begin
                        state <= IDLE;
                        if (victim_way == 0) begin
                            data_way0[index] <= mem_rdata;
                            tag_way0[index]  <= {1'b0, 1'b1, tag};
                        end else begin
                            data_way1[index] <= mem_rdata;
                            tag_way1[index]  <= {1'b0, 1'b1, tag};
                        end
                    end else begin
                        mem_req <= 1;
                    end
                end
            endcase
        end
    end
endmodule