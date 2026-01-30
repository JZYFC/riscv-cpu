`include "riscv_define.v"

module Cache (
    input wire clk,
    input wire rst_n,

    // CPU 侧接口
    input wire [31:0]  paddr,
    input wire         req,
    input wire         we,
    input wire [31:0]  wdata,
    input wire [3:0]   wstrb,
    output reg [31:0]  rdata,
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

    //参数
    localparam SETS = 1 << `CACHE_INDEX_BITS;
    localparam TAG_BITS = `CACHE_TAG_BITS;
    
    //地址分解 
    wire [TAG_BITS-1:0]      tag;
    wire [`CACHE_INDEX_BITS-1:0] index;
    wire [3:0]               offset;

    // 高位 [31 : 10]
    assign tag   = paddr[31 : 32-TAG_BITS]; 
    
    // 中间位 [9 : 4] 
    assign index = paddr[4 + `CACHE_INDEX_BITS - 1 : 4]; 
    
    // 低位 [3 : 0]
    assign offset= paddr[3:0];

    reg [127:0] data_way0 [0:SETS-1];
    reg [127:0] data_way1 [0:SETS-1];
    reg [TAG_BITS+1:0] tag_way0 [0:SETS-1]; 
    reg [TAG_BITS+1:0] tag_way1 [0:SETS-1];   
    reg lru [0:SETS-1];
    wire [TAG_BITS+1:0] raw_tag0;
    wire [TAG_BITS+1:0] raw_tag1;
    
    assign raw_tag0 = tag_way0[index];
    assign raw_tag1 = tag_way1[index];

    wire valid0;
    wire valid1;
    wire dirty0;
    wire dirty1;
    wire [TAG_BITS-1:0] saved_tag0;
    wire [TAG_BITS-1:0] saved_tag1;

    assign valid0 = raw_tag0[TAG_BITS];
    assign valid1 = raw_tag1[TAG_BITS];
    assign dirty0 = raw_tag0[TAG_BITS+1];
    assign dirty1 = raw_tag1[TAG_BITS+1];
    assign saved_tag0 = raw_tag0[TAG_BITS-1:0];
    assign saved_tag1 = raw_tag1[TAG_BITS-1:0];

    // Hit 逻辑
    wire hit0;
    wire hit1;
    wire hit;
    
    assign hit0 = valid0 && (saved_tag0 == tag);
    assign hit1 = valid1 && (saved_tag1 == tag);
    assign hit  = (hit0 || hit1) && req;

    // 读数据选择
    reg [127:0] selected_line;
    always @(*) begin
        if (hit1) selected_line = data_way1[index];
        else      selected_line = data_way0[index];
    end
    
    // 提取32位字
    always @(*) begin
        case(offset[3:2])
            2'b00: rdata = selected_line[31:0];
            2'b01: rdata = selected_line[63:32];
            2'b10: rdata = selected_line[95:64];
            2'b11: rdata = selected_line[127:96];
        endcase
    end

    localparam IDLE = 0, REFILL = 1, WRITEBACK = 2;
    reg [1:0] state;
    
    // 替换策略
    wire victim_way;
    wire victim_dirty;
    wire [TAG_BITS-1:0] victim_tag;
    
    assign victim_way = lru[index]; 
    assign victim_dirty = victim_way ? dirty1 : dirty0;
    assign victim_tag = victim_way ? saved_tag1 : saved_tag0;

    assign stall_cpu = (req && !hit);
    reg [127:0] temp_data; 
    reg [TAG_BITS+1:0] temp_tag;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            valid_out <= 0;
        end else begin
            valid_out <= 0;
            mem_req <= 0;
            mem_we <= 0;

            case (state)
                IDLE: begin
                    if (req) begin
                        if (hit) begin
                            valid_out <= 1;
                            // 更新 LRU
                            lru[index] <= hit0 ? 1'b1 : 1'b0;

                            // 处理写命中 (Write Hit)
                            if (we) begin
                                if (hit0) begin
                                    // 更新 Dirty 位
                                    temp_tag = tag_way0[index];
                                    temp_tag[TAG_BITS+1] = 1'b1;
                                    tag_way0[index] <= temp_tag;

                                    // 数据写入：读-改-写
                                    temp_data = data_way0[index];
                                    case(offset[3:2])
                                        2'b00: temp_data[31:0] = wdata;
                                        2'b01: temp_data[63:32] = wdata;
                                        2'b10: temp_data[95:64] = wdata;
                                        2'b11: temp_data[127:96] = wdata;
                                    endcase
                                    data_way0[index] <= temp_data;
                                end else begin
                                    // Hit Way 1
                                    temp_tag = tag_way1[index];
                                    temp_tag[TAG_BITS+1] = 1'b1;
                                    tag_way1[index] <= temp_tag;

                                    temp_data = data_way1[index];
                                    case(offset[3:2])
                                        2'b00: temp_data[31:0] = wdata;
                                        2'b01: temp_data[63:32] = wdata;
                                        2'b10: temp_data[95:64] = wdata;
                                        2'b11: temp_data[127:96] = wdata;
                                    endcase
                                    data_way1[index] <= temp_data;
                                end
                            end
                        end else begin
                            // Miss
                            if (victim_dirty) begin
                                state <= WRITEBACK;
                                // 修正: 地址拼接
                                mem_addr <= {victim_tag, index, 4'b0000};
                                mem_wdata <= victim_way ? data_way1[index] : data_way0[index];
                                mem_req <= 1;
                                mem_we <= 1;
                            end else begin
                                state <= REFILL;
                                // 修正: 地址拼接
                                mem_addr <= {paddr[31:4], 4'b0000};
                                mem_req <= 1;
                                mem_we <= 0;
                            end
                        end
                    end
                end

                WRITEBACK: begin
                    if (mem_ready) begin
                        state <= REFILL;
                        mem_addr <= {paddr[31:4], 4'b0000};
                        mem_req <= 1;
                        mem_we <= 0;
                    end else begin
                        mem_req <= 1;
                        mem_we <= 1;
                    end
                end

                REFILL: begin
                    if (mem_ready) begin
                        state <= IDLE;
                        if (victim_way == 0) begin
                            data_way0[index] <= mem_rdata;
                            // Set: Dirty=0, Valid=1, Tag=current tag
                            tag_way0[index]  <= {1'b0, 1'b1, tag};
                        end else begin
                            data_way1[index] <= mem_rdata;
                            tag_way1[index]  <= {1'b0, 1'b1, tag};
                        end
                    end else begin
                        mem_req <= 1;
                        mem_we <= 0;
                    end
                end
            endcase
        end
    end

endmodule