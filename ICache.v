`include "riscv_define.v"

module ICache (
    input wire clk,
    input wire rst_n,

    // CPU interface
    input wire [31:0]  paddr,
    input wire         req,
    output wire [127:0] rdata_line,
    output reg         valid_out,
    output wire        stall_cpu,

    // Memory interface
    output reg         mem_req,
    output reg         mem_we,
    output reg [31:0]  mem_addr,
    output reg [127:0] mem_wdata,
    input  wire [127:0] mem_rdata,
    input  wire        mem_ready,
    input  wire [31:0] mem_resp_addr
);
    localparam SETS = 1 << `CACHE_INDEX_BITS;
    localparam TAG_BITS = `CACHE_TAG_BITS;

    wire [TAG_BITS-1:0] tag;
    wire [`CACHE_INDEX_BITS-1:0] index;

    assign tag   = paddr[31 : 32-TAG_BITS];
    assign index = paddr[4 + `CACHE_INDEX_BITS - 1 : 4];

    reg [127:0] data_way0 [0:SETS-1];
    reg [127:0] data_way1 [0:SETS-1];
    reg [TAG_BITS+1:0] tag_way0 [0:SETS-1];
    reg [TAG_BITS+1:0] tag_way1 [0:SETS-1];
    reg lru [0:SETS-1];

    wire [TAG_BITS+1:0] raw_tag0 = tag_way0[index];
    wire [TAG_BITS+1:0] raw_tag1 = tag_way1[index];
    wire valid0 = raw_tag0[TAG_BITS];
    wire valid1 = raw_tag1[TAG_BITS];
    wire [TAG_BITS-1:0] saved_tag0 = raw_tag0[TAG_BITS-1:0];
    wire [TAG_BITS-1:0] saved_tag1 = raw_tag1[TAG_BITS-1:0];

    wire hit0 = valid0 && (saved_tag0 == tag);
    wire hit1 = valid1 && (saved_tag1 == tag);
    wire hit  = (hit0 || hit1) && req;

    assign rdata_line = hit1 ? data_way1[index] : data_way0[index];

    localparam IDLE = 1'b0;
    localparam REFILL = 1'b1;
    reg state;

    wire victim_way = lru[index];
    assign stall_cpu = req && !hit;

    // Latched miss metadata: refill must never use live paddr/index/tag.
    reg [`CACHE_INDEX_BITS-1:0] miss_index;
    reg [TAG_BITS-1:0] miss_tag;
    reg miss_victim_way;
    reg [31:0] miss_line_addr;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            valid_out <= 1'b0;
            mem_req <= 1'b0;
            mem_we <= 1'b0;
            mem_addr <= 32'b0;
            mem_wdata <= 128'b0;
            miss_index <= {`CACHE_INDEX_BITS{1'b0}};
            miss_tag <= {TAG_BITS{1'b0}};
            miss_victim_way <= 1'b0;
            miss_line_addr <= 32'b0;
            for (i = 0; i < SETS; i = i + 1) begin
                data_way0[i] <= 128'b0;
                data_way1[i] <= 128'b0;
                tag_way0[i] <= {(TAG_BITS+2){1'b0}};
                tag_way1[i] <= {(TAG_BITS+2){1'b0}};
                lru[i] <= 1'b0;
            end
        end else begin
            valid_out <= 1'b0;
            mem_req <= 1'b0;
            mem_we <= 1'b0;
            mem_addr <= 32'b0;
            mem_wdata <= 128'b0;

            case (state)
                IDLE: begin
                    if (req) begin
                        if (hit) begin
                            valid_out <= 1'b1;
                            lru[index] <= hit0 ? 1'b1 : 1'b0;
                        end else begin
                            state <= REFILL;
                            miss_index <= index;
                            miss_tag <= tag;
                            miss_victim_way <= victim_way;
                            miss_line_addr <= {paddr[31:4], 4'b0000};
                            mem_req <= 1'b1;
                            mem_we <= 1'b0;
                            mem_addr <= {paddr[31:4], 4'b0000};
                        end
                    end
                end
                REFILL: begin
                    mem_req <= 1'b1;
                    mem_we <= 1'b0;
                    mem_addr <= miss_line_addr;
                    if (mem_ready) begin
                        // Only install the line if the returned response address matches
                        // the outstanding miss. This prevents silent line skew when memory
                        // responses get mis-associated.
                        if (mem_resp_addr == miss_line_addr) begin
                            state <= IDLE;
                            if (!miss_victim_way) begin
                                data_way0[miss_index] <= mem_rdata;
                                tag_way0[miss_index] <= {1'b0, 1'b1, miss_tag};
                            end else begin
                                data_way1[miss_index] <= mem_rdata;
                                tag_way1[miss_index] <= {1'b0, 1'b1, miss_tag};
                            end
                            lru[miss_index] <= ~miss_victim_way;
                        end else begin
                            // Keep waiting/retry; arbiter/mainmem should eventually return
                            // the correct line for miss_line_addr.
                            state <= REFILL;
                        end
                    end
                end
            endcase
        end
    end
endmodule
