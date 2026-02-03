`include "riscv_define.v"

module Cache (
    input wire clk,
    input wire rst_n,

    // CPU Interface
    input wire [31:0]  paddr,
    input wire         req,
    input wire         we,
    input wire [31:0]  wdata,
    input wire [3:0]   wstrb,
    output reg [31:0]  rdata,
    output reg         valid_out,
    output wire        stall_cpu,

    // Main mem Interface
    output reg         mem_req,
    output reg         mem_we,
    output reg [31:0]  mem_addr,
    output reg [127:0] mem_wdata,
    input  wire [127:0] mem_rdata,
    input  wire        mem_ready
);

    // Parameters
    localparam SETS = 1 << `CACHE_INDEX_BITS;
    localparam TAG_BITS = `CACHE_TAG_BITS;
    
    // Address Breakdown
    wire [TAG_BITS-1:0]      tag;
    wire [`CACHE_INDEX_BITS-1:0] index;
    wire [3:0]               offset;

    // High bits [31 : 10]
    assign tag   = paddr[31 : 32-TAG_BITS]; 
    
    // Mid bits [9 : 4] 
    assign index = paddr[4 + `CACHE_INDEX_BITS - 1 : 4]; 
    
    // Low bits [3 : 0]
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

    // Hit Logic
    wire hit0;
    wire hit1;
    wire hit;
    
    assign hit0 = valid0 && (saved_tag0 == tag);
    assign hit1 = valid1 && (saved_tag1 == tag);
    assign hit  = (hit0 || hit1) && req;

    // Select by data
    reg [127:0] selected_line;
    always @(*) begin
        if (hit1) selected_line = data_way1[index];
        else      selected_line = data_way0[index];
    end
    
    // Read 32 bit word
    always @(*) begin
        case(offset[3:2])
            2'b00: rdata = selected_line[31:0];
            2'b01: rdata = selected_line[63:32];
            2'b10: rdata = selected_line[95:64];
            2'b11: rdata = selected_line[127:96];
        endcase
    end

    localparam IDLE = 0, REFILL = 1, WRITEBACK = 2, WRITE_THROUGH = 3;
    reg [1:0] state;
    integer i;
    
    // Replacement Policy
    wire victim_way;
    wire victim_dirty;
    wire [TAG_BITS-1:0] victim_tag;
    
    assign victim_way = lru[index]; 
    assign victim_dirty = victim_way ? dirty1 : dirty0;
    assign victim_tag = victim_way ? saved_tag1 : saved_tag0;

    assign stall_cpu = (req && !hit);
    reg [127:0] temp_data; 
    reg [TAG_BITS+1:0] temp_tag;
    reg [127:0] wt_data;
    reg [31:0]  wt_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            valid_out <= 0;
            mem_req <= 0;
            mem_we <= 0;
            mem_addr <= 0;
            mem_wdata <= 0;
            rdata <= 0;
            wt_data <= 0;
            wt_addr <= 0;
            for (i = 0; i < SETS; i = i + 1) begin
                data_way0[i] <= 0;
                data_way1[i] <= 0;
                tag_way0[i] <= 0;
                tag_way1[i] <= 0;
                lru[i] <= 0;
            end
        end else begin
            valid_out <= 0;
            mem_req <= 0;
            mem_we <= 0;

            case (state)
                IDLE: begin
                    if (req) begin
                        if (hit) begin
                            // Update LRU
                            lru[index] <= hit0 ? 1'b1 : 1'b0;

                            if (we) begin
                                if (hit0) begin
                                    // Write-through: keep line clean
                                    temp_tag = tag_way0[index];
                                    temp_tag[TAG_BITS+1] = 1'b0;
                                    tag_way0[index] <= temp_tag;

                                    // Handle Write Data: Read-Modify-Write
                                    temp_data = data_way0[index];
                                    case(offset[3:2])
                                        2'b00: begin
                                            if (wstrb[0]) temp_data[7:0]   = wdata[7:0];
                                            if (wstrb[1]) temp_data[15:8]  = wdata[15:8];
                                            if (wstrb[2]) temp_data[23:16] = wdata[23:16];
                                            if (wstrb[3]) temp_data[31:24] = wdata[31:24];
                                        end
                                        2'b01: begin
                                            if (wstrb[0]) temp_data[39:32] = wdata[7:0];
                                            if (wstrb[1]) temp_data[47:40] = wdata[15:8];
                                            if (wstrb[2]) temp_data[55:48] = wdata[23:16];
                                            if (wstrb[3]) temp_data[63:56] = wdata[31:24];
                                        end
                                        2'b10: begin
                                            if (wstrb[0]) temp_data[71:64] = wdata[7:0];
                                            if (wstrb[1]) temp_data[79:72] = wdata[15:8];
                                            if (wstrb[2]) temp_data[87:80] = wdata[23:16];
                                            if (wstrb[3]) temp_data[95:88] = wdata[31:24];
                                        end
                                        2'b11: begin
                                            if (wstrb[0]) temp_data[103:96]  = wdata[7:0];
                                            if (wstrb[1]) temp_data[111:104] = wdata[15:8];
                                            if (wstrb[2]) temp_data[119:112] = wdata[23:16];
                                            if (wstrb[3]) temp_data[127:120] = wdata[31:24];
                                        end
                                    endcase
                                    data_way0[index] <= temp_data;
                                end else begin
                                    // Hit Way 1
                                    temp_tag = tag_way1[index];
                                    temp_tag[TAG_BITS+1] = 1'b0;
                                    tag_way1[index] <= temp_tag;

                                    temp_data = data_way1[index];
                                    case(offset[3:2])
                                        2'b00: begin
                                            if (wstrb[0]) temp_data[7:0]   = wdata[7:0];
                                            if (wstrb[1]) temp_data[15:8]  = wdata[15:8];
                                            if (wstrb[2]) temp_data[23:16] = wdata[23:16];
                                            if (wstrb[3]) temp_data[31:24] = wdata[31:24];
                                        end
                                        2'b01: begin
                                            if (wstrb[0]) temp_data[39:32] = wdata[7:0];
                                            if (wstrb[1]) temp_data[47:40] = wdata[15:8];
                                            if (wstrb[2]) temp_data[55:48] = wdata[23:16];
                                            if (wstrb[3]) temp_data[63:56] = wdata[31:24];
                                        end
                                        2'b10: begin
                                            if (wstrb[0]) temp_data[71:64] = wdata[7:0];
                                            if (wstrb[1]) temp_data[79:72] = wdata[15:8];
                                            if (wstrb[2]) temp_data[87:80] = wdata[23:16];
                                            if (wstrb[3]) temp_data[95:88] = wdata[31:24];
                                        end
                                        2'b11: begin
                                            if (wstrb[0]) temp_data[103:96]  = wdata[7:0];
                                            if (wstrb[1]) temp_data[111:104] = wdata[15:8];
                                            if (wstrb[2]) temp_data[119:112] = wdata[23:16];
                                            if (wstrb[3]) temp_data[127:120] = wdata[31:24];
                                        end
                                    endcase
                                    data_way1[index] <= temp_data;
                                end
                                if (paddr[11:4] == 8'hEB) begin
                                    $display("DBG_DC_STORE_HIT addr=%h offset=%h wdata=%h wstrb=%b line=%h",
                                             paddr, offset, wdata, wstrb, temp_data);
                                end
                                wt_data <= temp_data;
                                wt_addr <= {paddr[31:4], 4'b0000};
                                mem_addr <= {paddr[31:4], 4'b0000};
                                mem_wdata <= temp_data;
                                mem_req <= 1;
                                mem_we <= 1;
                                state <= WRITE_THROUGH;
                            end else begin
                                valid_out <= 1;
                            end
                        end else begin
                            // Miss
                            if (victim_dirty) begin
                                state <= WRITEBACK;
                                // Fix: address concatenation
                                mem_addr <= {victim_tag, index, 4'b0000};
                                mem_wdata <= victim_way ? data_way1[index] : data_way0[index];
                                mem_req <= 1;
                                mem_we <= 1;
                            end else begin
                                state <= REFILL;
                                // Fix: address concatenation
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
                        // Fix Bug 2: Merge pending store data if this is a write miss
                        temp_data = mem_rdata;
                        if (we) begin
                            case(offset[3:2])
                                2'b00: begin
                                    if (wstrb[0]) temp_data[7:0]   = wdata[7:0];
                                    if (wstrb[1]) temp_data[15:8]  = wdata[15:8];
                                    if (wstrb[2]) temp_data[23:16] = wdata[23:16];
                                    if (wstrb[3]) temp_data[31:24] = wdata[31:24];
                                end
                                2'b01: begin
                                    if (wstrb[0]) temp_data[39:32] = wdata[7:0];
                                    if (wstrb[1]) temp_data[47:40] = wdata[15:8];
                                    if (wstrb[2]) temp_data[55:48] = wdata[23:16];
                                    if (wstrb[3]) temp_data[63:56] = wdata[31:24];
                                end
                                2'b10: begin
                                    if (wstrb[0]) temp_data[71:64] = wdata[7:0];
                                    if (wstrb[1]) temp_data[79:72] = wdata[15:8];
                                    if (wstrb[2]) temp_data[87:80] = wdata[23:16];
                                    if (wstrb[3]) temp_data[95:88] = wdata[31:24];
                                end
                                2'b11: begin
                                    if (wstrb[0]) temp_data[103:96]  = wdata[7:0];
                                    if (wstrb[1]) temp_data[111:104] = wdata[15:8];
                                    if (wstrb[2]) temp_data[119:112] = wdata[23:16];
                                    if (wstrb[3]) temp_data[127:120] = wdata[31:24];
                                end
                            endcase
                        end

                        if (victim_way == 0) begin
                            data_way0[index] <= temp_data;
                            // Write-through: keep line clean
                            tag_way0[index]  <= {1'b0, 1'b1, tag};
                        end else begin
                            data_way1[index] <= temp_data;
                            tag_way1[index]  <= {1'b0, 1'b1, tag};
                        end
                        // Update LRU: filled way is MRU, other is LRU
                        lru[index] <= ~victim_way;
                        if (we && (paddr[11:4] == 8'hEB)) begin
                            $display("DBG_DC_REFILL_STORE addr=%h offset=%h wdata=%h wstrb=%b line=%h",
                                     paddr, offset, wdata, wstrb, temp_data);
                        end
                        if (we) begin
                            wt_data <= temp_data;
                            wt_addr <= {paddr[31:4], 4'b0000};
                            mem_addr <= {paddr[31:4], 4'b0000};
                            mem_wdata <= temp_data;
                            mem_req <= 1;
                            mem_we <= 1;
                            state <= WRITE_THROUGH;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        mem_req <= 1;
                        mem_we <= 0;
                    end
                end
                WRITE_THROUGH: begin
                    if (mem_ready) begin
                        state <= IDLE;
                        valid_out <= 1;
                        if (wt_addr[11:4] == 8'hEB) begin
                            $display("DBG_DC_WRITE_THROUGH addr=%h line=%h", wt_addr, wt_data);
                        end
                    end else begin
                        mem_req <= 1;
                        mem_we <= 1;
                        mem_addr <= wt_addr;
                        mem_wdata <= wt_data;
                    end
                end
            endcase
        end
    end

endmodule
