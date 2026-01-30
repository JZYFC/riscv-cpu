`include "riscv_define.v"

module TLB #(
    parameter TLB_ENTRIES = 8 // 表项数
)(
    input wire clk,
    input wire rst_n,

    // 查找请求
    input wire [31:0] vaddr,      // 虚拟地址
    input wire        req,        // 查找请求信号
    
    // 查找结果
    output reg [31:0] paddr,      // 物理地址
    output reg        hit,        // 命中信号
    output reg        miss,       // 未命中信号

    // 维护接口
    input wire        we,         // 写使能
    input wire [31:0] w_vaddr,    // 要写入的虚拟地址
    input wire [31:0] w_paddr     // 要写入的物理地址
);

    // 计算 Log2 的函数 (替代系统函数 $clog2)
    function integer clog2_func;
        input integer value;
        begin
            value = value - 1;
            for (clog2_func = 0; value > 0; clog2_func = clog2_func + 1)
                value = value >> 1;
        end
    endfunction

    localparam PTR_WIDTH = clog2_func(TLB_ENTRIES);

    // TLB 表项结构
    reg                   valid [0:TLB_ENTRIES-1];
    reg [`VPN_WIDTH-1:0]  vpn   [0:TLB_ENTRIES-1];
    reg [`PPN_WIDTH-1:0]  ppn   [0:TLB_ENTRIES-1];
    
    // 替换指针
    reg [PTR_WIDTH-1:0] replace_ptr; 

    // 提取虚拟页号
    wire [`VPN_WIDTH-1:0] current_vpn;
    wire [`PAGE_OFFSET_BITS-1:0] offset;
    
    assign current_vpn = vaddr[31:12];
    assign offset      = vaddr[11:0];

    // 并行查找
    reg [TLB_ENTRIES-1:0] match_vec;
    integer i;
    
    always @(*) begin
        for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
            match_vec[i] = valid[i] && (vpn[i] == current_vpn); 
        end
    end

    // 生成结果
    always @(*) begin
        hit = 0;
        paddr = 32'b0;
        miss = 0;
        if (req) begin
            if (match_vec != 0) begin // 如果有一位匹配
                hit = 1;
                for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
                    if (match_vec[i]) paddr = {ppn[i], offset};
                end
            end else begin
                miss = 1;
            end
        end
    end

    // 更新逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            replace_ptr <= 0;
            for (i = 0; i < TLB_ENTRIES; i = i + 1) valid[i] <= 0;
        end else if (we) begin
            valid[replace_ptr] <= 1'b1;
            vpn[replace_ptr]   <= w_vaddr[31:12];
            ppn[replace_ptr]   <= w_paddr[31:12];
            replace_ptr        <= replace_ptr + 1; 
        end
    end

endmodule