`include "riscv_define.v"

// Small fully-associative TLB with round-robin replacement.
// Used by LSU for simple virtual-to-physical translation.
module TLB #(
    parameter TLB_ENTRIES = 8 // Number of entries
)(
    input wire clk,
    input wire rst_n,

    // Lookup request
    input wire [31:0] vaddr,      // Virtual address
    input wire        req,        // Lookup valid

    // Lookup result
    output reg [31:0] paddr,      // Physical address
    output reg        hit,        // Translation hit
    output reg        miss,       // Translation miss

    // Fill/update interface
    input wire        we,         // Write enable
    input wire [31:0] w_vaddr,    // Virtual address to insert
    input wire [31:0] w_paddr     // Physical address to insert
);

    // Portable log2 helper (for environments without $clog2).
    function integer clog2_func;
        input integer value;
        begin
            value = value - 1;
            for (clog2_func = 0; value > 0; clog2_func = clog2_func + 1)
                value = value >> 1;
        end
    endfunction

    localparam PTR_WIDTH = clog2_func(TLB_ENTRIES);

    // Entry arrays
    reg                   valid [0:TLB_ENTRIES-1];
    reg [`VPN_WIDTH-1:0]  vpn   [0:TLB_ENTRIES-1];
    reg [`PPN_WIDTH-1:0]  ppn   [0:TLB_ENTRIES-1];

    // Round-robin replacement pointer.
    reg [PTR_WIDTH-1:0] replace_ptr;

    wire [`VPN_WIDTH-1:0] current_vpn;
    wire [`PAGE_OFFSET_BITS-1:0] offset;

    assign current_vpn = vaddr[31:12];
    assign offset      = vaddr[11:0];

    // Match vector over all entries.
    reg [TLB_ENTRIES-1:0] match_vec;
    integer i;

    always @(*) begin
        for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
            match_vec[i] = valid[i] && (vpn[i] == current_vpn);
        end
    end

    // Combinational lookup result.
    always @(*) begin
        hit = 0;
        paddr = 32'b0;
        miss = 0;
        if (req) begin
            if (match_vec != 0) begin // Use matching entry's PPN + page offset.
                hit = 1;
                for (i = 0; i < TLB_ENTRIES; i = i + 1) begin
                    if (match_vec[i]) paddr = {ppn[i], offset};
                end
            end else begin
                miss = 1;
            end
        end
    end

    // Sequential update path.
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
