`include "riscv_define.v"

// Simple Gshare-style branch predictor with BTB and small RAS.
module BranchPredictor #(
    parameter GHR_BITS   = `BP_GHR_BITS,
    parameter IDX_BITS   = `BP_IDX_BITS,
    parameter RAS_DEPTH  = `BP_RAS_DEPTH
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     flush,

    // Fetch query
    input  wire                     fetch_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] fetch_pc,
    output wire                     pred_taken,
    output wire [`INST_ADDR_WIDTH-1:0] pred_target,
    output wire [GHR_BITS-1:0]      pred_hist,

    // Two resolution/update ports (applied in order 0 then 1)
    input  wire                     update0_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] update0_pc,
    input  wire                     update0_taken,
    input  wire [`INST_ADDR_WIDTH-1:0] update0_target,
    input  wire [GHR_BITS-1:0]      update0_hist,
    input  wire                     update0_is_call,
    input  wire                     update0_is_return,

    input  wire                     update1_valid,
    input  wire [`INST_ADDR_WIDTH-1:0] update1_pc,
    input  wire                     update1_taken,
    input  wire [`INST_ADDR_WIDTH-1:0] update1_target,
    input  wire [GHR_BITS-1:0]      update1_hist,
    input  wire                     update1_is_call,
    input  wire                     update1_is_return
);

    localparam PHT_ENTRIES = (1 << IDX_BITS);
    localparam TAG_BITS    = `INST_ADDR_WIDTH - IDX_BITS - 2;
    localparam [(`INST_ADDR_WIDTH-1):0] FETCH_STRIDE = `IF_BATCH_SIZE * `INST_ADD_STEP;

    function [IDX_BITS-1:0] make_idx;
        input [GHR_BITS-1:0] hist;
        input [`INST_ADDR_WIDTH-1:0] pc;
        begin
            if (GHR_BITS >= IDX_BITS) begin
                make_idx = hist[IDX_BITS-1:0] ^ pc[IDX_BITS+1:2];
            end else begin
                make_idx = {{(IDX_BITS-GHR_BITS){1'b0}}, hist} ^ pc[IDX_BITS+1:2];
            end
        end
    endfunction

    // Pattern history table: 2-bit counters
    reg [1:0] pht [0:PHT_ENTRIES-1];

    // BTB
    reg [`INST_ADDR_WIDTH-1:0] btb_target [0:PHT_ENTRIES-1];
    reg [TAG_BITS-1:0]         btb_tag    [0:PHT_ENTRIES-1];
    reg                        btb_valid  [0:PHT_ENTRIES-1];
    reg                        btb_is_ret [0:PHT_ENTRIES-1];

    // RAS
    reg [`INST_ADDR_WIDTH-1:0] ras [0:RAS_DEPTH-1];
    reg [$clog2(RAS_DEPTH):0]  ras_sp;

    reg [GHR_BITS-1:0] ghr;

    wire [IDX_BITS-1:0] fetch_idx = make_idx(ghr, fetch_pc);
    wire [TAG_BITS-1:0] fetch_tag = fetch_pc[`INST_ADDR_WIDTH-1:IDX_BITS+2];
    wire btb_hit = btb_valid[fetch_idx] && (btb_tag[fetch_idx] == fetch_tag);
    // Only predict taken when BTB hits to avoid suppressing slot1 on non-branch PCs.
    assign pred_taken  = fetch_valid && btb_hit && pht[fetch_idx][1];
    wire [`INST_ADDR_WIDTH-1:0] ras_top = ras[(ras_sp==0)?{($clog2(RAS_DEPTH)+1){1'b0}}: (ras_sp-1'b1)];
    wire [`INST_ADDR_WIDTH-1:0] btb_target_sel = btb_is_ret[fetch_idx] ? ras_top : btb_target[fetch_idx];
    assign pred_target = (btb_hit ? btb_target_sel : (fetch_pc + FETCH_STRIDE));
    assign pred_hist   = ghr;

    integer i;
    // Helper: update a single counter/BTB/RAS/ghr
    task automatic apply_update;
        input                     vld;
        input [`INST_ADDR_WIDTH-1:0] pc;
        input                     taken;
        input [`INST_ADDR_WIDTH-1:0] target;
        input [GHR_BITS-1:0]      hist;
        input                     is_call;
        input                     is_return;
        reg [IDX_BITS-1:0] idx;
        reg [TAG_BITS-1:0] tag;
        begin
            if (vld) begin
                idx = make_idx(hist, pc);
                tag = pc[`INST_ADDR_WIDTH-1:IDX_BITS+2];
                // PHT update
                if (taken) begin
                    if (pht[idx] != 2'b11) pht[idx] = pht[idx] + 1'b1;
                end else begin
                    if (pht[idx] != 2'b00) pht[idx] = pht[idx] - 1'b1;
                end
                // BTB update on taken
                if (taken) begin
                    btb_target[idx] <= target;
                    btb_tag[idx]    <= tag;
                    btb_valid[idx]  <= 1'b1;
                    btb_is_ret[idx] <= is_return;
                end
                // RAS maintenance
                if (is_call) begin
                    ras[ras_sp[$clog2(RAS_DEPTH)-1:0]] <= pc + `INST_ADD_STEP;
                    if (ras_sp != RAS_DEPTH) ras_sp <= ras_sp + 1'b1;
                end else if (is_return && ras_sp != 0) begin
                    ras_sp <= ras_sp - 1'b1;
                end
                // GHR shift in actual outcome
                ghr <= {ghr[GHR_BITS-2:0], taken};
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ghr <= {GHR_BITS{1'b0}};
            ras_sp <= 0;
            for (i=0; i<PHT_ENTRIES; i=i+1) begin
                pht[i] <= 2'b01; // weakly not taken
                btb_target[i] <= {`INST_ADDR_WIDTH{1'b0}};
                btb_tag[i]    <= {TAG_BITS{1'b0}};
                btb_valid[i]  <= 1'b0;
                btb_is_ret[i] <= 1'b0;
            end
        end else if (flush) begin
            ghr <= {GHR_BITS{1'b0}};
            ras_sp <= 0;
        end else begin
            // Apply updates in-order
            apply_update(update0_valid, update0_pc, update0_taken, update0_target, update0_hist, update0_is_call, update0_is_return);
            apply_update(update1_valid, update1_pc, update1_taken, update1_target, update1_hist, update1_is_call, update1_is_return);
        end
    end

endmodule
