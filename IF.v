`include "riscv_define.v"
// Fetch instruction from I$, predict branch, and send to decode

// TODO: connect cache/memory interface
// TODO: add branch prediction logic
// TODO: add exception handling logic
// TODO: batch instruction fetch to instruction buffer
module IF(
    input clk,
    input rst_n,

    // Inputs


    // Outputs
    output [`INST_ADDR_WIDTH-1:0] out_inst_0,
    output [`INST_ADDR_WIDTH-1:0] out_inst_1,
    output [`INST_ADDR_WIDTH-1:0] out_inst_2,
    output [`INST_ADDR_WIDTH-1:0] out_inst_3,
    output [`IF_BATCH_SIZE-1:0]   out_inst_valid, // mask to indicate which instruction is valid 
);
reg [`INST_ADDR_WIDTH-1:0] reg_PC;
reg                   first_clk_passed;  // reserved for memory related timing
reg                   second_clk_passed; // reserved for memory related timing

// TODO: Adapt to batch fetching
assign next_PC = second_clk_passed ? reg_PC + `INST_ADD_STEP : reg_PC;

always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        reg_PC <= `INST_INIT;
        first_clk_passed <= 1'b0;
        second_clk_passed <= 1'b0;
    end else begin
        reg_PC <= next_PC;
        first_clk_passed <= 1'b1;
        second_clk_passed <= first_clk_passed;
    end
end


endmodule