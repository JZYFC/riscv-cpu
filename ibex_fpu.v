`include "riscv_define.v"

module ibex_fpu(
    input  [31:0]                          A_i,
    input  [31:0]                          B_i,
    input  [31:0]                          C_i,
    input  [3:0]                           opcode_i,
    input  [`ROUNDING_MODE_WIDTH-1:0]      rounding_mode_i,

    output reg [31:0]                 result_o,
    output reg                        exception_flag_o
);
    
    wire a_sign;
    wire [7:0] a_exponent;
    wire [23:0] a_mantissa;
    wire b_sign;
    wire [7:0] b_exponent;
    wire [23:0] b_mantissa;
    wire c_sign;
    wire [7:0] c_exponent; //
    wire [23:0] c_mantissa; //

    wire [31:0] adder_out;
    wire [31:0] madder_out; //
    wire [31:0] multiplier_out;
    wire less_out;
    wire eq_out;

    wire ADD;
    wire SUB;
    wire MUL;
    wire MADD;
    wire MSUB;
    wire NMSUB;
    wire NMADD;
    wire SUBABS;
    wire EQ;
    wire LT;

    assign a_sign = A_i[31];
    assign a_exponent[7:0] = A_i[30:23];
    assign a_mantissa[23:0] = {1'b1, A_i[22:0]};

    assign b_sign = B_i[31];
    assign b_exponent[7:0] = B_i[30:23];
    assign b_mantissa[23:0] = {1'b1, B_i[22:0]};

    assign c_sign = C_i[31];                      //
    assign c_exponent[7:0] = C_i[30:23];          //
    assign c_mantissa[23:0] = {1'b1, C_i[22:0]};  // 

    assign ADD    = (opcode_i == `FP_OP_ADD);
    assign SUB    = (opcode_i == `FP_OP_SUB);
    assign MUL    = (opcode_i == `FP_OP_MUL);
    assign MADD   = (opcode_i == `FP_OP_MADD);
    assign MSUB   = (opcode_i == `FP_OP_MSUB);
    assign NMSUB  = (opcode_i == `FP_OP_NMSUB);
    assign NMADD  = (opcode_i == `FP_OP_NMADD);
    assign SUBABS = (opcode_i == `FP_OP_SUBABS);
    assign EQ     = (opcode_i == `FP_OP_FEQ);
    assign LT     = (opcode_i == `FP_OP_FLT);

    // combinational inputs to submodules (avoid procedural reg driving to prevent delta-cycle stale reads)
    wire [31:0] adder_a_in;
    wire [31:0] adder_b_in;
    wire [31:0] madder_b_in;

    assign adder_a_in = A_i;
    assign adder_b_in = (SUB || SUBABS) ? {~B_i[31], B_i[30:0]} : B_i;
    assign madder_b_in = (MSUB || NMSUB) ? {~C_i[31], C_i[30:0]} : C_i;

    adder A1
    (
        .a(adder_a_in),
        .b(adder_b_in),
        .out(adder_out)
    );

    adder A2
    (
        .a(multiplier_out),
        .b(madder_b_in),
        .out(madder_out)
    );

    multiplier M1
    (
        .a(A_i),
        .b(B_i),
        .out(multiplier_out)
    );

    fless L1
    ( 
        .a(A_i),
        .b(B_i),
        .c(less_out)
    );

    feq E1
    (
        .a(A_i),
        .b(B_i),
        .c(eq_out)
    );

    always @(*) begin
        reg [31:0] corner;
        corner = 32'b0;
        result_o = 32'b0;
        exception_flag_o = 1'b0;

        if (ADD) begin
            //If a is NaN or b is zero return a
            if ((a_exponent == 255 && a_mantissa != 0) || (b_exponent == 0) && (b_mantissa == 0)) begin
                result_o = A_i;
            //If b is NaN or a is zero return b
            end else if ((b_exponent == 255 && b_mantissa != 0) || (a_exponent == 0) && (a_mantissa == 0)) begin
                result_o = B_i;
            //if a or b is inf return inf
            end else if ((a_exponent == 255) || (b_exponent == 255)) begin
                result_o = {a_sign ^ b_sign, 8'hff, 23'b0};
            end else begin // Passed all corner cases
                result_o = adder_out;
            end
        end else if (SUB) begin
            //If a is NaN or b is zero return a
            if ((a_exponent == 255 && a_mantissa != 0) || (b_exponent == 0) && (b_mantissa == 0)) begin
                result_o = A_i;
            //If b is NaN or a is zero return b
            end else if ((b_exponent == 255 && b_mantissa != 0) || (a_exponent == 0) && (a_mantissa == 0)) begin
                result_o = B_i;
            //if a or b is inf return inf
            end else if ((a_exponent == 255) || (b_exponent == 255)) begin
                result_o = {a_sign ^ b_sign, 8'hff, 23'b0};
            end else if (A_i == B_i) begin  // equal numbers subtraction
                result_o = 32'b0;
            end else begin // Passed all corner cases
                result_o = adder_out;
            end
        end else if (MUL) begin //Multiplication
            //If a is NaN return NaN
            if (a_exponent == 255 && a_mantissa != 0) begin
                result_o = A_i;
            //If b is NaN return NaN
            end else if (b_exponent == 255 && b_mantissa != 0) begin
                result_o = B_i;
            //If a or b is 0 return 0
            end else if ((a_exponent == 0) && (a_mantissa == 0) || (b_exponent == 0) && (b_mantissa == 0)) begin
                result_o = {a_sign ^ b_sign, 8'h00, 23'b0};
            //if a or b is inf return inf
            end else if ((a_exponent == 255) || (b_exponent == 255)) begin
                result_o = {a_sign, 8'hff, 23'b0};
            end else begin // Passed all corner cases
                result_o = multiplier_out;
            end
        end else if (SUBABS) begin
            //If a is NaN or b is zero return a
            if ((a_exponent == 255 && a_mantissa != 0) || (b_exponent == 0) && (b_mantissa == 0)) begin
                result_o = {1'b0, A_i[30:0]};
            //If b is NaN or a is zero return b
            end else if ((b_exponent == 255 && b_mantissa != 0) || (a_exponent == 0) && (a_mantissa == 0)) begin
                result_o = {1'b0, B_i[30:0]};
            //if a or b is inf return inf
            end else if ((a_exponent == 255) || (b_exponent == 255)) begin
                result_o = {1'b0, 8'hff, 23'b0};
            end else if (A_i == B_i) begin  // equal numbers subtraction
                result_o = 32'b0;
            end else begin // Passed all corner cases
                result_o = {1'b0, adder_out[30:0]};
            end
        end
        else if (MADD) begin   // FIXME the corner cases need to be reconsidered 
            // if a is NaN return NaN
            if (a_exponent == 255 && a_mantissa != 0) begin
                result_o = A_i;  // if b is NaN return NaN
            end else if (b_exponent == 255 && b_mantissa != 0) begin
                result_o = B_i;  // if c is 0 return a*b
            end else if ((c_exponent == 0) && (c_mantissa == 0)) begin
                result_o = multiplier_out;    //If a is 0 or b is 0 or c is NaN return c 
            end else if ((b_exponent == 0 && b_mantissa == 0) || (a_exponent == 0) && (a_mantissa == 0) || (c_exponent == 255 && c_mantissa != 0)) begin
                result_o = C_i;
            //if a is inf or b is inf or c is inf return inf
            end else if (a_exponent == 255) begin
                result_o = {a_sign ^ c_sign, 8'hff, 23'b0};
            end else if (b_exponent == 255) begin
                result_o = {b_sign ^ c_sign, 8'hff, 23'b0};
            end else if (c_exponent == 255) begin
                result_o = {multiplier_out[31] ^ c_sign, 8'hff, 23'b0};
            end else begin 
                result_o = madder_out;
            end
        end else if (MSUB) begin
            // if a is NaN return NaN
            if (a_exponent == 255 && a_mantissa != 0) begin
                result_o = A_i;  // if b is NaN return NaN
            end else if (b_exponent == 255 && b_mantissa != 0) begin
                result_o = B_i;  // if c is 0 return a*b
            end else if ((c_exponent == 0) && (c_mantissa == 0)) begin
                result_o = multiplier_out;    //If a is 0 or b is 0 or c is NaN return c 
            end else if ((b_exponent == 0 && b_mantissa == 0) || (a_exponent == 0) && (a_mantissa == 0) || (c_exponent == 255 && c_mantissa != 0)) begin
                result_o = C_i;
            //if a is inf or b is inf or c is inf return inf
            end else if (a_exponent == 255) begin
                result_o = {a_sign ^ c_sign, 8'hff, 23'b0};
            end else if (b_exponent == 255) begin
                result_o = {b_sign ^ c_sign, 8'hff, 23'b0};
            end else if (c_exponent == 255) begin
                result_o = {multiplier_out[31] ^ c_sign, 8'hff, 23'b0};
            end else begin // Passed all corner cases
                result_o = madder_out;
            end
        end else if (NMADD) begin 
            // if a is NaN return NaN
            if (a_exponent == 255 && a_mantissa != 0) begin
                result_o = A_i;  // if b is NaN return NaN
            end else if (b_exponent == 255 && b_mantissa != 0) begin
                result_o = B_i;  // if c is 0 return a*b
            end else if ((c_exponent == 0) && (c_mantissa == 0)) begin
                result_o = multiplier_out;    //If a is 0 or b is 0 or c is NaN return c 
            end else if ((b_exponent == 0 && b_mantissa == 0) || (a_exponent == 0) && (a_mantissa == 0) || (c_exponent == 255 && c_mantissa != 0)) begin
                result_o = C_i;
            //if a is inf or b is inf or c is inf return inf
            end else if (a_exponent == 255) begin
                result_o = {a_sign ^ c_sign, 8'hff, 23'b0};
            end else if (b_exponent == 255) begin
                result_o = {b_sign ^ c_sign, 8'hff, 23'b0};
            end else if (c_exponent == 255) begin
                result_o = {multiplier_out[31] ^ c_sign, 8'hff, 23'b0};
            end else begin // Passed all corner cases
                result_o = {~madder_out[31], madder_out[30:0]};
            end
        end else if (NMSUB) begin //NMSUB
            // if a is NaN return NaN
            if (a_exponent == 255 && a_mantissa != 0) begin
                result_o = A_i;  // if b is NaN return NaN
            end else if (b_exponent == 255 && b_mantissa != 0) begin
                result_o = B_i;  // if c is 0 return a*b
            end else if ((c_exponent == 0) && (c_mantissa == 0)) begin
                result_o = multiplier_out;    //If a is 0 or b is 0 or c is NaN return c 
            end else if ((b_exponent == 0 && b_mantissa == 0) || (a_exponent == 0) && (a_mantissa == 0) || (c_exponent == 255 && c_mantissa != 0)) begin
                result_o = C_i;
            //if a is inf or b is inf or c is inf return inf
            end else if (a_exponent == 255) begin
                result_o = {a_sign ^ c_sign, 8'hff, 23'b0};
            end else if (b_exponent == 255) begin
                result_o = {b_sign ^ c_sign, 8'hff, 23'b0};
            end else if (c_exponent == 255) begin
                result_o = {multiplier_out[31] ^ c_sign, 8'hff, 23'b0};
            end else begin // Passed all corner cases
                result_o = {~madder_out[31], madder_out[30:0]};
            end
        end else if (EQ) begin
            // if either a or b is NaN
            if ((a_exponent == 255 && a_mantissa != 0) || (b_exponent == 255) && (b_mantissa != 0)) begin
                exception_flag_o = 1'b1;
                result_o = 32'b0;
            end
            else begin
                result_o = {31'b0, eq_out};
            end
        end else if (LT) begin
            if ((a_exponent == 255 && a_mantissa != 0) || (b_exponent == 255) && (b_mantissa != 0)) begin
                exception_flag_o = 1'b1;
                result_o = 32'b0;
            end
            else begin
                result_o = {31'b0, less_out};
            end
        end else begin
            result_o = 32'b0;
        end

    end 
endmodule


module adder(a, b, out);
  input  [31:0] a, b;
  output [31:0] out;

  wire [31:0] out;
  reg a_sign;
  reg [7:0] a_exponent;
  reg [23:0] a_mantissa;
  reg b_sign;
  reg [7:0] b_exponent;
  reg [23:0] b_mantissa;

  reg o_sign;
  reg [7:0] o_exponent;
  reg [24:0] o_mantissa;

  reg [7:0] diff;
  reg [23:0] tmp_mantissa;
  reg [7:0] tmp_exponent;


  reg  [7:0] i_e;
  reg  [24:0] i_m;
  wire [7:0] o_e;
  wire [24:0] o_m;

  addition_normaliser norm1
  (
    .in_e(i_e),
    .in_m(i_m),
    .out_e(o_e),
    .out_m(o_m)
  );

  assign out[31] = o_sign;
  assign out[30:23] = o_exponent;
  assign out[22:0] = o_mantissa[22:0];

    always @(*) begin
        a_sign = a[31];
        if(a[30:23] == 0) begin
            a_exponent = 8'b00000001;        // 若阶数为0，则让a阶数+1，位数右移一位
            a_mantissa = {1'b0, a[22:0]};
        end else begin
            a_exponent = a[30:23];
            a_mantissa = {1'b1, a[22:0]};    // CONFUESD 不是很懂这个1是什么意思
        end
        b_sign = b[31];
        if(b[30:23] == 0) begin
            b_exponent = 8'b00000001;
            b_mantissa = {1'b0, b[22:0]};
        end else begin
            b_exponent = b[30:23];
            b_mantissa = {1'b1, b[22:0]};
        end
    if (a_exponent == b_exponent) begin // Equal exponents
      o_exponent = a_exponent;
      if (a_sign == b_sign) begin // Equal signs = add
        o_mantissa = a_mantissa + b_mantissa;   // 尾数设计多出一位是为了尾数相加溢出时可以知道确实溢出了
        //Signify to shift
        o_mantissa[24] = 1;                     // CONFUSED
        o_sign = a_sign;
      end else begin // result_opposite signs = subtract
        if(a_mantissa > b_mantissa) begin
          o_mantissa = a_mantissa - b_mantissa;
          o_sign = a_sign;
        end else begin
          o_mantissa = b_mantissa - a_mantissa;
          o_sign = b_sign;
        end
      end
    end else begin //Unequal exponents
      if (a_exponent > b_exponent) begin // A_i is bigger
        o_exponent = a_exponent;
        o_sign = a_sign;
        diff = a_exponent - b_exponent;
        tmp_mantissa = b_mantissa >> diff;
        if (a_sign == b_sign)
          o_mantissa = a_mantissa + tmp_mantissa;
        else
          o_mantissa = a_mantissa - tmp_mantissa;
      end else if (a_exponent < b_exponent) begin // B_i is bigger
        o_exponent = b_exponent;
        o_sign = b_sign;
        diff = b_exponent - a_exponent;
        tmp_mantissa = a_mantissa >> diff;
        if (a_sign == b_sign) begin
          o_mantissa = b_mantissa + tmp_mantissa;
        end else begin
                    o_mantissa = b_mantissa - tmp_mantissa;
        end
      end
    end
    if(o_mantissa[24] == 1) begin
      o_exponent = o_exponent + 1;
      o_mantissa = o_mantissa >> 1;
    end else if((o_mantissa[23] != 1) && (o_exponent != 0)) begin
      i_e = o_exponent;
      i_m = o_mantissa;
      o_exponent = o_e;
      o_mantissa = o_m;
    end
  end
endmodule

module multiplier(a, b, out);
  input  [31:0] a, b;
  output [31:0] out;

  wire [31:0] out;
    reg a_sign;
  reg [7:0] a_exponent;
  reg [23:0] a_mantissa;
    reg b_sign;
  reg [7:0] b_exponent;
  reg [23:0] b_mantissa;

  reg o_sign;
  reg [7:0] o_exponent;
  reg [24:0] o_mantissa;

    reg [47:0] product;

  assign out[31] = o_sign;
  assign out[30:23] = o_exponent;
  assign out[22:0] = o_mantissa[22:0];

    reg  [7:0] i_e;
    reg  [47:0] i_m;
    wire [7:0] o_e;
    wire [47:0] o_m;

    multiplication_normaliser norm1
    (
        .in_e(i_e),
        .in_m(i_m),
        .out_e(o_e),
        .out_m(o_m)
    );


  always @ ( * ) begin
        a_sign = a[31];
        if(a[30:23] == 0) begin
            a_exponent = 8'b00000001;
            a_mantissa = {1'b0, a[22:0]};
        end else begin
            a_exponent = a[30:23];
            a_mantissa = {1'b1, a[22:0]};
        end
        b_sign = b[31];
        if(b[30:23] == 0) begin
            b_exponent = 8'b00000001;
            b_mantissa = {1'b0, b[22:0]};
        end else begin
            b_exponent = b[30:23];
            b_mantissa = {1'b1, b[22:0]};
        end
    o_sign = a_sign ^ b_sign;
    o_exponent = a_exponent + b_exponent - 127;
    product = a_mantissa * b_mantissa;
        // Normalization
    if(product[47] == 1) begin
      o_exponent = o_exponent + 1;
      product = product >> 1;
    end else if((product[46] != 1) && (o_exponent != 0)) begin
      i_e = o_exponent;
      i_m = product;
      o_exponent = o_e;
      product = o_m;
    end
        o_mantissa = product[46:23];
    end
endmodule

module addition_normaliser(in_e, in_m, out_e, out_m);
  input [7:0] in_e;
  input [24:0] in_m;
  output [7:0] out_e;
  output [24:0] out_m;

  wire [7:0] in_e;
  wire [24:0] in_m;
  reg [7:0] out_e;
  reg [24:0] out_m;

  always @ ( * ) begin
        if (in_m[23:3] == 21'b000000000000000000001) begin
            out_e = in_e - 20;
            out_m = in_m << 20;
        end else if (in_m[23:4] == 20'b00000000000000000001) begin
            out_e = in_e - 19;
            out_m = in_m << 19;
        end else if (in_m[23:5] == 19'b0000000000000000001) begin
            out_e = in_e - 18;
            out_m = in_m << 18;
        end else if (in_m[23:6] == 18'b000000000000000001) begin
            out_e = in_e - 17;
            out_m = in_m << 17;
        end else if (in_m[23:7] == 17'b00000000000000001) begin
            out_e = in_e - 16;
            out_m = in_m << 16;
        end else if (in_m[23:8] == 16'b0000000000000001) begin
            out_e = in_e - 15;
            out_m = in_m << 15;
        end else if (in_m[23:9] == 15'b000000000000001) begin
            out_e = in_e - 14;
            out_m = in_m << 14;
        end else if (in_m[23:10] == 14'b00000000000001) begin
            out_e = in_e - 13;
            out_m = in_m << 13;
        end else if (in_m[23:11] == 13'b0000000000001) begin
            out_e = in_e - 12;
            out_m = in_m << 12;
        end else if (in_m[23:12] == 12'b000000000001) begin
            out_e = in_e - 11;
            out_m = in_m << 11;
        end else if (in_m[23:13] == 11'b00000000001) begin
            out_e = in_e - 10;
            out_m = in_m << 10;
        end else if (in_m[23:14] == 10'b0000000001) begin
            out_e = in_e - 9;
            out_m = in_m << 9;
        end else if (in_m[23:15] == 9'b000000001) begin
            out_e = in_e - 8;
            out_m = in_m << 8;
        end else if (in_m[23:16] == 8'b00000001) begin
            out_e = in_e - 7;
            out_m = in_m << 7;
        end else if (in_m[23:17] == 7'b0000001) begin
            out_e = in_e - 6;
            out_m = in_m << 6;
        end else if (in_m[23:18] == 6'b000001) begin
            out_e = in_e - 5;
            out_m = in_m << 5;
        end else if (in_m[23:19] == 5'b00001) begin
            out_e = in_e - 4;
            out_m = in_m << 4;
        end else if (in_m[23:20] == 4'b0001) begin
            out_e = in_e - 3;
            out_m = in_m << 3;
        end else if (in_m[23:21] == 3'b001) begin
            out_e = in_e - 2;
            out_m = in_m << 2;
        end else if (in_m[23:22] == 2'b01) begin
            out_e = in_e - 1;
            out_m = in_m << 1;
        end
  end
endmodule

module multiplication_normaliser(in_e, in_m, out_e, out_m);
  input [7:0] in_e;
  input [47:0] in_m;
  output [7:0] out_e;
  output [47:0] out_m;

  wire [7:0] in_e;
  wire [47:0] in_m;
  reg [7:0] out_e;
  reg [47:0] out_m;

  always @ ( * ) begin
      if (in_m[46:41] == 6'b000001) begin
            out_e = in_e - 5;
            out_m = in_m << 5;
        end else if (in_m[46:42] == 5'b00001) begin
            out_e = in_e - 4;
            out_m = in_m << 4;
        end else if (in_m[46:43] == 4'b0001) begin
            out_e = in_e - 3;
            out_m = in_m << 3;
        end else if (in_m[46:44] == 3'b001) begin
            out_e = in_e - 2;
            out_m = in_m << 2;
        end else if (in_m[46:45] == 2'b01) begin
            out_e = in_e - 1;
            out_m = in_m << 1;
        end
  end
endmodule

module fless(
    input wire [31:0] a,
    input wire [31:0] b,
    output wire c
);
    wire s_a;
    wire s_b;
    wire [7:0] e_a;
    wire [7:0] e_b;
    wire [22:0] m_a;
    wire [22:0] m_b;

    assign s_a = a[31];
    assign s_b = b[31];
    assign e_a = a[30:23];
    assign e_b = b[30:23];
    assign m_a = a[22:0];
    assign m_b = b[22:0];
    
    wire [1:0] sel_s;

    assign sel_s =
        (~s_a & s_b) ? 2'd0 :
        (s_a & ~s_b) ? 2'd1 :
        (s_a & s_b) ? 2'd2 : 2'd3;
    
    assign c = 
    (a == 32'h80000000 && b == 32'h00000000) ? 0 :
    (sel_s == 1) ? 1 : 
    (sel_s == 2 && e_a > e_b) ? 1 :
    (sel_s == 3 && e_a < e_b) ? 1 :
    (sel_s == 2 && e_a == e_b && m_a > m_b) ? 1 :
    (sel_s == 3 && e_a == e_b && m_a < m_b) ? 1 : 0;
endmodule


module feq(
    input wire [31:0] a,
    input wire [31:0] b,
    output wire c
    );
    assign c = 
    (a == 32'h80000000 && b == 32'h00000000) ? 1 :
    (a == 32'h00000000 && b == 32'h80000000) ? 1 :
    a == b ? 1 :
    0;
endmodule