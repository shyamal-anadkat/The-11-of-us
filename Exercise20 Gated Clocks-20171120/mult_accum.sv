module mult_accum(clk,clr,en,A,B,accum);

input clk,clr,en;
input [7:0] A,B;
output reg [63:0] accum;

reg [15:0] prod_reg;
reg en_stg2;

///////////////////////////////////////////
// Generate and flop product if enabled //
/////////////////////////////////////////
always_ff @(posedge clk)
    if (en)
      prod_reg <= A*B;

/////////////////////////////////////////////////////
// Pipeline the enable signal to accumulate stage //
///////////////////////////////////////////////////
always_ff @(posedge clk)
    en_stg2 <= en;

always_ff @(posedge clk)
    if (clr)
      accum <= 64'h0000000000000000;
    else if (en_stg2)
      accum <= accum + prod_reg;

endmodule
