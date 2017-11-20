module mult_accum_gated(clk,clr,en,A,B,accum);

input clk,clr,en;
input [7:0] A,B;
output reg [63:0] accum;

reg [15:0] prod_reg;
reg en_stg2;
logic gclk2, clk_en_lat;

///////////////////////////////////////////
// Generate and flop product if enabled //
/////////////////////////////////////////
always_ff @(posedge gclk2) 
      prod_reg <= A*B;

assign gclk2 = clk & clk_en_lat;

always @(clk, en) begin
	if(!clk)	
		clk_en_lat <= en;
end

/////////////////////////////////////////////////////
// Pipeline the enable signal to accumulate stage //
///////////////////////////////////////////////////
always_ff @(posedge clk)
    en_stg2 <= en;

always_ff @(posedge gclk2)
    if (clr)
      accum <= 64'h0000000000000000;
    else if (clk & clk_en_lat)
      accum <= accum + prod_reg;

endmodule
