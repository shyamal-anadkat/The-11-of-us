module mult_accum_gated(clk,clr,en,A,B,accum);

input clk,clr,en;
input [7:0] A,B;
output reg [63:0] accum;

reg [15:0] prod_reg;
reg en_stg2;
logic gclk2, gclk1, clk_en_lat;

///////////////////////////////////////////
// Generate and flop product if enabled //
/////////////////////////////////////////
always_ff @(posedge gclk1) 
      prod_reg <= A*B;

assign gclk1 = clk & clk_en_lat;

always @(clk, en) begin
	if(!clk)	
		clk_en_lat <= en;
end

/////////////////////////////////////////////////////
// Pipeline the enable signal to accumulate stage //
///////////////////////////////////////////////////
always_ff @(posedge clk)
    en_stg2 <= en;

always @(gclk1, en_stg2) begin
	if(!gclk1)	
		gclk2 <= en_stg2;
end

always_ff @(posedge gclk2)
    if (clr | en_stg2)
      accum <= 64'h0000000000000000;
    else
      accum <= accum + prod_reg;

endmodule
