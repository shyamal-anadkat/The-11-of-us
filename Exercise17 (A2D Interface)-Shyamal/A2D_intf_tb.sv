module A2D_intf_tb ();

	logic clk, rst_n;
	logic SS_n, SCLK, MOSI, MISO, strt_cnv, cnv_cmplt;
	logic [2:0] chnnl;
	logic [11:0] res;

//// instantiate iDUTs ////
A2D_intf  iDUTA2D(
	.cnv_cmplt(cnv_cmplt), 
	.res(res), 
	.SS_n(SS_n),
	.strt_cnv(strt_cnv), 
	.chnnl(chnnl),
	.SCLK(SCLK), 
	.MOSI(MOSI), 
	.MISO(MISO), 
	.clk(clk), 
	.rst_n(rst_n));

ADC128S iDUTADC(
	.clk(clk),
	.rst_n(rst_n),
	.SS_n(SS_n),
	.SCLK(SCLK),
	.MISO(MISO),
	.MOSI(MOSI));

initial begin 



end


endmodule