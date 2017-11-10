module A2D_Intf ();

input clk, rst_n;
input strt_cnv;
input [2:0] chnnl;
input MISO, wrt;

output logic cnv_cmplt;
output logic [11:0] res;
output logic SS_n, SCLK, MOSI, done;


//// instantiate SPImstr 16 ////
SPI_mstr16 SPImstr16(.SS_n(), 
	.SCLK(SCLK), 
	.MOSI(MOSI), 
	.done(done), 
	.rd_data(), 
	.MISO(MISO), 
	.wrt(wrt), 
	.cmd(), 
	.clk(clk), 
	.rst_n(rst_n));
 
endmodule