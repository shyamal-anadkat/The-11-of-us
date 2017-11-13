module A2D_test (LED, SS_n, SCLK, MOSI, MISO, rst_n, clk);

	output logic [7:0] LED;
	output logic SS_n, SCLK, MOSI;

	input rst_n, clk, MISO;

	logic cnv_cmplt, strt_cnv;
	logic [11:0] res;


	A2D_intf A2DINTF (
		.cnv_cmplt(cnv_cmplt), 
		.res(res), 
		.SS_n(SS_n), 
		.strt_cnv(strt_cnv), 
		.chnnl(3'd0),  
		.SCLK(SCLK), 
		.MOSI(MOSI), 
		.MISO(MISO), 
		.clk(clk), 
		.rst_n(rst_n));


	always_ff @(posedge clk or negedge rst_n) begin 
		if(!rst_n) 
			LED <= 8'h00;
		else if(cnv_cmplt)
			//upper 8 bits of conversion results
		LED <=res[11:4]; 
	end

endmodule