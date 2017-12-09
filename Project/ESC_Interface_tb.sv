module ESC_interface_tb();

`timescale 1ns/1ps

reg [10:0] SPEED;
reg [9:0] OFF;
wire PWM;
reg clk, rst_n;

ESC_interface DUT (.SPEED(SPEED), .OFF(OFF), .PWM(PWM), .clk(clk), .rst_n(rst_n));

initial begin
	rst_n = 1'b1;
	SPEED = 11'd0;
	OFF = 10'd0;
	clk = 1'd0;
end

// 50 MHz clk with 20 ns period
always begin
	#10 clk <= ~clk;
end

always begin
	rst_n = 1'd0;
	#10
	rst_n = 1'd1;
	SPEED = 11'd512;
	OFF = 10'd256;
	
	//should start PWMing here
	#20971528
	$stop();
end

endmodule
