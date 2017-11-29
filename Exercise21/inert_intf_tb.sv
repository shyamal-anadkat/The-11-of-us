module inert_intf_tb();
reg clk, rst_n, motors_off, strt_cal, INT;
wire SS_n, SCLK, MISO, MOSI, vld, cal_done, INT, frnt, bck, lft, rght;
wire [15:0] ptch, roll, yaw;
reg [10:0] frnt_spd, bck_spd, lft_spd, rght_spd;

localparam SPEED = 11'h245;

CycloneIV cyc(.SS_n(SS_n),
	.SCLK(SCLK),
	.MISO(MISO),
	.MOSI(MOSI),
	.INT(INT),
	.frnt_ESC(frnt),
	.back_ESC(bck),
	.left_ESC(lft),
	.rght_ESC(rght));

ESCs #(18) escs(.clk(clk), 
	.rst_n(rst_n), 
	.frnt_spd(frnt_spd), 
	.bck_spd(bck_spd), 
	.lft_spd(lft_spd), 
	.rght_spd(rght_spd), 
	.motors_off(motos_off), 
	.frnt(frnt), 
	.bck(bck), 
	.lft(lft), 
	.rght(rght));

inert_intf intf(.clk(clk), 
	.rst_n(rst_n), 
	.strt_cal(strt_cal), 
	.SS_n(SS_n), 
	.SCLK(SCLK), 
	.MOSI(MOSI), 
	.MISO(MISO), 
	.INT(INT),
	.vld(vld), 
	.cal_done(cal_done), 
	.ptch(ptch), 
	.roll(roll), 
	.yaw(yaw));

initial begin
	clk = 0;
	rst_n = 0;
	strt_cal = 0;
	motors_off = 0;
	frnt_spd = SPEED;
	bck_spd = SPEED;
	lft_spd = SPEED;
	rght_spd = SPEED;
end

always
	#10 clk = ~clk;

always begin
	repeat(2)@(posedge clk);
	rst_n = 1;
	// test calibration
	strt_cal = 1;
	@(posedge clk);
	strt_cal = 0;
	repeat(10)@(posedge clk);
	// test that motors are not being driven
	motors_off = 1;
	repeat(5)@(posedge clk);
	motors_off = 0;
	// test other things
	repeat(32)@(posedge SCLK);
	$stop();
end
endmodule
