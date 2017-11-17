module cmd_cfg_tb();
reg clk, rst_n, cmd_rdy, cal_done, cnv_cmplt;
reg [7:0] cmd, batt;
reg [15:0] data;
wire clr_cmd_rdy, send_resp, strt_cal, inertial_cal, motors_off, strt_cnv;
wire [7:0] resp;
wire [15:0] d_ptch, d_yaw, d_roll;
wire [8:0] thrst;

localparam REQ_BATT = 8'h01;
localparam SET_PTCH = 8'h02;
localparam SET_ROLL = 8'h03;
localparam SET_YAW = 8'h04;
localparam SET_THRST = 8'h05;
localparam CALIBRATE = 8'h06;
localparam EMER_LAND = 8'h07;
localparam MTRS_OFF = 8'h08;

cmd_cfg DUT(
	.clk(clk),
	.rst_n(rst_n),
	.cmd_rdy(cmd_rdy),
	.cmd(cmd),
	.data(data),
	.clr_cmd_rdy(clr_cmd_rdy),
	.resp(resp),
	.send_resp(send_resp),
	.d_ptch(d_ptch),
	.d_roll(d_roll),
	.d_yaw(d_yaw),
	.thrst(thrst),
	.batt(batt),
	.strt_cal(strt_cal),
	.inertial_cal(inertial_cal),
	.cal_done(cal_done),
	.motors_off(motors_off),
	.strt_cnv(strt_cnv),
	.cnv_cmplt(cnv_cmplt));

localparam NUM_CMDS = 8;
reg [7:0] commands [NUM_CMDS-1:0];
reg [15:0] datas [NUM_CMDS-1:0];
initial begin
	clk = 0;
	rst_n = 0;
	cmd_rdy = 0;
	cal_done = 0;
	cnv_cmplt = 0;
	cmd = 0;
	batt = 0;
	data = 0;
	// battery cmd
	commands[0] = REQ_BATT;
	// set pitch w/ value 6
	commands[1] = SET_PTCH;
	// set roll w/ value 4
	commands[2] = SET_ROLL;
	// set yaw w/ value 2
	commands[3] = SET_YAW;
	// set thrst with value 8
	commands[4] = SET_THRST;
	// cal copter
	commands[5] = CALIBRATE;
	// Emer land
	commands[6] = EMER_LAND;
	// motors off
	commands[7] = MTRS_OFF;
	
	// battery cmd
	datas[0] = 16'h0000;
	// set pitch w/ value 6
	datas[1] = 16'h0006;
	// set roll w/ value 4
	datas[2] = 16'h0004;
	// set yaw w/ value 2
	datas[3] = 16'h0002;
	// set thrst with value 8
	datas[4] = 16'h0008;
	// cal copter
	datas[5] = 16'h0000;
	// Emer land
	datas[6] = 16'h0000;
	// motors off
	datas[7] = 16'h0000;
end

always
	#10 clk = ~clk;

integer i;
always begin
	@(posedge clk);
	rst_n = 1'b1;
	@(posedge clk);
	for (i = 0; i < NUM_CMDS; i=i+1) begin
		cmd = commands[i];
		data = datas[i];
		cmd_rdy = 1'b1;
		@(posedge clk);
		@(posedge clk);
		cmd_rdy = 1'b0;
		if (i == 0)
			cnv_cmplt = 1'b1;
		if (i == 5)
			cal_done = 1'b1;
		@(posedge send_resp);
		
	end
end


endmodule
