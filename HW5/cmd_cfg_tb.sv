module cmd_cfg_tb();
reg clk, rst_n, cmd_rdy, cal_done, cnv_cmplt;
reg [7:0] cmd, batt;
reg [15:0] data;
wire clr_cmd_rdy, send_resp, strt_cal, interial_cal, motors_off, strt_cnv;
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
	.inertial_cal(intertial_cal),
	.cal_done(cal_done),
	.motors_off(motors_off),
	.strt_cnv(strt_cnv),
	.cnv_cmplt(cnv_cmplt));

localparam NUM_CMDS = 8;
reg [24:0] commands [NUM_CMDS-1:0];

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
	commands[0] = {REQ_BATT, 16'h0000};
	// set pitch w/ value 6
	commands[1] = {SET_PTCH, 16'h0006};
	// set roll w/ value 4
	commands[2] = {SET_ROLL, 16'h0004};
	// set yaw w/ value 2
	commands[3] = {SET_YAW,  16'h0002};
	// set thrst with value 8
	commands[4] = {SET_THRST, 16'h0008};
	// cal copter
	commands[5] = {CALIBRATE, 16'h0000};
	// Emer land
	commands[6] = {EMER_LAND, 16'h0000};
	// motors off
	commands[7] = {MTRS_OFF, 16'h0000};
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
		cmd_rdy = 1'b1;
		@(posedge send_resp);
		
	end
end


endmodule
