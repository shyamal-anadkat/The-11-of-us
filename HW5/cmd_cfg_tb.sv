/*
* Testbench for cmd_cfg unit.
* Eric Heinz, Shyamal Anadkat, Sanjay Rajmohan 
* Ultimately will be project's top level testbench 
* Tests that all the basic commands work. 
*/
module cmd_cfg_tb ();


//// internal wires/stimuli ////
logic clk, rst_n, cnv_cmplt, cal_done, snd_cmd_stim;
logic [7:0] resp, batt, cmd_to_cfg, cmd_stim;
logic cmd_rdy, clr_cmd_rdy, snd_resp, resp_sent;
logic [15:0] d_ptch, d_roll, d_yaw, data, data_to_cfg, data_stim;
logic [8:0] thrst;
logic [7:0] resp_commMaster;
logic motors_off, strt_cnv, inertial_cal;
logic RX_TX, TX_RX;


//// command encodings ////
localparam REQ_BATT 	= 8'h01;
localparam SET_PTCH 	= 8'h02;
localparam SET_ROLL	= 8'h03;
localparam SET_YAW  	= 8'h04;
localparam SET_THRST 	= 8'h05;
localparam CALIBRATE 	= 8'h06;
localparam EMER_LAND 	= 8'h07;
localparam MTRS_OFF 	= 8'h08;

// response params
localparam ACK = 8'ha5;


//// CommMaster iDUT instance. We apply stim to inputs to CommMaster ////
CommMaster masteriDUT(   
	.resp(resp_commMaster), 
	.resp_rdy(resp_rdy), 
	.frm_snt(frm_snt), 
	.TX(TX_RX), .RX(RX_TX), 
	.cmd(cmd_stim), 
	.snd_cmd(snd_cmd_stim), 
	.data(data_stim), 
	.clk(clk), .rst_n(rst_n));

//// UART Wrapper instantiation ////
UART_wrapper UART_wrapper_iDUT(
	//resp is input to wrapper
	.clk(clk), .rst_n(rst_n), 
	.RX(TX_RX), .TX(RX_TX), 
	.cmd(cmd_to_cfg), 
	.data(data_to_cfg), 
	.cmd_rdy(cmd_rdy), 
	.snd_resp(snd_resp), 
	.resp_sent(resp_sent), 
	.resp(resp), 
	.clr_cmd_rdy(clr_cmd_rdy));

//// cmd_cfg unit instantiation ////
cmd_cfg cmd_cfgiDUT (
	.d_ptch(d_ptch), .d_roll(d_roll),  .d_yaw(d_yaw), 
	.thrst (thrst),
	.resp(resp), 
	.send_resp(snd_resp), 
	.clr_cmd_rdy(clr_cmd_rdy), 
	.strt_cal(strt_cal), 
	.inertial_cal(inertial_cal),
	.motors_off(motors_off), 
	.strt_cnv(strt_cnv), 
	.data(data_to_cfg), 
	.cmd(cmd_to_cfg), 
	.batt(batt), 
	.cal_done(cal_done), 
	.cnv_cmplt(cnv_cmplt), 
	.cmd_rdy(cmd_rdy), .clk(clk), .rst_n(rst_n));


localparam NUM_CMDS = 8;
reg [7:0] commands [NUM_CMDS-1:0];
reg [15:0] datas [NUM_CMDS-1:0];
initial begin
	clk = 0;
	rst_n = 0;
	cal_done = 0;
	//cnv_cmplt = 0;
	batt = 8'h21;
	cnv_cmplt = 0;
	snd_cmd_stim = 0;
	cmd_stim = 0;
	data_stim = 0;
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
	@(negedge clk);
	rst_n = 1'b1;
	@(posedge clk);
	for (i = 0; i < NUM_CMDS; i=i+1) begin
		cmd_stim = commands[i];
		data_stim = datas[i];
		snd_cmd_stim = 1;
		@(posedge clk);
		snd_cmd_stim = 0;
		@(posedge cmd_rdy);
		@(posedge clk);
		@(posedge clk);
		snd_cmd_stim = 1'b0;
		if (i == 7) begin
		    @(posedge resp_rdy);
			cmd_stim = 8'h06;
			snd_cmd_stim = 1;
			@(posedge clk);
			snd_cmd_stim = 0;
			@(posedge cmd_rdy);
			@(posedge clk);
			@(posedge clk);
			snd_cmd_stim = 1'b0;
		end
		if (i == 0) begin
			// model ADC
			cnv_cmplt = 1'b1;
			if (resp != batt) begin
				$display("Resp should be battery level");
				$stop();
			end
		end else begin
			if (resp != ACK) begin
				$display("Resp should be an ack");
				$stop();
			end
		end
		if (i == 5 || i == 7)
			cal_done = 1'b1;
		@(posedge resp_rdy); // change to frm_snt if you want this to fully finish
	end
	$display("Tests finished. Please review the wave.");
	$stop();
end
endmodule
