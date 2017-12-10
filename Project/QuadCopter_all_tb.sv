`timescale 1ns/1ps
module QuadCopter_all_tb();
			
//// Interconnects to DUT/support defined as type wire /////
wire SS_n,SCLK,MOSI,MISO,INT;
wire SS_A2D_n,SCLK_A2D,MOSI_A2D,MISO_A2D;
wire RX,TX;
wire [7:0] resp;				// response from DUT
wire cmd_sent,resp_rdy;
wire frnt_ESC, back_ESC, left_ESC, rght_ESC;

////// Stimulus is declared as type reg ///////
reg clk, RST_n;
reg [7:0] cmd_to_copter;		// command to Copter via wireless link
reg [15:0] data;				// data associated with command
reg send_cmd;					// asserted to initiate sending of command (to your CommMaster)
reg clr_resp_rdy;				// asserted to knock down resp_rdy

/////// declare any localparams here /////


////////////////////////////////////////////////////////////////
// Instantiate Physical Model of Copter with Inertial sensor //
//////////////////////////////////////////////////////////////	
CycloneIV iQuad(.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),.INT(INT),
                .frnt_ESC(frnt_ESC),.back_ESC(back_ESC),.left_ESC(left_ESC),
				.rght_ESC(rght_ESC));				  

///////////////////////////////////////////////////
// Instantiate Model of A2D for battery voltage //
/////////////////////////////////////////////////
ADC128S iA2D(.clk(clk),.rst_n(RST_n),.SS_n(SS_A2D_n),.SCLK(SCLK_A2D),
             .MISO(MISO_A2D),.MOSI(MOSI_A2D));			
	 
////// Instantiate DUT ////////
QuadCopter iDUT(.clk(clk),.RST_n(RST_n),.SS_n(SS_n),.SCLK(SCLK),.MOSI(MOSI),.MISO(MISO),
                .INT(INT),.RX(RX),.TX(TX),.LED(),.FRNT(frnt_ESC),.BCK(back_ESC),
				.LFT(left_ESC),.RGHT(rght_ESC),.SS_A2D_n(SS_A2D_n),.SCLK_A2D(SCLK_A2D),
				.MOSI_A2D(MOSI_A2D),.MISO_A2D(MISO_A2D));


//// Instantiate Master UART (used to send commands to Copter) //////
CommMaster iMSTR(.clk(clk), .rst_n(RST_n), .RX(TX), .TX(RX),
                 .cmd(cmd_to_copter), .data(data), .snd_cmd(send_cmd),
			.frm_snt(cmd_sent), .resp_rdy(resp_rdy),
			.resp(resp), .clr_resp_rdy(clr_resp_rdy));

//////////////////////////////////////////////
//          Command Opcodes                 //
//////////////////////////////////////////////
localparam REQ_BATT = 8'h01;
localparam SET_PTCH = 8'h02;
localparam SET_ROLL = 8'h03;
localparam SET_YAW = 8'h04;
localparam SET_THRST = 8'h05;
localparam CALIBRATE = 8'h06;
localparam EMER_LAND = 8'h07;
localparam MTRS_OFF = 8'h08;

localparam NO_DATA = 16'd0;

reg [10:0] prev_frnt_spd, prev_bck_spd, prev_rgt_spd, prev_lft_spd;

initial begin

  /// This is where we do the real work.
  // This section is be done as a bunch of calls to testing sub tasks contained in a separate file.

  // Ultimate command line sim cmd: 
  //  vsim -gui -novopt work.QuadCopter_all_tb; add wave -position insertpoint sim:/QuadCopter_all_tb/*; run

  Initialize();

  ///////////////////////////////////////////////////////
  // Test basic commands and responses                 //
  ///////////////////////////////////////////////////////
  SendCmd(.comd(REQ_BATT), .dat(NO_DATA));
  ChkResp(8'hC0);

  SendCmd(.comd(EMER_LAND), .dat(NO_DATA));
  ChkVal16(.act(iDUT.ifly.ptch), .exp(16'd0), .name("Ptch"));
  ChkVal16(.act(iDUT.ifly.yaw), .exp(16'd0), .name("Yaw"));
  ChkVal16(.act(iDUT.ifly.roll), .exp(16'd0), .name("Roll"));
  ChkVal16(.act(iDUT.ifly.thrst), .exp(16'd0), .name("Thrst"));
  ChkPosAck;

  SendCmd(.comd(MTRS_OFF), .dat(NO_DATA));
  ChkVal(.act(iDUT.motors_off), .exp(1'b1), .name("Motors"));
  ChkPosAck;
  ChkVal(.act(iDUT.motors_off), .exp(1'b1), .name("Motors")); // should stay high until calibrate

  SendCmd(.comd(CALIBRATE), .dat(NO_DATA));
  ChkVal(.act(iDUT.motors_off), .exp(1'b0), .name("Motors"));
  ChkVal16(.act(iDUT.ifly.ptch), .exp(16'd0), .name("Ptch"));
  ChkVal16(.act(iDUT.ifly.yaw), .exp(16'd0), .name("Yaw"));
  ChkVal16(.act(iDUT.ifly.roll), .exp(16'd0), .name("Roll"));
  ChkVal16(.act(iDUT.ifly.thrst), .exp(16'd0), .name("Thrst"));
  ChkVal(.act(iDUT.motors_off), .exp(1'b0), .name("Motors")); // should no longer be high
  ChkPosAck;

  SendCmd(.comd(SET_PTCH), .dat(16'h003a));
  ChkVal16(.act(iDUT.ifly.d_ptch), .exp(16'h003a), .name("DPtch"));
  ChkPosAck;

  SendCmd(.comd(SET_ROLL), .dat(16'h003a));
  ChkVal16(.act(iDUT.ifly.d_roll), .exp(16'h003a), .name("DRoll"));
  ChkPosAck;

  SendCmd(.comd(SET_YAW), .dat(16'h003a));
  ChkVal16(.act(iDUT.ifly.d_yaw), .exp(16'h003a), .name("DYaw"));
  ChkPosAck;

  SendCmd(.comd(SET_THRST), .dat(16'h00fd));
  ChkVal16(.act(iDUT.ifly.thrst), .exp(16'h0fd), .name("Thrst"));
  ChkPosAck;

  repeat(90)@(posedge frnt_ESC);
  // ptch roll and yaw should converge to the values we want
  // check if within 75%
  ChkPerc(.act(iDUT.ifly.ptch), .exp(iDUT.ifly.d_ptch), .perc(75), .name("Ptch"));
  ChkPerc(.act(iDUT.ifly.roll), .exp(iDUT.ifly.d_roll), .perc(75), .name("Roll"));
  ChkPerc(.act(iDUT.ifly.yaw), .exp(iDUT.ifly.d_yaw), .perc(75), .name("Yaw"));

  // does increasing thrst make motors faster?
  prev_frnt_spd = iDUT.frnt_spd;
  prev_bck_spd = iDUT.bck_spd;
  prev_rgt_spd = iDUT.rght_spd;
  prev_lft_spd = iDUT.lft_spd;

  SendCmd(.comd(SET_THRST), .dat(16'h01ff));
  ChkVal16(.act(iDUT.ifly.thrst), .exp(16'h01ff), .name("Thrst"));
  ChkPosAck;

  repeat(10)@(posedge clk);
  ChkGtr(.act(iDUT.frnt_spd), .exp(prev_frnt_spd), .name("frnt_spd"));
  ChkGtr(.act(iDUT.rght_spd), .exp(prev_rgt_spd), .name("rght_spd"));
  ChkGtr(.act(iDUT.lft_spd), .exp(prev_lft_spd), .name("lft_spd"));
  ChkGtr(.act(iDUT.bck_spd), .exp(prev_bck_spd), .name("bck_spd"));

  // verify copter can land after takeoff
  SendCmd(.comd(EMER_LAND), .dat(NO_DATA));
  ChkVal16(.act(iDUT.ifly.d_ptch), .exp(16'd0), .name("DPtch"));
  ChkVal16(.act(iDUT.ifly.d_yaw), .exp(16'd0), .name("DYaw"));
  ChkVal16(.act(iDUT.ifly.d_roll), .exp(16'd0), .name("DRoll"));
  ChkVal16(.act(iDUT.ifly.thrst), .exp(16'd0), .name("Thrst"));
  ChkPosAck;

  $display("Success!");
  $stop();

end

always
  #10 clk = ~clk;

`include "tb_tasks.sv"	// maybe have a separate file with tasks to help with testing
endmodule
