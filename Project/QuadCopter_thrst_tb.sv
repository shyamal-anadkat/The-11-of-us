module QuadCopter_thrst_tb();
			
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

  Initialize();

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

  // does decreasing thrst make motors slower?
  prev_frnt_spd = iDUT.frnt_spd;
  prev_bck_spd = iDUT.bck_spd;
  prev_rgt_spd = iDUT.rght_spd;
  prev_lft_spd = iDUT.lft_spd;

  SendCmd(.comd(SET_THRST), .dat(16'h0100));
  ChkVal16(.act(iDUT.ifly.thrst), .exp(16'h0100), .name("Thrst"));
  ChkPosAck;

  repeat(10)@(posedge clk);
  ChkLess(.act(iDUT.frnt_spd), .exp(prev_frnt_spd), .name("frnt_spd"));
  ChkLess(.act(iDUT.rght_spd), .exp(prev_rgt_spd), .name("rght_spd"));
  ChkLess(.act(iDUT.lft_spd), .exp(prev_lft_spd), .name("lft_spd"));
  ChkLess(.act(iDUT.bck_spd), .exp(prev_bck_spd), .name("bck_spd"));

  // does setting thrst to zero cause speeds to be zero?
  SendCmd(.comd(SET_THRST), .dat(16'h0000));
  ChkVal16(.act(iDUT.ifly.thrst), .exp(16'h0000), .name("Thrst"));
  ChkPosAck;

  repeat(10)@(posedge clk);
  ChkVal16(.act(iDUT.frnt_spd), .exp(16'd0), .name("frnt_spd"));
  ChkVal16(.act(iDUT.rght_spd), .exp(16'd0), .name("rght_spd"));
  ChkVal16(.act(iDUT.lft_spd), .exp(16'd0), .name("lft_spd"));
  ChkVal16(.act(iDUT.bck_spd), .exp(16'd0), .name("bck_spd"));

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