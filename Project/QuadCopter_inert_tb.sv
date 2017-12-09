module QuadCopter_inert_tb();
			
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

initial begin  
  Initialize();
  
  SendCmd(.comd(CALIBRATE), .dat(NO_DATA));
  ChkPosAck;

  // pitch roll and yaw should start as zero
  ChkVal16(.act(iDUT.ifly.yaw), .exp(16'd0), .name("Yaw"));
  ChkVal16(.act(iDUT.ifly.roll), .exp(16'd0), .name("Roll"));
  ChkVal16(.act(iDUT.ifly.ptch), .exp(16'd0), .name("Ptch"));

  // set desired values
  SendCmd(.comd(SET_PTCH), .dat(16'h002a));
  ChkPosAck;
  SendCmd(.comd(SET_ROLL), .dat(16'h003a));
  ChkPosAck;
  SendCmd(.comd(SET_YAW), .dat($signed(16'hff1f))); // make yaw negative
  ChkPosAck;

  // ptch roll and yaw should converge to the values we want
  repeat(12)@(posedge frnt_ESC);
  // check if within 5%
  ChkPerc(.act(iDUT.ifly.ptch), .exp(iDUT.ifly.d_ptch), .perc(5),.name("Ptch"));
  ChkPerc(.act(iDUT.ifly.roll), .exp(iDUT.ifly.d_roll), .perc(5), .name("Roll"));
  ChkPerc(.act(iDUT.ifly.yaw), .exp(iDUT.ifly.d_yaw), .perc(5), .name("Yaw"));

  repeat(50)@(posedge frnt_ESC);
  // check if within 50%
  ChkPerc(.act(iDUT.ifly.ptch), .exp(iDUT.ifly.d_ptch), .perc(50),.name("Ptch"));
  ChkPerc(.act(iDUT.ifly.roll), .exp(iDUT.ifly.d_roll), .perc(50), .name("Roll"));
  // yaw moves slower than the other two since roll and ptch are added to sensior fused value
  ChkPerc(.act(iDUT.ifly.yaw), .exp(iDUT.ifly.d_yaw), .perc(30), .name("Yaw"));

  repeat(40)@(posedge frnt_ESC);
  // check if within 75%
  ChkPerc(.act(iDUT.ifly.ptch), .exp(iDUT.ifly.d_ptch), .perc(75),.name("Ptch"));
  ChkPerc(.act(iDUT.ifly.roll), .exp(iDUT.ifly.d_roll), .perc(75), .name("Roll"));
  ChkPerc(.act(iDUT.ifly.yaw), .exp(iDUT.ifly.d_yaw), .perc(55), .name("Yaw"));

  repeat(40)@(posedge frnt_ESC);
  // check if within 85%
  ChkPerc(.act(iDUT.ifly.ptch), .exp(iDUT.ifly.d_ptch), .perc(85),.name("Ptch"));
  ChkPerc(.act(iDUT.ifly.roll), .exp(iDUT.ifly.d_roll), .perc(85), .name("Roll"));
  ChkPerc(.act(iDUT.ifly.yaw), .exp(iDUT.ifly.d_yaw), .perc(85), .name("Yaw"));

  // try with new values
  SendCmd(.comd(SET_PTCH), .dat(16'h000f));
  ChkPosAck;
  SendCmd(.comd(SET_ROLL), .dat(16'h001f));
  ChkPosAck;
  SendCmd(.comd(SET_YAW), .dat(16'hfffe6));
  ChkPosAck;

  // ptch roll and yaw should converge to the values we want
  repeat(100)@(posedge frnt_ESC);
  // check if within 85%
  ChkPerc(.act(iDUT.ifly.ptch), .exp(iDUT.ifly.d_ptch), .perc(85),.name("Ptch"));
  ChkPerc(.act(iDUT.ifly.roll), .exp(iDUT.ifly.d_roll), .perc(85), .name("Roll"));
  ChkPerc(.act(iDUT.ifly.yaw), .exp(iDUT.ifly.d_yaw), .perc(45), .name("Yaw"));

  $display("Success! Pitch, roll, and yaw are converging");
  $stop();
end

always
  #10 clk = ~clk;

`include "tb_tasks.sv" // test helper tasks
endmodule
