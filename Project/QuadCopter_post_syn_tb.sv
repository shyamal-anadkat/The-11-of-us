`timescale 1ns/1ps
module QuadCopter_post_syn_tb();
			
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

  ///////////////////////////////////////////////////////
  // Test battery read command and response            //
  ///////////////////////////////////////////////////////
  SendCmd(.comd(REQ_BATT), .dat(NO_DATA));
  ChkResp(8'hC0);
  SendCmd(.comd(REQ_BATT), .dat(NO_DATA));
  ChkResp(8'hBF);
  

  $display("Success! Post syn test passed");
  $stop();

end

task Initialize();
clk = 1'b0;
RST_n = 1'b0;
cmd_to_copter = 8'h00;
data = 16'h0000;
send_cmd = 1'b0;
clr_resp_rdy = 1'b0;
@(posedge clk);
@(negedge clk);
RST_n = 1'b1;
@(posedge clk);
endtask

// Send command task
task SendCmd(input [7:0] comd, input [15:0] dat);
data = dat;
cmd_to_copter = comd;
send_cmd = 1'b1;
@(posedge clk);
send_cmd = 1'b0;
@(posedge cmd_sent);
endtask

// Check response command
task ChkResp(input [7:0] expected);
@(posedge resp_rdy);
if (resp != expected) begin
	$display("inertial interface command is incorrect. Was 0x%x should be 0x%x", resp, expected);
	$stop();
end
ClrRdy;
endtask

// Knock down rdy signal
task ClrRdy;
clr_resp_rdy = 1'b1;
@(posedge clk);
clr_resp_rdy = 1'b0;
endtask

always
  #10 clk = ~clk;

endmodule