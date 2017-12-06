module QuadCopter(clk,RST_n,SS_n,SCLK,MOSI,MISO,INT,RX,TX,LED,FRNT,BCK,LFT,RGHT,
                  SS_A2D_n,SCLK_A2D,MOSI_A2D,MISO_A2D);

  input clk;			// 50MHz clock
  input RST_n;			// Reset from push button
  input RX;				// command from BLE interface
  input MISO;			// response from inertial sensor
  input INT;			// Interrupt pin from inertial
  input MISO_A2D;		// response from A2D SPI interface
  
  
  output SS_n;			// slave select to inertial sensor
  output SCLK;			// SCLK to inertial sensor
  output MOSI;			// MOSI to inertial sensor
  output TX;			// TX to BLE interface
  output [7:0] LED;		// goes to 8-LEDs on copter, map to whatever you wish. (active high)
  output FRNT;			// front motor PWM
  output BCK;			// back motor PWM
  output LFT;			// left motor PWM
  output RGHT;			// right motor PWM
  output SS_A2D_n;		// slave select to A2D
  output SCLK_A2D;		// SCLK of A2D SPI interface
  output MOSI_A2D;		// MOSI to A2D
  
  
  ///////////////////////////////////////
  // Interconnecting internal signals //
  /////////////////////////////////////
  wire cmd_rdy;					// command from wireless ready
  wire [7:0] cmd;				// 8-bit command from wireless
  wire [15:0] data;				// 16-bit data from wireless
  wire clr_cmd_rdy;				// clear the command from wireless
  wire [7:0] resp;				// response to wireless
  wire send_resp;				// asserted to send response to wireless
  wire resp_sent;				// indicates response to wireless has been sent
  
  wire vld;						// goes high one clock cycle when new inertial measurement
  wire signed [15:0] ptch;		// current pitch
  wire signed [15:0] roll;		// current roll
  wire signed [15:0] yaw;		// current yaw
  wire signed [15:0] d_ptch;	// desired pitch
  wire signed [15:0] d_roll;	// desired roll
  wire signed [15:0] d_yaw;		// desired yaw
  wire [11:0] batt_level;		// 12-bit version of battery level, only use upper 8-bits
  wire [8:0] thrst;				// desired thrust
  wire rst_n;					// internal synchronized global reset
  wire strt_cal;				// from cmd_cfg to inertial_intf
  wire inertial_cal;			// indicates calibration in progress to flght_control
  wire motors_off;				// to flight control, forces motors off
  wire cal_done;				// from inertial_intf to cmd_cfg
  wire strt_cnv;				// from cmd_cfg to A2D to initiate battery conversion
  wire cnv_cmplt;				// from A2D_intf to cmd_cfg to indicate conversion complete
  wire [10:0] frnt_spd;			// front motor speed from flght_cntrl
  wire [10:0] bck_spd;			// front motor speed from flght_cntrl
  wire [10:0] lft_spd;			// front motor speed from flght_cntrl
  wire [10:0] rght_spd;			// front motor speed from flght_cntrl  
  
  /////////////////////////////////////////////////////////////////////////
  // Instantiate UART_wrapper that receives commands from wireless link //
  /////////////////////////////////////////////////////////////////////// 
  //Signal interface to UART_wrapper was not rigidly specified, so you instantiate
  //your UART_wrapper here.
   UART_wrapper iWRAPPER(.clk(clk), .rst_n(rst_n), 
    .RX(RX), .TX(TX), 
    .cmd(cmd), .data(data), 
    .cmd_rdy(cmd_rdy), 
    .snd_resp(send_resp), 
    .resp_sent(resp_sent), 
    .resp(resp), 
    .clr_cmd_rdy(clr_cmd_rdy));
						
  ///////////////////////////////////////////////////////////////////////
  // Instantiate command config unit (interprets & executes commands) //
  /////////////////////////////////////////////////////////////////////	  
  cmd_cfg iCMD(.clk(clk), .rst_n(rst_n), .cmd_rdy(cmd_rdy), .cmd(cmd), .data(data),
               .clr_cmd_rdy(clr_cmd_rdy), .resp(resp), .send_resp(send_resp),
			   .d_ptch(d_ptch), .d_roll(d_roll), .d_yaw(d_yaw), .thrst(thrst),.batt(batt_level[11:4]),
			   .strt_cal(strt_cal),.inertial_cal(inertial_cal),.motors_off(motors_off),
			   .cal_done(cal_done),.strt_cnv(strt_cnv),.cnv_cmplt(cnv_cmplt));
				  
				  
  //////////////////////////////////////////////////////////
  // Instantiate interface to inertial sensor (ST iNEMO) //
  ////////////////////////////////////////////////////////
  inert_intf iNEMO(.clk(clk),.rst_n(rst_n),.ptch(ptch),.roll(roll),
                   .yaw(yaw),.strt_cal(strt_cal),.cal_done(cal_done),
					.vld(vld),.SS_n(SS_n),.SCLK(SCLK),
				     .MOSI(MOSI),.MISO(MISO),.INT(INT));

  //////////////////////////////////////////////
  // Instantiate flight controller math unit //
  ////////////////////////////////////////////		 
  flght_cntrl ifly(.clk(clk),.rst_n(rst_n),.vld(vld),.d_ptch(d_ptch),.d_roll(d_roll),
                   .d_yaw(d_yaw),.ptch(ptch),.roll(roll),.yaw(yaw),.thrst(thrst),
                   .inertial_cal(inertial_cal),.frnt_spd(frnt_spd),.bck_spd(bck_spd),
				   .lft_spd(lft_spd),.rght_spd(rght_spd));

  ///////////////////////
  // Instantiate ESCs //
  /////////////////////					 
  ESCs iESC (.clk(clk),.rst_n(rst_n),.frnt_spd(frnt_spd),.bck_spd(bck_spd),.lft_spd(lft_spd),
             .rght_spd(rght_spd),.motors_off(motors_off),.frnt(FRNT),.bck(BCK),.lft(LFT),.rght(RGHT));
			 
  //Hey...what is this unit?  Remember back in Exercise08 and Exercise09 you created a ESC_interface,
  //and we tested it.  This block is fairly simple and instantiates 4 copies of ESC_interface along with some
  //other logic.  Look at the project spec for more details.
			 
  ////////////////////////////////////////////////////////////
  // Instantiate A2D Interface for reading battery voltage //
  //////////////////////////////////////////////////////////
  A2D_intf iA2D(.clk(clk),.rst_n(rst_n),.strt_cnv(strt_cnv),.cnv_cmplt(cnv_cmplt),.chnnl(3'b000),
                .res(batt_level),.SS_n(SS_A2D_n),.SCLK(SCLK_A2D),.MOSI(MOSI_A2D),.MISO(MISO_A2D));

  
  /////////////////////////////////////
  // Instantiate reset synchronizer //
  ///////////////////////////////////  
  reset_synch iRST(.clk(clk),.RST_n(RST_n),.rst_n(rst_n));
  
endmodule
