module CycloneIV(SS_n,SCLK,MISO,MOSI,INT,frnt_ESC,back_ESC,left_ESC,rght_ESC);
  /////////////////////////////////////////////////
  // Model of a ST iNEMO 6-axis inertial device //
  // and overall physics of quadcopter         //
  // This code is a terrible hack.  Remember  //
  // do as I say, not as I do.  I am the     //
  // instructor and I judge you.  Please    //
  // don't judge me.                       //
  //////////////////////////////////////////

  input SS_n;				// active low slave select
  input SCLK;				// Serial clock
  input MOSI;				// serial data in from master
  input frnt_ESC;
  input back_ESC;
  input left_ESC;
  input rght_ESC;
  
  output MISO;				// serial data out to master
  output reg INT;			// interrupt output
  
  localparam FRNT_OFFSET = 15'h0000;	// these are 4X the difference from
  localparam BACK_OFFSET = 15'h0000;	// smallest offset defined in ESCs.v 
  localparam LEFT_OFFSET = 15'h0000;    // These used to have meaning when I was using shitty ESCs
  localparam RGHT_OFFSET = 15'h0000;    // Bought good ones, and now offset term non needed.

  localparam WEIGHT = 16'h8000;
  
  typedef enum reg[1:0] {IDLE,HALF1,HALF2} state_t;
  ///////////////////////////////////////////////
  // Registers needed in design declared next //
  /////////////////////////////////////////////
  state_t state,nstate;
  reg [15:0] shft_reg_tx;	// SPI shift register for transmitted data (falling edge)
  reg [15:0] shft_reg_rx;	// SPI shift register for received data (rising edge)
  reg [3:0] bit_cnt;		// Needed to know when to interpret R/Wn and address for tx_data
  reg write_reg;			// Used as sentinel to mark that command is write, so write is 
                            // completed at end of transaction
  reg POR_n;				// Power On Reset active low
  

  //////// Need array to hold all possible registers of iNEMO ////////
  reg [7:0]registers[0:127];
  reg internal_clk;				// 12.5MHz
  reg [14:0]update_period;		// around 400Hz
  reg clr_INT;
  
  reg signed [15:0] ax, ay, az;					// ESC inputs over time are used to compute
  reg signed [15:0] vert_v, ptch_v, roll_v, yaw_v;		// compute these sensor outputs.  _v terms
  reg signed [15:0] vert_p, ptch_p, roll_p, yaw_p;		// are integrated to get angular position
  
  reg [14:0] frnt_cnt,back_cnt,left_cnt,rght_cnt;	// used to measure ESC pulse widths
  reg [14:0] frnt,back,left,rght;					// captured offset corrected ESC speeds
  reg [16:0] thrst;					// summation of all ESC pulses compensated for offsets
  reg frnt_ff1,back_ff1,left_ff1,rght_ff1;			// used for edge detection of ESC pulses
  reg all_done_ff1, all_done_ff2;					// used to determine when all ESC have had falling edge
  reg airborne;
  
  /////////////////////////////////////////////
  // SM outputs declared as type logic next //
  ///////////////////////////////////////////
  logic ld_tx_reg, shft_tx, init;
  logic set_write,clr_write;
  logic [7:0] tx_data;
  
  wire NEMO_setup;		// once registers setup it will start the measurement cycle of inertial sensor
  wire frnt_rise,back_rise,left_rise,rght_rise;		// +edge detect on ESCs
  wire frnt_fall,back_fall,left_fall,rght_fall;		// -edge detection on ESCs
  wire all_fall;									// triggers when all ESC's have had neg edge
  wire signed [15:0] vert_a,ptch_a,roll_a,yaw_a;			// pitch roll and yaw acceleration rates
  
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  update_period <= 15'h0000;
	else if (NEMO_setup)
	  update_period <= update_period + 1;
	  
	  
  //// Infer main SPI shift register ////
  always_ff @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  shft_reg_tx <= 16'h0000;
	else if (init)
	  shft_reg_tx <= 16'h0000;
	else if (ld_tx_reg)						// occurs at beginning and middle of 16-bit transaction
	  shft_reg_tx <= {tx_data,8'h00};
	else if (shft_tx)
	  shft_reg_tx <= {shft_reg_tx[14:0],1'b0};

  //// Infer main SPI shift register ////
  always_ff @(posedge SCLK, negedge POR_n)
    if (!POR_n)
	  shft_reg_rx <= 16'h0000;
	else if (!SS_n)
	  shft_reg_rx <= {shft_reg_rx[14:0],MOSI};
	  
  always_ff @(negedge SCLK)
    if (init)
	  bit_cnt <= 4'b0000;
	else if (shft_tx)
	  bit_cnt <= bit_cnt + 1;
	  
  always_ff @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  write_reg <= 1'b0;
	else if (set_write)
	  write_reg <= 1'b1;
	else if (write_reg)		// can only be high for one SCLK period
	  write_reg <= 1'b0;
	 
  ///////////////////////////////////////////////////
  // At end of SPI transaction, if it was a write //
  // the register being written is updated       //
  ////////////////////////////////////////////////
  always_ff @(posedge SS_n)
    if (write_reg)
      registers[shft_reg_rx[14:8]] <= shft_reg_rx[7:0];
	
  //////////////////////////////////////////////////
  // model update_period for ODR of inert sensor //
  ////////////////////////////////////////////////
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  update_period <= 15'h0000;
	else if (NEMO_setup)
	  update_period <= update_period + 1;
	
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  INT <= 1'b0;
	else if (clr_INT)
	  INT <= 1'b0;
	else if (&update_period)
	  INT <= 1'b1;
	
  //// Infer state register next ////
  always @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  state <= IDLE;
	else
	  state <= nstate;

  ///////////////////////////////////////
  // Implement state transition logic //
  /////////////////////////////////////
  always_comb
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
	  ld_tx_reg = 0;
      shft_tx = 0;
      init = 0;
	  tx_data = 16'h0000;
	  set_write = 0;
      nstate = IDLE;	  

      case (state)
        IDLE : begin
          if (!SS_n) begin
		    init = 1;
            nstate = HALF1;
          end
        end
		HALF1 : begin
		  shft_tx = 1;
		  if (bit_cnt==4'b0111) begin
		    ld_tx_reg = 1;
			tx_data = response(shft_reg_rx[7:0]);		// response if function of first 8-bits received
		    nstate = HALF2;
	      end else
		    nstate = HALF1;
		end
		HALF2 : begin
		  shft_tx = 1;		
		  if (bit_cnt==4'b1110) begin
		    set_write = ~shft_reg_rx[14];				// if it is a write set the write sentinel
		    nstate = IDLE;
		  end else
		    nstate = HALF2;
		end
      endcase
    end
	

  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n) begin
	  ax = 16'h0000;
	  ay = 16'h0000;
	  az = 16'h3FFF;
	end else begin
	  ax = {{4{roll_p[15]}},roll_p[15:4]} + {{3{roll_p[15]}},roll_p[15:3]}-roll_p;		// -13/16 of roll_p
	  ay = ptch_p - {{3{ptch_p[15]}},ptch_p[15:3]} - {{4{ptch_p[15]}},ptch_p[15:4]};	// 13/16 of ptch_p
	  az = 16'h3FFF;
	end
	
  ///// MISO is shift_reg[15] with a tri-state ///////////
  assign MISO = (SS_n) ? 1'bz : shft_reg_tx[15];

  initial begin
    POR_n = 0;
	clr_INT = 0;
	internal_clk = 0;
	vert_v = 16'h0000;
	ptch_v = 16'h0000;
	roll_v = 16'h0000;
	yaw_v = 16'h0000;
	vert_p = 16'h0000;
	ptch_p = 16'h0000;
	roll_p = 16'h0000;
	yaw_p = 16'h0000;
	#10;
	POR_n = 1;
  end
  
  always
    #40 internal_clk = ~internal_clk;	// generate 12.5MHz internal clock
  
  function [7:0] response (input [7:0] in_byte);
    if (in_byte[7])	begin		// if it is a read respond with requested register
	  case (in_byte[6:0])
	    7'h22 : begin response = ptch_v[7:0]; clr_INT=1; end
		7'h23 : begin response = ptch_v[15:8]; clr_INT=0; end
	    7'h24 : response = roll_v[7:0];
		7'h25 : response = roll_v[15:8];
	    7'h26 : response = yaw_v[7:0];
		7'h27 : response = yaw_v[15:8];
	    7'h28 : response = ax[7:0];
		7'h29 : response = ax[15:8];
	    7'h2A : response = ay[7:0];
		7'h2B : response = ay[15:8];
	    7'h2C : response = az[7:0];
		7'h2D : response = az[15:8];
	    default : response = registers[in_byte[6:0]];	// case it is just a generic register
	  endcase
	end else					// it is a write
	  response = 8'hA5;			// respond with 0xA5
  endfunction
  
  assign NEMO_setup = ((registers[7'h0d]==8'h02) && (registers[7'h11]==8'h62)) ? 1'b1 : 1'b0;
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // Next section is more of a model of the physics of quadcopter as determined by ESC pulse widths //
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge internal_clk) begin
    frnt_ff1 <= frnt_ESC;
	back_ff1 <= back_ESC;
	left_ff1 <= left_ESC;
	rght_ff1 <= rght_ESC;
	all_done_ff1 <= frnt_ESC | back_ESC | left_ESC | rght_ESC;
	all_done_ff2 <= all_done_ff1;
  end
  
  assign frnt_rise = (frnt_ESC & ~frnt_ff1);
  assign back_rise = (back_ESC & ~back_ff1);
  assign left_rise = (left_ESC & ~left_ff1);
  assign rght_rise = (rght_ESC & ~rght_ff1);
  
  assign frnt_fall = (~frnt_ESC & frnt_ff1);
  assign back_fall = (~back_ESC & back_ff1);
  assign left_fall = (~left_ESC & left_ff1);
  assign rght_fall = (~rght_ESC & rght_ff1);
  
  assign all_fall = (~all_done_ff1 & all_done_ff2);
  
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  frnt_cnt <= 0;
	else if (frnt_rise)
	  frnt_cnt <= 0;
	else if (frnt_ESC)
	  frnt_cnt <= frnt_cnt + 1;
	  
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  back_cnt <= 0;
	else if (back_rise)
	  back_cnt <= 0;
	else if (back_ESC)
	  back_cnt <= back_cnt + 1;	  
	  
  ///////////////////////////////////////////////
  // Who wrote this code?  How come there are //
  // so few useful comments?  Shame on them  //
  ////////////////////////////////////////////
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  left_cnt <= 0;
	else if (left_rise)
	  left_cnt <= 0;
	else if (left_ESC)
	  left_cnt <= left_cnt + 1;

  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  rght_cnt <= 0;
	else if (rght_rise)
	  rght_cnt <= 0;
	else if (rght_ESC)
	  rght_cnt <= rght_cnt + 1;	  
	  
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  frnt <= 0;
	else if (frnt_fall)
	  frnt <= frnt_cnt - FRNT_OFFSET;
	 
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  back <= 0;
	else if (back_fall)
	  back <= back_cnt - BACK_OFFSET;

  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  left <= 0;
	else if (left_fall)
	  left <= left_cnt - LEFT_OFFSET;

  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  rght <= 0;
	else if (rght_fall)
	  rght <= rght_cnt - RGHT_OFFSET;

  assign thrst = frnt+back+left+rght;

  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
      airborne <= 1'b0;
    else if ((vert_p>16'h0001) && !airborne) begin
      airborne <= 1'b1;
      $display("Quadcopter is now Airborne!\n");
    end

  always_comb
    if (airborne && vert_p[15]) begin
      $display("Vertical position <0, Quadcopter crashed!");
      $stop();
    end
  
  assign vert_a = (airborne || (thrst[16:1]>WEIGHT)) ? thrst[16:1] - WEIGHT : 16'h0000;
  assign ptch_a = (airborne) ? frnt - back : 16'h0000;		// pitch angle acceleration
  assign roll_a = (airborne) ? left - rght : 16'h0000;		// roll angle acceleration
  assign yaw_a = (airborne) ? ((frnt+back) - (left+rght)) : 16'h0000;	// yaw acceleration

  ////////////////////////////////////////////////////////////////////////
  // Now integrate accel to get velocity, and velocity to get position //
  //////////////////////////////////////////////////////////////////////
  always @(negedge all_fall) begin
      vert_v = satSum16(vert_v,{{2{vert_a[15]}},vert_a[15:2]});
      ptch_v = satSum16(ptch_v,ptch_a);
      roll_v = satSum16(roll_v,roll_a);
      yaw_v = satSum16(yaw_v,yaw_a);
      vert_p = satSum16(vert_p,{{6{vert_v[15]}},vert_v[15:6]});
      ptch_p = satSum16(ptch_p,{{7{ptch_v[15]}},ptch_v[15:7]});
      roll_p = satSum16(roll_p,{{7{roll_v[15]}},roll_v[15:7]});
      yaw_p = satSum16(yaw_p,{{7{yaw_v[15]}},yaw_v[15:7]});	
  end
   
  function [15:0] satSum16 (input [15:0] in1,in2);
    reg [15:0] simp_sum;

    simp_sum = in1 + in2;
    if (in1[15] && in2[15] && !simp_sum[15])
	satSum16 = 16'h8000;
    else if (!in1[15] && !in2[15] && simp_sum[15])
	satSum16 = 16'h7FFF;
    else
	satSum16 = simp_sum;
  endfunction
  
endmodule  
  
