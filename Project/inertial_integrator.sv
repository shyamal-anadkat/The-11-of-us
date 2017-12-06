module inertial_integrator(clk,rst_n,strt_cal,cal_done,vld,ptch_rt,roll_rt,yaw_rt,
                           ax,ay,ptch,roll,yaw);
						   
parameter SMPL_CNT_WIDTH = 11;		// normally take 2048 samples; Pass in a 3 for shorter simulation times

input clk, rst_n;
input strt_cal;						// goes high to initiate calibration
input vld;							// goes high for 1 clock cycle when new data valid
input signed [15:0] ptch_rt, roll_rt, yaw_rt;		// raw gyro rate readings from inert_intf
input signed [15:0] ax,ay;							// raw accel readings from inert_intf
output logic cal_done;				// asserted when calibration is completed
output signed [15:0] ptch, roll, yaw;
  
  wire signed [15:0] ptch_comp,roll_comp,yaw_comp;	// offset compensated gyro rate readings
  wire signed [24:0] ptch_g_product, roll_g_product;// used in fusion calculations
  wire signed [15:0] ptch_g, roll_g;				// pitch and roll exclusively from accel (used in fusion)
  wire signed [26:0] fusion_ptch;					// fusion term added or subtracted for leaking to ptch_g
  wire signed [26:0] fusion_roll;					// fusion term added or subtracted for leaking to ptch_g
  
  /////////////////
  // SM outputs //
  ///////////////
  logic clr_integrators;
  logic clr_smpl_cntr;
  logic en_smpl_cntr;
  logic compensate_offset;
 
  ////////////////////
  // Define States //
  //////////////////
  typedef enum reg[1:0] {IDLE,CALIBRATING,RUNNING} state_t;
  state_t state,nstate;
  
  //////////////////////////////
  // Define needed registers //
  ////////////////////////////
  reg signed [26:0] ptch_int,roll_int,yaw_int;		// angle integrators
  reg signed [15:0] ptch_off,roll_off,yaw_off;		// offset registers
  reg [SMPL_CNT_WIDTH-1:0] smpl_cntr;				// 2048 samples nominally, but can be shortened for sim
  reg signed [19:0] ax_accum, ay_accum;				// accumulators for ax,ay averaging
  reg signed [15:0] ax_avg, ay_avg;					// average of 16 ax and ay readings
  reg [3:0] avg_cntr;								// average 16 accel samples
  
  ////////////////////////
  // Infer State Flops //
  //////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	   state <= IDLE;
	 else
	   state <= nstate;
		
  //////////////////////////////////////
  // state transition & output logic //
  ////////////////////////////////////
  always_comb begin
    //////////////////////
	 // Default outputs //
	 ////////////////////
	 cal_done = 0;
	 clr_integrators = 0;
	 clr_smpl_cntr = 0;
	 en_smpl_cntr = 0;
	 compensate_offset = 0;
	 
	 case (state)
	   IDLE : begin
		  if (strt_cal) begin				// if start calibration we:
		    clr_integrators = 1;			// clear the integrators to
			clr_smpl_cntr = 1;				// average 2048 samples and
		    nstate = CALIBRATING;			// enter the calibrating state
		  end else
		    nstate = IDLE;
		end
		CALIBRATING : begin
		  en_smpl_cntr = vld;				// count valid samples averaged
		  if ((&smpl_cntr) && vld) begin	// have 2048 valid samples in offset calcs
		    cal_done = 1;
			 clr_integrators = 1;
			 nstate = RUNNING;
		  end else
			 nstate = CALIBRATING;
		end
		default : begin		// this is same as RUNNING
		  compensate_offset = 1;
		  if (strt_cal) begin				// only exit RUNNING if we get another strt_cal
			 clr_smpl_cntr = 1;
			 nstate = CALIBRATING;
		  end else
		    nstate = RUNNING;
		end
	 endcase
	 
  end
  
  /////////////////////////////////////////////////////////////////////////
  // During calibration just purly integrate sign extended raw readings //
  // During actual run, we compensate by subtracting offset from raw   //
  //////////////////////////////////////////////////////////////////////
  assign ptch_comp = (compensate_offset) ? ptch_rt-ptch_off : ptch_rt;
  assign roll_comp = (compensate_offset) ? roll_rt-roll_off : roll_rt;
  assign yaw_comp = (compensate_offset) ? yaw_rt-yaw_off : yaw_rt;
  
  ///////////////////////////////////////////////////
  // Integrate first 2048 samples to form average //
  // then capture them as offsets to be          //
  // subtracted from remaining samples          //
  ///////////////////////////////////////////////
  always @(posedge clk, negedge rst_n)
   if (!rst_n)
	  smpl_cntr <= 0;
	else if (clr_smpl_cntr)
	  smpl_cntr <= 0;
	else if (en_smpl_cntr)
	  smpl_cntr <= smpl_cntr + 1;
  
  always @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	  ptch_int <= 0;
	  roll_int <= 0;
	  yaw_int <= 0;
	end else if (clr_integrators) begin
	  ptch_int <= 0;
	  roll_int <= 0;
	  yaw_int <= 0;
	end else if (vld) begin
	  ///////////////////////////////////////////////////////////////////
	  // During calibration we use the integrators as accumulators to //
	  // get an average of 2048 samples of each axis, which is later //
	  // subtracted from each respective reading during RUN state   //
	  ///////////////////////////////////////////////////////////////
	  ptch_int <= ptch_int + {{11{ptch_comp[15]}},ptch_comp} + fusion_ptch;
	  roll_int <= roll_int + {{11{roll_comp[15]}},roll_comp} + fusion_roll;
	  yaw_int  <= yaw_int + {{11{yaw_comp[15]}},yaw_comp};
	end
	
  always @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	  ptch_off <= 16'h0000;
	  roll_off <= 16'h0000;
	  yaw_off <= 16'h0000;
	end else if (cal_done) begin
	  ///////////////////////////////////////////////////////////////
	  // Our calibrated offset is the average of 2048 samples, so //
	  // we arithmetically shift the integrators down 11-bits.   //
	  // Actually shift a parametized number of bits (short sim)//
	  ///////////////////////////////////////////////////////////
	  ptch_off <= ptch_int[SMPL_CNT_WIDTH+15:SMPL_CNT_WIDTH];
	  roll_off <= roll_int[SMPL_CNT_WIDTH+15:SMPL_CNT_WIDTH];
	  yaw_off <= yaw_int[SMPL_CNT_WIDTH+15:SMPL_CNT_WIDTH];
	end
	
	////////////////////////////////////////////////
	// Implement averaging of 16 ax & ay samples //
	//////////////////////////////////////////////	
	always @(posedge clk, negedge rst_n)
	  if (!rst_n) begin
	    ax_accum <= 20'h000000;
		ay_accum <= 20'h000000;
	  end else if ((vld) && (&avg_cntr)) begin
	    ax_accum <= 20'h000000;
		ay_accum <= 20'h000000;
	  end else if (vld) begin
	    ax_accum <= ax_accum + {{4{ax[15]}},ax};
		ay_accum <= ay_accum + {{4{ay[15]}},ay};	  
	  end
	  
	always @(posedge clk, negedge rst_n)
	  if (!rst_n)
	    avg_cntr <= 4'h0;
	  else if (vld)
	    avg_cntr <= avg_cntr + 1;
		 
	always @(posedge clk, negedge rst_n)
	  if (!rst_n) begin 
	    ax_avg <= 16'h0000;
		ay_avg <= 16'h0000;
	  end else if ((vld) && (&avg_cntr)) begin
	    //// div by 16 to get average ////
	    ax_avg <= ax_accum[19:4];
		ay_avg <= ay_accum[19:4];
	  end	 
	    
	  
	//////////////////////
	// Divide by 2^13. //
	////////////////////////////////////////////////////
	// This is just a scaling factor.  Don't ask why //
	// this value was used.  A similar value was    //
	// used in the remote control glove.           //
	////////////////////////////////////////////////
	assign ptch = {{2{ptch_int[26]}},ptch_int[26:13]};
	assign roll = {{2{roll_int[26]}},roll_int[26:13]};
	assign yaw = {{2{yaw_int[26]}},yaw_int[26:13]};
	
	////////////////////////////////////////////////////////
	// Now calculate pitch and roll from G readings only //
	////////////////////////////////////////////////////////////////////////
	// Where did that 327 number come from?  Trial/error and observation //
	//////////////////////////////////////////////////////////////////////
	assign ptch_g_product = ay_avg*$signed(327);
	assign ptch_g = {{4{ptch_g_product[24]}},ptch_g_product[24:13]};
	assign roll_g_product = -ax_avg*$signed(327);
	assign roll_g = {{4{roll_g_product[24]}},roll_g_product[24:13]};
	
    /////////////////////////////////////////////////////////////////////
	// During calibration this fusion offset is zero.  During running //
	// it will "leak" the integrator + if ptch_g>ptch, and leak if   //
	// negative if ptch_g<ptch.  So "DC" reading approaches that of //
	// what is calculated by the accel alone.                      //
	////////////////////////////////////////////////////////////////
	assign fusion_ptch = (!compensate_offset) ? 30'h00000000 :
	                     (ptch_g>$signed(ptch)) ? 30'h00000800 : 30'hFFFFF000;
	assign fusion_roll = (!compensate_offset) ? 30'h00000000 :
	                     (roll_g>$signed(roll)) ? 30'h00000800 : 30'hFFFFF000;

endmodule