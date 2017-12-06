/************************************/
/* Main module for flight control
/************************************/
module flght_cntrl(clk,rst_n,vld,inertial_cal,d_ptch,d_roll,d_yaw,ptch,
		roll,yaw,thrst,frnt_spd,bck_spd,lft_spd,rght_spd);
				
parameter D_QUEUE_DEPTH = 14;	// delay for derivative term
				
input clk,rst_n;
input vld;			// tells when a new valid inertial reading ready
				// only update D_QUEUE on vld readings
input inertial_cal;				// need to run motors at CAL_SPEED during inertial calibration
input signed [15:0] d_ptch,d_roll,d_yaw;	// desired pitch roll and yaw (from cmd_cfg)
input signed [15:0] ptch,roll,yaw;		// actual pitch roll and yaw (from inertial interface)
input [8:0] thrst;			// thrust level from slider
output [10:0] frnt_spd;			// 11-bit unsigned speed at which to run front motor
output [10:0] bck_spd;			// 11-bit unsigned speed at which to back front motor
output [10:0] lft_spd;			// 11-bit unsigned speed at which to left front motor
output [10:0] rght_spd;			// 11-bit unsigned speed at which to right front motor

///////////////////////////////////////////////////
// Need integer for loop used to create D_QUEUE //
/////////////////////////////////////////////////
integer x;
//////////////////////////////
// Define needed registers //
////////////////////////////								
reg signed [9:0] prev_ptch_err[0:D_QUEUE_DEPTH-1];
reg signed [9:0] prev_roll_err[0:D_QUEUE_DEPTH-1];
reg signed [9:0] prev_yaw_err[0:D_QUEUE_DEPTH-1];	// need previous error terms for D of PD

///////////////////////////////////////////////////////////////
// some Parameters to keep things more generic and flexible //
/////////////////////////////////////////////////////////////
  
localparam CAL_SPEED = 11'h1B0;		// speed to run motors at during inertial calibration
localparam MIN_RUN_SPEED = 13'h200;	// minimum speed while running  
localparam D_COEFF = 6'b00111;		// D coefficient in PID control = +7

//////////////////////////////////////////////////////
//                                                 //
// Perform calculations on pitch, roll, and yaw   //
//                                               //
//////////////////////////////////////////////////

//////////////////////////
// Pitch calculations  //
////////////////////////
wire signed [16:0] ptch_err;
wire signed [9:0] ptch_err_sat;
wire signed [9:0] ptch_pterm;
wire signed [9:0] ptch_D_diff;
wire signed [5:0] ptch_D_diff_sat;
wire signed [11:0] ptch_dterm;

// Get error term
assign ptch_err = ptch - d_ptch;
// subtract oldest queue entry from current saturated error
assign ptch_D_diff = ptch_err_sat - prev_ptch_err[D_QUEUE_DEPTH-1];

// saturate *_D_diff and *_err, get 5/8 of saturated error term
saturate sat_ptch(.signed_err(ptch_err), .signed_err_sat(ptch_err_sat), 
             .signed_D_diff(ptch_D_diff), .signed_D_diff_sat(ptch_D_diff_sat), 
             .signed_pterm(ptch_pterm));
// Get dterm
assign ptch_dterm = ptch_D_diff_sat * $signed(D_COEFF);

///////////////////////
// Roll calculations //
///////////////////////
wire signed [16:0] roll_err;
wire signed [9:0] roll_err_sat;
wire signed [9:0] roll_pterm;
wire signed [9:0] roll_D_diff;
wire signed [5:0] roll_D_diff_sat;
wire signed [11:0] roll_dterm;

// Get error term
assign roll_err = roll - d_roll;
// subtract oldest queue entry from current saturated error
assign roll_D_diff = roll_err_sat - prev_roll_err[D_QUEUE_DEPTH-1];

// saturate *_D_diff and *_err, get 5/8 of saturated error term
saturate sat_roll(.signed_err(roll_err), .signed_err_sat(roll_err_sat), 
             .signed_D_diff(roll_D_diff), .signed_D_diff_sat(roll_D_diff_sat), 
             .signed_pterm(roll_pterm));
// Get dterm
assign roll_dterm = roll_D_diff_sat * $signed(D_COEFF);


///////////////////////
// Yaw calculations  //
///////////////////////
wire signed [16:0] yaw_err;
wire signed [9:0] yaw_err_sat;
wire signed [9:0] yaw_pterm;
wire signed [9:0] yaw_D_diff;
wire signed [5:0] yaw_D_diff_sat;
wire signed [11:0] yaw_dterm;

// Get error term
assign yaw_err = yaw - d_yaw;
// subtract oldest queue entry from current saturated error
assign yaw_D_diff = yaw_err_sat - prev_yaw_err[D_QUEUE_DEPTH-1];

// saturate *_D_diff and *_err, get 5/8 of saturated error term
saturate sat_yaw(.signed_err(yaw_err), .signed_err_sat(yaw_err_sat), 
             .signed_D_diff(yaw_D_diff), .signed_D_diff_sat(yaw_D_diff_sat), 
             .signed_pterm(yaw_pterm));
// Get dterm
assign yaw_dterm = yaw_D_diff_sat * $signed(D_COEFF);
  
  
/// Logic to control queue ///
always_ff@(posedge clk, negedge rst_n) begin
  if (~rst_n) begin
    // reset all entries in queues
    for (x = 0; x < D_QUEUE_DEPTH; x = x+1) begin
        prev_ptch_err[x] <= 10'h00;
        prev_roll_err[x] <= 10'h00;
        prev_yaw_err[x] <= 10'h00;
    end
  end
  else if (vld) begin
    // if vld, then add new item and shift positions
    for (x = D_QUEUE_DEPTH-1; x >= 1; x=x-1) begin
      // promote previous values
      prev_ptch_err[x] <= prev_ptch_err[x-1];
      prev_roll_err[x] <= prev_roll_err[x-1];
      prev_yaw_err[x]  <= prev_yaw_err[x-1];
    end
    prev_ptch_err[0] <= ptch_err_sat;
    prev_roll_err[0] <= roll_err_sat;
    prev_yaw_err[0]  <= yaw_err_sat;
  end
end

// Bring together and perform arithmetic ops
wire [12:0] frnt_calc, bck_calc, lft_calc, rght_calc;
assign frnt_calc = {4'h0,thrst} + MIN_RUN_SPEED - {{3{ptch_pterm[9]}}, ptch_pterm} - {ptch_dterm[11],ptch_dterm} - {{3{yaw_pterm[9]}},yaw_pterm} - {yaw_dterm[11],yaw_dterm};
assign bck_calc  = {4'h0,thrst} + MIN_RUN_SPEED + {{3{ptch_pterm[9]}}, ptch_pterm} + {ptch_dterm[11],ptch_dterm} - {{3{yaw_pterm[9]}},yaw_pterm} - {yaw_dterm[11],yaw_dterm};
assign lft_calc  = {4'h0,thrst} + MIN_RUN_SPEED - {{3{roll_pterm[9]}}, roll_pterm} - {roll_dterm[11],roll_dterm} + {{3{yaw_pterm[9]}},yaw_pterm} + {yaw_dterm[11],yaw_dterm};
assign rght_calc = {4'h0,thrst} + MIN_RUN_SPEED + {{3{roll_pterm[9]}}, roll_pterm} + {roll_dterm[11],roll_dterm} + {{3{yaw_pterm[9]}},yaw_pterm} + {yaw_dterm[11],yaw_dterm};

// saturate each *_calc
wire [10:0] frnt_sat;
wire [10:0] bck_sat;
wire [10:0] lft_sat;
wire [10:0] rght_sat;
saturate_calc sat_frnt_calc(.calc(frnt_calc), .sat_calc(frnt_sat));
saturate_calc sat_bck_calc(.calc(bck_calc), .sat_calc(bck_sat));
saturate_calc sat_lft_calc(.calc(lft_calc), .sat_calc(lft_sat));
saturate_calc sat_rght_calc(.calc(rght_calc), .sat_calc(rght_sat));

// send each *_sat calc through mux if we want to calibrate
assign frnt_spd = inertial_cal ? CAL_SPEED : frnt_sat;
assign bck_spd  = inertial_cal ? CAL_SPEED : bck_sat;
assign rght_spd = inertial_cal ? CAL_SPEED : rght_sat;
assign lft_spd  = inertial_cal ? CAL_SPEED : lft_sat;
  
endmodule 


// Note: I am including these modules in this file
//  because the homework says to only submit two files

/************************************/
/* Module that saturates the 12 bit
/* calculation value (frnt, bck, etc)
/************************************/
module saturate_calc(calc, sat_calc);
  input [12:0] calc;
  output [10:0] sat_calc;

  assign sat_calc = calc[12] ? 
                        // negative condition, so zero output
                        11'h00
                      :
                        // positive condition
                        // check if we should clamp
                        calc[11] ? 11'h7FF : calc[10:0];
endmodule


/************************************/
/* Module that saturates unsigned_err,
/* signed_err, and signed_D_diff
/************************************/
module saturate(signed_err, signed_err_sat, signed_D_diff, signed_D_diff_sat, signed_pterm);

input signed [16:0] signed_err;
input signed[9:0] signed_D_diff;

output signed [9:0] signed_pterm;
output signed [9:0] signed_err_sat;
output signed [5:0] signed_D_diff_sat;

// pterm is 5/8 of err_sat
assign signed_pterm = {signed_err_sat[9], signed_err_sat[9:1]} + {{3{signed_err_sat[9]}}, signed_err_sat[9:3]};

// check if neg
assign signed_err_sat = signed_err[16] ? 
				// this is neg check
				// if they are not all 1's, then return saturated value; else trim
				~&signed_err[15:9] ? 10'b1000000000 : signed_err[9:0]
			:
				 // this is pos check
				 // if any are 1's, return sat value; else trim
				 |signed_err[15:9] ? 10'b0111111111 : signed_err[8:0];
			
assign signed_D_diff_sat = signed_D_diff[9] ? 
				// this is neg check
				// if they are not all 1's, then return saturated value; else trim
				~&signed_D_diff[8:5] ? 6'b100000 : signed_D_diff[5:0]
			   :
				 // this is pos
				 // if any are 1's, return sat value; else trim
				 |signed_D_diff[8:5] ? 6'b011111 : signed_D_diff[5:0];	

endmodule
