/*
* Eric Heinz, Shyamal Anadkat, Sanjay Rajmohan
* The cmd_cfg unit interprets the 24-bit commands 
* from UART_wrapper (sent via
* BLE from the remote control). 
*/


module cmd_cfg (d_ptch, d_roll, d_yaw, thrst, resp, send_resp, clr_cmd_rdy, strt_cal, inertial_cal,
	motors_off, strt_cnv, data, cmd, batt, cal_done, cnv_cmplt, cmd_rdy, clk, rst_n);

///////////////////////////////////////////////
// Module Interface Input and Outputs /////////
///////////////////////////////////////////////
output logic signed [15:0] d_ptch, d_roll, d_yaw;
output logic [8:0] thrst;
output logic [7:0] resp;
output logic strt_cnv, motors_off, clr_cmd_rdy, send_resp, strt_cal, inertial_cal;

input [15:0] data;
input [7:0] cmd, batt;
input cal_done, cnv_cmplt, cmd_rdy, clk, rst_n;


/// timer width parameter (will be changed to 26 for real quad) ///
parameter TIMER_WIDTH = 26;

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


///////////////////////////////////////////////
// Internal wires/regs declared next //////////
///////////////////////////////////////////////
logic [TIMER_WIDTH-1:0] mtr_ramp_tmr;
logic clr_tmr;
logic wrt_ptch, wrt_roll, wrt_yaw, wrt_thrst, emer_land;
logic en_mtrs, mtrs_off;

///////////////////////////////////////////////
// State Definitions                 //////////
///////////////////////////////////////////////
typedef enum logic [2:0] { IDLE, SET_BATT, POS_ACK, WAIT, CALIBRATE_QUAD } state_t;
state_t state, next;

//// motors_off flop logic ////
always_ff @(posedge clk, negedge rst_n)
if (!rst_n)
	motors_off <= 1'b1;
else if (mtrs_off)
	motors_off <= 1'b1;
else if (en_mtrs)
	motors_off <= 1'b0;

//// wait timer flop ////
always_ff @(posedge clk, negedge rst_n)
if (!rst_n)
	mtr_ramp_tmr <= 0;
else if (clr_tmr)
	mtr_ramp_tmr <= 0;
else
mtr_ramp_tmr <= mtr_ramp_tmr + 1;

//// ptch flop ////
always_ff @(posedge clk, negedge rst_n)
if (!rst_n)
	d_ptch <= 16'h0000;
else if (emer_land)
	d_ptch <= 16'h0000;
else if (wrt_ptch)
	d_ptch <= $signed(data);

//// roll flop ////
always_ff @(posedge clk, negedge rst_n)
if (!rst_n)
	d_roll <= 16'h0000;
else if (emer_land)
	d_roll <= 16'h0000;
else if (wrt_roll)
	d_roll <= $signed(data);

//// yaw flop ////
always_ff @(posedge clk, negedge rst_n)
if (!rst_n)
	d_yaw <= 16'h0000;
else if (emer_land)
	d_yaw <= 16'h0000;
else if (wrt_yaw)
	d_yaw <= $signed(data);

//// thrst flop ////
always_ff @(posedge clk, negedge rst_n)
if (!rst_n)
	thrst <= 9'h000;
else if (emer_land)
	thrst <= 9'h000;
else if (wrt_thrst)
	thrst <= data[8:0];

//// Infer state register next ////
always_ff @(posedge clk, negedge rst_n)
if (!rst_n)
	state <= IDLE;
else
state <= next;

  //////////////////////////////////////
  // Implement state tranisiton logic //
  /////////////////////////////////////
  always_comb begin

  	//////////////////////
    // Default outputs //
    ////////////////////
    next = IDLE;
	resp = 8'hA5;
    wrt_ptch = 1'b0;
    wrt_roll = 1'b0;
    wrt_yaw = 1'b0;
    wrt_thrst = 1'b0;
    emer_land = 1'b0;
    strt_cnv = 1'b0;
    clr_tmr = 1'b1;
    mtrs_off = 1'b0;
    en_mtrs = 1'b0;
    clr_cmd_rdy = 1'b0;
    send_resp = 1'b0;
    strt_cal = 1'b0;
    inertial_cal = 1'b0;

    case (state)
    	IDLE: begin
    		if (cmd_rdy) begin
    			case (cmd)
    				REQ_BATT: begin
                        // begin conversion then go to SET_BATT to wait for completion
    					strt_cnv = 1'b1;
    					next = SET_BATT;
    				end

    				SET_PTCH: begin
    					wrt_ptch = 1'b1;
    					next = POS_ACK;
    				end

    				SET_ROLL: begin
    					wrt_roll = 1'b1;
    					next = POS_ACK;
    				end

    				SET_YAW: begin
    					wrt_yaw = 1'b1;
    					next = POS_ACK;
    				end

    				SET_THRST: begin
    					wrt_thrst = 1'b1;
    					next = POS_ACK;
    				end

    				CALIBRATE: begin 
    					clr_cmd_rdy = 1'b1;
    					clr_tmr = 1'b0;
    					en_mtrs = 1'b1;
    					next = WAIT;
    				end
    				
    				EMER_LAND: begin
    					emer_land = 1'b1;
    					next = POS_ACK;
    				end
    				
    				MTRS_OFF: begin
    					mtrs_off = 1'b1;
    					next = POS_ACK;
    				end
    			endcase
    		end
    		else
    			next = IDLE;
    	end
    	
    	SET_BATT: begin
    		// wait for ADC response
    		if (cnv_cmplt) begin
    			resp = batt;
    			clr_cmd_rdy = 1'b1;
    			send_resp = 1'b1;
    			next = IDLE;
    		end
    		else
    			next = SET_BATT;
    	end
    	
        // state to send positive acknowledgement
    	POS_ACK: begin
    		resp = 8'hA5;
    		clr_cmd_rdy = 1'b1;
    		send_resp = 1'b1;
    		next = IDLE;
    	end
    	
    	WAIT: begin
    		clr_tmr = 1'b0;
    		en_mtrs = 1'b1;
            // Wait for timer to be saturated, then assert strt_cal for one cycle
    		if (&mtr_ramp_tmr) begin
    			next = CALIBRATE_QUAD;
    			strt_cal = 1'b1;
    		end else
    			next = WAIT;
    	end
    	
    	CALIBRATE_QUAD: begin
    		// keep intertial_cal asserted until we leave this state
    		inertial_cal = 1'b1;
    		if (cal_done)
    			next = POS_ACK;
    		else
    			next = CALIBRATE_QUAD;
    	end
    	
    	default: begin
    		next = IDLE;
    	end
    endcase
end

endmodule
