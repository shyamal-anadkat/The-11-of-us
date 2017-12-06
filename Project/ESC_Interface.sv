module ESC_interface(SPEED, OFF, PWM, clk, rst_n);

// 50MHz clock, active low asynch rst
input clk, rst_n;

// Result from flight controller telling how fast to run each
//  motor. 
input [10:0] SPEED;

// Unsigned offset added to SPEED to compensate for variation
//  in ESC/motor pairs. Give all 4 motors the same pulse width
//  and they dont run close to the same speed. This offset is
//  used to compensate that to first order.
input [9:0] OFF;

// Output to the ESC to control motor speed. It is effectively a
//  PWM signal.
output reg PWM;

// If both SPEED and OFF were zero the PWM width would be 1ms
// For every count of SPEED or OFF the speed would increase by 0.32usec
//  we can see a 20-bit counter will be part of this module
//  .32usec for 50MHz clk => 16 clk cycles
//  1 ms => 50,000 clk cycles
parameter PERIOD_WIDTH = 20;
reg [PERIOD_WIDTH-1:0] counter;

wire [11:0] comp_speed;
assign comp_speed = SPEED + OFF;

wire [15:0] promote_comp_speed;
// left shift comp_speed by four
assign promote_comp_speed = {comp_speed[11:0], 4'b0000};

wire [16:0] setting;
assign setting = promote_comp_speed + 16'd50000;

// FF to hold count value and increment on each clock
always_ff@(posedge clk, negedge rst_n) begin
  if (~rst_n)
    counter <= 0;
  else
    counter <= counter + 20'd1;
end

// Used to determine if we should reset PWM flop
wire comparator;
assign comparator = (counter[16:0] >= setting[16:0]);

wire counter_full;
assign counter_full = &counter;

// counter_full asserted -> Set
// comparator asserted -> Rst
always_ff@(posedge clk, negedge rst_n) begin
	if (~rst_n)
		PWM <= 1'b0;
	else if (counter_full)
		PWM <= 1'b1;
	else if (comparator)
		PWM <= 1'b0;
	// PWM maintains value if not explicity stated
end

endmodule
