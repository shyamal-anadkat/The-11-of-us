module ESCs(clk, rst_n, frnt_spd, bck_spd, lft_spd, rght_spd, motors_off, frnt, bck, lft, rght);
input [10:0] frnt_spd, bck_spd, lft_spd, rght_spd;
input clk, rst_n, motors_off;
output frnt, bck, lft, rght;

localparam FRNT_OFF = 10'h220;
localparam BCK_OFF = 10'h220;
localparam LFT_OFF = 10'h220;
localparam RGHT_OFF = 10'h220;

parameter WIDTH = 20;
wire [10:0] motors_off_ext = {11{motors_off}};
wire [10:0] frnt_SPEED, bck_SPEED, lft_SPEED, rght_SPEED;
wire [9:0] frnt_OFF, bck_OFF, lft_OFF, rght_OFF;

// disable speeds and offsets if motors off is asserted
//  motors_off == 1 => zero
//  motors_off == 0 => original values
assign frnt_SPEED = frnt_spd &~motors_off_ext;
assign bck_SPEED = bck_spd &~motors_off_ext;
assign lft_SPEED = lft_spd &~motors_off_ext;
assign rght_SPEED = rght_spd &~motors_off_ext;
assign frnt_OFF = FRNT_OFF &~motors_off_ext;
assign bck_OFF = BCK_OFF &~motors_off_ext;
assign lft_OFF = LFT_OFF &~motors_off_ext;
assign rght_OFF = RGHT_OFF &~motors_off_ext;

ESC_interface #(WIDTH) esc_frnt(.SPEED(frnt_SPEED), .OFF(frnt_OFF), .PWM(frnt), .clk(clk), .rst_n(rst_n));
ESC_interface #(WIDTH) esc_bck (.SPEED(bck_SPEED),  .OFF(bck_OFF),  .PWM(bck),  .clk(clk), .rst_n(rst_n));
ESC_interface #(WIDTH) esc_lft (.SPEED(lft_SPEED),  .OFF(lft_OFF),  .PWM(lft),  .clk(clk), .rst_n(rst_n));
ESC_interface #(WIDTH) esc_rght(.SPEED(rght_SPEED), .OFF(rght_OFF), .PWM(rght), .clk(clk), .rst_n(rst_n));
endmodule
