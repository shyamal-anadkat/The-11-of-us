module inert_intf_test (sel, stat, strt_cal, next, cal_done, clk, rst_n);

	output logic [1:0] sel;
	output logic stat, strt_cal;

	input clk, rst_n, next, cal_done;

	logic SS_n, SCLK, MISO, MOSI, vld, INT;
	logic [15:0] ptch, roll, yaw;

	typedef enum logic [1:0] {CAL, PTCH, ROLL, YAW} state_t;
	state_t state, nxt_state;

	inert_intf intf(.clk(clk), 
		.rst_n(rst_n), 
		.strt_cal(strt_cal), 
		.SS_n(SS_n), 
		.SCLK(SCLK), 
		.MOSI(MOSI), 
		.MISO(MISO), 
		.INT(INT),
		.vld(vld), 
		.cal_done(cal_done), 
		.ptch(ptch), 
		.roll(roll), 
		.yaw(yaw));

//// SM implementation ////
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		state <= CAL;
	end else begin
		state <= nxt_state;
	end
end

always_comb begin
	nxt_state = CAL;
	stat = 1'b0;
	strt_cal = 1'b0;
	sel = 2'b00;

	case (state)
		CAL: begin 
			if(cal_done) begin
				nxt_state = PTCH;
				sel = ptch;
			end else begin 
				nxt_state = CAL;
				stat = 1;
				strt_cal = 1;
			end
		end

		PTCH: begin
			if(next) begin
				nxt_state = ROLL;
				sel = roll;
			end else begin
				nxt_state = PTCH;
				sel = ptch;
			end
		end
		ROLL: begin
			if(next) begin
				nxt_state = YAW;
				sel = yaw;
			end else begin
				nxt_state = ROLL;
				sel = roll;
			end
		end
		YAW:begin 
			if(next) begin
				nxt_state = PTCH;
			end else begin
				nxt_state = YAW;
				sel = yaw;
			end
		end
		default: begin
			nxt_state = CAL;
		end
	endcase // state
end

endmodule