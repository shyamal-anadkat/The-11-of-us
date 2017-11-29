module inert_intf_test (LED, SS_n, SCLK, MOSI, NEXT, RST_n, clk, INT, MISO);

	output logic SS_n, SCLK, MOSI;
	output logic [7:0] LED;
	input NEXT, INT, RST_n, clk, MISO;

	logic vld, strt_cal;
	logic [15:0] ptch, roll, yaw;
	logic [1:0] sel;

	typedef enum logic [1:0] {CAL, PTCH, ROLL, YAW} state_t;
	state_t state, nxt_state;

	reg rst_n_sync, rst_n_1;

	always_ff@(posedge clk) begin
	  rst_n_1 <= RST_n;
	  rst_n_sync <= rst_n_1;
	end

	inert_intf intf(.clk(clk), 
		.rst_n(rst_n_sync), 
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
always_ff@(posedge clk or negedge rst_n_sync) begin
  if (~rst_n_sync) begin
    state <= CAL;
  end else begin
    state <= nxt_state;
  end
end


assign LED = (sel == 2'b00) ? yaw[8:1] : 
(sel == 2'b01) ? roll[8:1] : (sel == 2'b10) ? ptch[8:1] : 7'h00;


always_comb begin
	nxt_state = CAL;
	strt_cal = 1'b0;
	sel = 2'b11;

	case (state)
		CAL: begin 
			if(cal_done) begin
				nxt_state = PTCH;
				sel = 2'b10;
			end else begin 
				nxt_state = CAL;
				sel = 2'b11;
				strt_cal = 1;
			end
		end

		PTCH: begin
			if(NEXT) begin
				nxt_state = ROLL;
				sel = 2'b01;
			end else begin
				nxt_state = PTCH;
				sel = 2'b10;
			end
		end
		ROLL: begin
			if(NEXT) begin
				nxt_state = YAW;
				sel = 2'b00;
			end else begin
				nxt_state = ROLL;
				sel = 2'b00;
			end
		end
		YAW:begin 
			if(NEXT) begin
				nxt_state = PTCH;
			end else begin
				nxt_state = YAW;
				sel = 2'b00;
			end
		end
		default: begin
			nxt_state = CAL;
		end
	endcase // state
end

endmodule