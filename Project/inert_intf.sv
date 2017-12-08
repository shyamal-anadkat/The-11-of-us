// Eric Heinz, Shyamal Anadkat, Sanjay Rajmohan

module inert_intf (ptch, roll, yaw, cal_done, vld, MOSI, SCLK, SS_n, clk, rst_n, MISO, INT, strt_cal);

input clk, rst_n, MISO, INT, strt_cal;

output [15:0] ptch, roll, yaw;
output cal_done, SS_n, SCLK, MOSI;
output logic vld;

// After reset de-asserts the system must write to some registers to
// configure the inertial sensor to operate in the mode we wish. The table
// below specifies the writes to perform.
localparam WR_EN_INT = 16'h0D02;
localparam WR_SETUP_ACCL = 16'h1062;
localparam WR_SETUP_GYRO = 16'h1162;
localparam WR_ROUNDING_ON = 16'h1460;

/// specifies the addresses you need to use to read inertial ///
// data. Recall for a read the lower byte of the 16-bit packet is a don’t care
localparam RD_PTCH_L = 16'hA200;
localparam RD_PTCH_H = 16'hA300;
localparam RD_ROLL_L = 16'hA400;
localparam RD_ROLL_H = 16'hA500;
localparam RD_YAW_L = 16'hA600;
localparam RD_YAW_H = 16'hA700;
localparam RD_AXL = 16'hA800;
localparam RD_AXH = 16'hA900;
localparam RD_AYL = 16'hAA00;
localparam RD_AYH = 16'hAB00;

logic [15:0] cmd, ptch_rt, roll_rt, yaw_rt, ax, ay, timer;
logic [7:0] ptch_h, ptch_l, roll_h, roll_l, yaw_h, yaw_l, ax_h, ax_l, ay_h, ay_l;
logic wrt, INT_ff1, INT_ff2;

// Enable registers
logic C_P_H, C_P_L, C_R_H, C_R_L, C_Y_H, C_Y_L, C_AX_H, C_AX_L, C_AY_H, C_AY_L;

wire [15:0] rd_data;
wire done;

typedef enum logic [3:0] { INIT1, INIT2, INIT3, INIT4, WAIT_FOR_INT, READ_PTCH_L, READ_PTCH_H, READ_ROLL_L,
						   READ_ROLL_H, READ_YAW_L, READ_YAW_H, READ_AXL, READ_AXH,
						   READ_AYL, READ_AYH } state_t;
state_t state, next;

//// double flop INT ( metastability ) ////
always_ff @(posedge clk, negedge rst_n) begin 
	if (!rst_n) begin
		INT_ff1 <= 1'b0;
		INT_ff2 <= 1'b0;
	end
	else begin
		INT_ff1 <= INT;
		INT_ff2 <= INT_ff1;
	end
end

// 16 bit timer flop (might need to test this)
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		timer <= 16'h0000;
	else
		timer <= timer + 1;

// ptch_l holding register	
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		ptch_l <= 8'h00;
	else if (C_P_L)
		ptch_l <= rd_data[15:8];
		
// ptch_h holding register	
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		ptch_h <= 8'h00;
	else if (C_P_H)
		ptch_h <= rd_data[15:8];
		
// roll_l holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		roll_l <= 8'h00;
	else if (C_R_L)
		roll_l <= rd_data[15:8];
		
// roll_h holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		roll_h <= 8'h00;
	else if (C_R_H)
		roll_h <= rd_data[15:8];
		
// yaw_l holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		yaw_l <= 8'h00;
	else if (C_Y_L)
		yaw_l <= rd_data[15:8];
		
// yaw_h holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		yaw_h <= 8'h00;
	else if (C_Y_H)
		yaw_h <= rd_data[15:8];
		
// ax_l holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		ax_l <= 8'h00;
	else if (C_AX_L)
		ax_l <= rd_data[15:8];
		
// ax_h holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		ax_h <= 8'h00;
	else if (C_AX_H)
		ax_h <= rd_data[15:8];
		
// ay_l holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		ay_l <= 8'h00;
	else if (C_AY_L)
		ay_l <= rd_data[15:8];
		
// ay_h holding register
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		ay_h <= 8'h00;
	else if (C_AY_H)
		ay_h <= rd_data[15:8];
		
// Combine high and low bytes
assign ptch_rt = {ptch_h, ptch_l};
assign roll_rt = {roll_h, roll_l};
assign yaw_rt = {yaw_h, yaw_l};
assign ax = {ax_h, ax_l};
assign ay = {ay_h, ay_l};

// SPI_mstr16 instance
SPI_mstr16 spi (.SS_n(SS_n),
				.SCLK(SCLK),
				.MOSI(MOSI),
				.done(done),
				.rd_data(rd_data),
				.MISO(MISO),
				.wrt(wrt),
				.cmd(cmd),
				.clk(clk),
				.rst_n(rst_n));

// Inertial integrator instance
inertial_integrator #(3) integrator (.clk(clk),
								.rst_n(rst_n),
								.strt_cal(strt_cal),
								.cal_done(cal_done),
								.vld(vld),
								.ptch_rt(ptch_rt),
								.roll_rt(roll_rt),
								.yaw_rt(yaw_rt),
								.ax(ax),
								.ay(ay),
								.ptch(ptch),
								.roll(roll),
								.yaw(yaw));


// SM implementation
always_ff @(posedge clk, negedge rst_n)
	if (!rst_n)
		state <= INIT1;
	else
		state <= next;

always_comb begin
	next = INIT1;
	wrt = 1'b0;
	cmd = WR_EN_INT;
	C_P_H = 1'b0;
	C_P_L = 1'b0;
	C_R_H = 1'b0;
	C_R_L = 1'b0;
	C_Y_H = 1'b0;
	C_Y_L = 1'b0;
	C_AX_H = 1'b0;
	C_AX_L = 1'b0;
	C_AY_H = 1'b0;
	C_AY_L = 1'b0;
	vld = 1'b0;
	
	case (state)
		INIT1: begin
			if (&timer) begin
				wrt = 1'b1;
				next = INIT2;
			end
			else
				next = INIT1;
			cmd = WR_EN_INT;
		end
		
		INIT2: begin
			if (&timer) begin
				wrt = 1'b1;
				next = INIT3;
			end
			else
				next = INIT2;
			cmd = WR_SETUP_ACCL;
		end
		
		INIT3: begin
			if (&timer) begin
				wrt = 1'b1;
				next = INIT4;
			end
			else
				next = INIT3;
			cmd = WR_SETUP_GYRO;
		end
		
		INIT4: begin
			if (&timer) begin
				wrt = 1'b1;
				next = WAIT_FOR_INT;
			end
			else
				next = INIT4;
			cmd = WR_ROUNDING_ON;
		end
		
		WAIT_FOR_INT: begin
			if (INT_ff2) begin
				next = READ_PTCH_L;
				cmd = RD_PTCH_L;
				wrt = 1'b1;
			end
			else
				next = WAIT_FOR_INT;
		end
		
		READ_PTCH_L: begin
			if (done) begin
				next = READ_PTCH_H;
				C_P_L = 1'b1;
				cmd = RD_PTCH_H;
				wrt = 1'b1;
			end
			else begin
				next = READ_PTCH_L;
				cmd = RD_PTCH_L;
			end
		end
		
		READ_PTCH_H: begin
			if (done) begin
				next = READ_ROLL_L;
				C_P_H = 1'b1;
				cmd = RD_ROLL_L;
				wrt = 1'b1;
			end
			else begin
				next = READ_PTCH_H;
				cmd = RD_PTCH_H;
			end
		end
		
		READ_ROLL_L: begin
			if (done) begin
				next = READ_ROLL_H;
				C_R_L = 1'b1;
				cmd = RD_ROLL_H;
				wrt = 1'b1;
			end
			else begin
				next = READ_ROLL_L;
				cmd = RD_ROLL_L;
			end
		end
		
		READ_ROLL_H: begin
			if (done) begin
				next = READ_YAW_L;
				C_R_H = 1'b1;
				cmd = RD_YAW_L;
				wrt = 1'b1;
			end
			else begin
				next = READ_ROLL_H;
				cmd = RD_ROLL_H;
			end
		end
		
		READ_YAW_L: begin
			if (done) begin
				next = READ_YAW_H;
				C_Y_L = 1'b1;
				cmd = RD_YAW_H;
				wrt = 1'b1;
			end
			else begin
				next = READ_YAW_L;
				cmd = RD_YAW_L;
			end
		end
		
		READ_YAW_H: begin
			if (done) begin
				next = READ_AXL;
				C_Y_H = 1'b1;
				cmd = RD_AXL;
				wrt = 1'b1;
			end
			else begin
				next = READ_YAW_H;
				cmd = RD_YAW_H;
			end
		end
		
		READ_AXL: begin
			if (done) begin
				next = READ_AXH;
				C_AX_L = 1'b1;
				cmd = RD_AXH;
				wrt = 1'b1;
			end
			else begin
				next = READ_AXL;
				cmd = RD_AXL;
			end
		end
		
		READ_AXH: begin
			if (done) begin
				next = READ_AYL;
				C_AX_H = 1'b1;
				cmd = RD_AYL;
				wrt = 1'b1;
			end
			else begin
				next = READ_AXH;
				cmd = RD_AXH;
			end
		end
		
		READ_AYL: begin
			if (done) begin
				next = READ_AYH;
				C_AY_L = 1'b1;
				cmd = RD_AYH;
				wrt = 1'b1;
			end
			else begin
				next = READ_AYL;
				cmd = RD_AYL;
			end
		end
		
		READ_AYH: begin
			if (done) begin
				next = WAIT_FOR_INT;
				C_AY_H = 1'b1;
				vld = 1'b1;
			end
			else begin
				next = READ_AYH;
				cmd = RD_AYH;
			end
		end
		
		default: begin
			next = INIT1;
		end
	endcase
end

endmodule
