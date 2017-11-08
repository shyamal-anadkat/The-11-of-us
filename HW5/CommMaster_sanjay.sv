module CommMaster (resp, resp_rdy, frm_snt, TX, data, cmd, clk, rst_n, snd_cmd, RX);

	output [7:0] resp;
	output resp_rdy, TX, frm_snt;

	input [15:0] data;
	input [7:0] cmd;
	input clk, rst_n, snd_cmd, RX;

	logic [7:0] data_15_8, data_7_0;
	wire [7:0] tx_data;

	logic [1:0] sel;
	logic trmt, set_cmplt, clr_cmplt;

	wire tx_done;

	typedef enum reg [1:0] { IDLE, WAITH, WAITM, WAITL } state_t;
	state_t state, next;

	// Upper data byte
	always_ff @(posedge clk)
		if (snd_cmd)
			data_15_8 <= data[15:8];
		
	// Lower data byte
	always_ff @(posedge clk)
		if (snd_cmd)
			data_7_0 <= data[7:0];

	// Select tx_data		
	assign tx_data = (sel == 2'b10) ? cmd :
					 (sel == 2'b01) ? data_15_8 : data_7_0;
		
	// Instantiate UART
	UART uart (.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .rx_rdy(resp_rdy), .clr_rx_rdy(),
			   .rx_data(resp), .trmt(trmt), .tx_data(tx_data), .tx_done(tx_done));
					 
	// SM implementation
	always_ff @(posedge clk, negedge rst_n)
		if (!rst_n)
			state <= IDLE;
		else
			state <= next;

	always_comb begin
		next = IDLE;
		sel = 2'b10;
		trmt = 1'b0;
		set_cmplt = 1'b1;
		clr_cmplt = 1'b0;
		
		case(state)
			IDLE: begin
				if (snd_cmd) begin
					next = WAITH;
					trmt = 1'b1;
				end
				else
					next = IDLE;
			end
			
			WAITH: begin
				if (tx_done) begin
					trmt = 1'b1;
					sel = 2'b01;
					next = WAITM;
				end
				else
					next = WAITH;
				clr_cmplt = 1'b1;
				set_cmplt = 1'b0;
			end
			
			WAITM: begin
				if (tx_done) begin
					trmt = 1'b1;
					sel = 2'b00;
					next = WAITL;
				end
				else
					next = WAITM;
				sel = 2'b01;
				clr_cmplt = 1'b1;
				set_cmplt = 1'b0;
			end
			
			WAITL: begin
				if (tx_done) begin
					clr_cmplt = 1'b0;
					set_cmplt = 1'b1;
					next = IDLE;
				end
				else
					next = WAITL;
				sel = 2'b00;
				clr_cmplt = 1'b1;
				set_cmplt = 1'b0;
			end
		endcase
	end

	// frm_snt output
	assign frm_snt = (set_cmplt) ? 1'b1 :
					 (clr_cmplt) ? 1'b0 : 1'bx;

endmodule
