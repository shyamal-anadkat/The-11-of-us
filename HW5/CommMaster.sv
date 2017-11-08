module CommMaster 
	(resp, resp_rdy, frm_snt, TX, RX, cmd, snd_cmd, data, clk, rst_n);

///////////////////////////////////////////////
// Module Interface Input and Outputs /////////
///////////////////////////////////////////////

//// outputs ////
output logic TX;

//// outputs for testbench ////
output logic [7:0] resp;
output logic resp_rdy;
output logic frm_snt;

//// inputs ////
input clk, rst_n;
input RX;
input [7:0] cmd;
input snd_cmd;
input [15:0] data;


///////////////////////////////////////////////
/// internal signals                        ///
///////////////////////////////////////////////

logic [1:0] sel;
logic trmt;
logic tx_done;
logic set_cmplt, clr_cmplt;

logic [7:0] tx_data; //input to UART
logic [7:0] low_bits, mid_bits;


//// Define state as enumerated type /////
typedef enum reg [1:0] {IDLE, WAITH, WAITM, WAITL} state_t;
state_t state, nxt_state;


//// instantiate transceiver ////
UART uartmod
(.clk(clk),
	.rst_n(rst_n),
	.RX(RX),.TX(TX),
	.rx_rdy(resp_rdy),
	.clr_rx_rdy(),
	.rx_data(resp),
	.trmt(trmt),
	.tx_data(tx_data),
	.tx_done(tx_done));


//// flops for high and mid bits ////
always_ff @(posedge clk) begin
	if(snd_cmd) begin
		low_bits <= data[7:0];
	end
end

always_ff @(posedge clk) begin
	if(snd_cmd) begin
		mid_bits <= data[15:8];
	end
end

//// frm_snt flop ////
always_ff @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		frm_snt <= 1'b0;
	end else if (clr_cmplt) begin
		frm_snt <= 1'b0;
	end else if(set_cmplt) begin
		frm_snt <= 1'b1;
	end
end

//// tx_data select logic /////
assign tx_data = (sel ==  2'b01) ? (mid_bits) : 
(sel == 2'b10) ? (cmd[7:0]) : (low_bits); 

////////////////////////////
// Infer state flop next //
//////////////////////////
always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		state <= IDLE;
	end else begin
		state <= nxt_state;
	end 
end


//// State Machine Implementation ////
always_comb begin 

	//// Default Outputs ////
	sel = 2'b10;
	trmt = 0;
	set_cmplt = 0;
	clr_cmplt = 0;
	nxt_state = IDLE;

	case (state)
		
		IDLE: begin 
			if(snd_cmd) begin
				trmt = 1;
				clr_cmplt = 1;
				sel = 2'b10;
				nxt_state = WAITH;
			end else begin
				nxt_state = IDLE;
				set_cmplt = 1;
			end
		end

		WAITH: begin
			if(tx_done) begin 
				sel = 2'b01;
				trmt = 1;
				nxt_state = WAITM;
			end else begin 
				nxt_state = WAITH;
				sel = 2'b10;
				clr_cmplt = 1;
			end
		end

		WAITM: begin
			if(tx_done) begin
				sel = 2'b00;
				trmt = 1;
				nxt_state = WAITL;
			end else begin
				sel = 2'b01;
				clr_cmplt = 1;
				nxt_state = WAITM;
			end

		end

	default : begin //WAITL STATE
		if(tx_done) begin
			set_cmplt = 1;
			nxt_state = IDLE;
		end else begin
			nxt_state = WAITL;
			clr_cmplt = 1;
			sel = 2'b00;
		end
	end
endcase
end


endmodule