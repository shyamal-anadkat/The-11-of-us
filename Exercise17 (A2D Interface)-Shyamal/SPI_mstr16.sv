/* Author: Shyamal Anadkat <<->> SPI_mstr16 */

//////////////////////////////////////////////////|
// Model of a SPI Master 16 bits                 ||
//////////////////////////////////////////////////|
module SPI_mstr16 (SS_n, SCLK, MOSI, done, rd_data, MISO, wrt, cmd, clk, rst_n);


///////////////////////////////////////////////
// Module Interface Input and Outputs /////////
///////////////////////////////////////////////
output logic SS_n, SCLK, MOSI,done;
output logic [15:0] rd_data;

input clk, rst_n, MISO;
input wrt;				// high for 1 clk period wld init a SPI transaction
input [15:0] cmd;		      // cmd to send (ADC chnnl included)


///////////////////////////////////////////////
// Internal wires/regs declared next //////////
///////////////////////////////////////////////

logic [4:0] sclk_div; 	//cntr for posedge clk, SCLK will be 1/32 of the 50MHz clock
logic [4:0] bit_cntr; 	//bit-cntr
logic rst_cnt, shft,set_done, clr_done;
logic smpl, MISO_smpl;
logic MISO_ff1, MISO_ff2;

localparam smpl_bits = 5'b01111;
localparam shft_bits = 5'b11111;


///////////////////////////////////////////////
// State Definitions                 //////////
///////////////////////////////////////////////
typedef enum reg [1:0] {IDLE, FRONT_PORCH, BITS, BACK_PORCH} state_t;
state_t state, nxt_state;

//// double flop MISO ( I respect metastability ) ////
always_ff @(posedge clk, negedge rst_n) begin 
	if(!rst_n) begin
		MISO_ff1 <= 1'b0;
		MISO_ff2 <= 1'b0;
	end else begin
		MISO_ff1 <= MISO;
		MISO_ff2 <= MISO_ff1;
	end
end

//// bit_cntr to know when we shifted 8 bits ////
always_ff @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		bit_cntr <= 5'b00000;
	end else if (rst_cnt) begin 
		bit_cntr <= 5'b00000;
	end else if (shft) begin
		bit_cntr <= bit_cntr + 1;
	end
end

//// sclk div bit counter for the CLOCK ////
always_ff @(posedge clk, negedge rst_n) begin
	if(!rst_n)
		sclk_div <= 5'b10111;
	else if(rst_cnt) 
		sclk_div <= 5'b10111;
	else
	sclk_div <= sclk_div + 1;
end


//// assign SCLK to MSB of Sclock ////
assign SCLK = sclk_div[4];


//// Implement set_done/clr_done flop ////
always_ff @(posedge clk, negedge rst_n) begin
	if(!rst_n)
		done <= 1'b0;
	else if (set_done)
		done <= 1'b1;
	else if (clr_done)
		done <= 1'b0;
end

//// Implement slave select flop ////
always_ff @(posedge clk, negedge rst_n) begin
	if(!rst_n)
		SS_n <= 1'b1;
	else if (set_done)
		SS_n <= 1'b1;
	else if (clr_done)
		SS_n <= 1'b0;
end

//// MISO smpl flop ////
always_ff @(posedge clk) begin
	if(smpl)
		MISO_smpl <= MISO_ff2;
end

//// rd_data flop (with optional else) ////
always_ff @(posedge clk, negedge rst_n) begin
	if(!rst_n)
		rd_data <= 16'h0000;
	else if(wrt) 
		rd_data <= cmd;
	else if(shft) 
		rd_data <= {rd_data[14:0], MISO_smpl};
end

//// assign MOSI  ////
assign MOSI = rd_data[15];


//// Infer state register next ////
always @(posedge clk, negedge rst_n) begin
	if (!rst_n) begin
		state <= IDLE;
	end else begin
		state <= nxt_state;
	end
end

  //////////////////////////////////////
  // Implement state tranisiton logic //
  /////////////////////////////////////

  always_comb begin
  	  //////////////////////
      // Default outputs //
      ////////////////////
      rst_cnt = 1'b0; 
      set_done = 1'b0;
      clr_done = 1'b0;
      nxt_state = IDLE;
      shft = 1'b0;

      case(state)

      	//// transition to FRONTPORCH on wrt ////
      	IDLE: begin 
      		rst_cnt = 1'b1;
      		if(wrt) begin
      			nxt_state = FRONT_PORCH;
      			clr_done = 1'b1;
      		end else begin
      			nxt_state = IDLE;
      			set_done = 1'b1;
      		end
      	end

      	FRONT_PORCH: begin
      		if(sclk_div == shft_bits) begin
      			nxt_state = BITS;
      		end else begin
      			nxt_state = FRONT_PORCH;
      		end
      	end

      	/// Actual shifting and sampling happens here ///
      	BITS: begin 
      		if(bit_cntr == 4'b1111 & sclk_div == smpl_bits) begin
      			nxt_state = BACK_PORCH;
      			smpl = 1'b1; 
      		end else if(sclk_div == smpl_bits) begin
      			nxt_state = BITS;
      			smpl = 1'b1;
      		end else if(sclk_div == shft_bits) begin
      			nxt_state = BITS;
      			shft = 1'b1;
      		end else begin
      			nxt_state = BITS;
      		end
      	end 

      	BACK_PORCH: begin 
      		if(sclk_div == shft_bits) begin
      			set_done = 1;
      			rst_cnt = 1;
      			shft = 1'b1;
      			nxt_state = IDLE;
      		end else begin
      			nxt_state = BACK_PORCH;
      		end
      	end 
      	default: 
      	nxt_state = IDLE;
endcase // state
end
endmodule