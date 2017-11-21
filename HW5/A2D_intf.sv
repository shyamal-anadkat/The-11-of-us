/*
* A2D_Intf Module ECE 551 
* Eric Heinz, Shyamal Anadkat, Sanjay Rajmohan 
*
*/
module A2D_intf (cnv_cmplt, res, SS_n, strt_cnv, chnnl,  SCLK, MOSI, MISO, clk, rst_n);

///////////////////////////////////////////////
// Module Interface Input and Outputs /////////
///////////////////////////////////////////////
input clk, rst_n;
input strt_cnv;
input [2:0] chnnl;

////SPI mstr inputs ////
input MISO;
output logic cnv_cmplt;
output logic [11:0] res;
//// SPI mstr outputs ////
output logic SS_n, SCLK, MOSI;

logic set_cmplt, clr_cmplt;
logic wrt, done;
logic [15:0] rd_data;
////////////////////////////////////////////////


///////////////////////////////////////////////
// State Definitions                 //////////
///////////////////////////////////////////////
typedef enum reg [1:0] {IDLE, TRANS1, WAIT, DONE} state_t;
state_t state, nxt_state;


//// instantiate SPImstr 16 ////
SPI_mstr16 SPImstr16(
	.SS_n(SS_n), 
	.SCLK(SCLK), 
	.MOSI(MOSI), 
	.done(done), 
	.rd_data(rd_data), 
	.MISO(MISO), 
	.wrt(wrt), 
	.cmd({2'b00, chnnl, 11'h000}), 
	.clk(clk), 
	.rst_n(rst_n));


assign res = rd_data[11:0];   //// only lower 12 buts valid ////


//// cnv_cmplt flop ////
always_ff @(posedge clk or negedge rst_n) begin 
	if(!rst_n) 
		cnv_cmplt <= 1'b0;
	else if(clr_cmplt)
		cnv_cmplt <= 1'b0;
	else if(set_cmplt)
		cnv_cmplt <= 1'b1;
end

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
      set_cmplt = 1'b0;
      clr_cmplt = 1'b0;
      wrt = 1'b0;
      nxt_state = IDLE;

      case(state)

      	//// default/idle state /////
      	IDLE: begin 
      		if(strt_cnv) begin
      			nxt_state = TRANS1;
      			clr_cmplt = 1'b1;
      			wrt = 1'b1;
      		end else begin
      			nxt_state = IDLE;
      		end
      	end
      	//// send command to A2D via SPI to ask for conversion on channel////
      	TRANS1:begin 
      		if(done) begin
      			nxt_state = WAIT;
      			wrt = 1'b1;
      		end else begin
      			nxt_state = TRANS1;
      		end 
      	end

        WAIT: begin
          nxt_state = DONE;
        end

      	//// read result of A2D back with new transaction////
      	DONE:begin 
      		if(done) begin
      			wrt = 1'b0;
      			nxt_state = IDLE;
      			set_cmplt = 1'b1;
          end else begin
      			nxt_state = DONE;
         end
       end 
       default: 
       nxt_state = IDLE;

      endcase // state
    end
  endmodule