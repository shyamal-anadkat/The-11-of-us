module UART_wrapper(clk, rst_n, RX, TX, cmd, data, cmd_rdy, snd_resp, resp_sent, resp, clr_cmd_rdy);
input clk, rst_n, TX, RX, snd_resp, clr_cmd_rdy;
input [7:0] resp;
wire clr_cmd_rdy, snd_resp, resp_sent;
output [15:0] data;
output [7:0] cmd;
output resp_sent;
output reg cmd_rdy;
reg assert_mux_1, assert_mux_2;
wire [7:0] curr_data;
reg set_cmd_rdy, clr_cmd_rdy_i;

reg rdy, clr_rdy;
UART uart(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .rx_rdy(rdy), .clr_rx_rdy(clr_rdy), 
          .rx_data(curr_data), .trmt(snd_resp), .tx_data(resp), .tx_done(resp_sent));

typedef enum {WAIT1, WAIT2, WAIT3} State;
State state, next_state;

reg [7:0] data_ff1, data_ff2;

// ffs to control rx_data handling
always_ff@(posedge clk, negedge rst_n) begin
  if (~rst_n) begin
    data_ff1 <= 8'd0;
    data_ff2 <= 8'd0;
  end else begin
    if (assert_mux_1)
      data_ff1 <= curr_data;
    if (assert_mux_2)
      data_ff2 <= curr_data;
  end
end

assign cmd = data_ff1;
assign data = {data_ff2, curr_data};

// ff to control state transitions
always_ff@(posedge clk, negedge rst_n) begin
  if (~rst_n) begin
    state <= WAIT1;
  end else begin
    state <= next_state;
  end
end

// ff to control cmd_rdy
always_ff@(posedge clk, negedge rst_n) begin
  if (~rst_n) begin
    cmd_rdy = 1'b0;
  end else begin
    if (clr_cmd_rdy | clr_cmd_rdy_i)
      cmd_rdy = 1'b0;
    else if (set_cmd_rdy)
      cmd_rdy = 1'b1;
  end
end

// state contorl logic
always_comb begin
  next_state = WAIT1;
  assert_mux_1 = 1'b0;
  assert_mux_2 = 1'b0;
  clr_rdy = 1'b0;
  set_cmd_rdy = 1'b0;
  clr_cmd_rdy_i = 1'b0;
  case(state)
    WAIT1:
      begin
        if (rdy) begin
          next_state = WAIT2;
          assert_mux_1 = 1'b1;
          clr_rdy = 1'b1;
          clr_cmd_rdy_i = 1'b1;
        end
      end
    WAIT2:
      begin
        if (rdy) begin
          next_state = WAIT3;
          assert_mux_2 = 1'b1;
          clr_rdy = 1'b1;
        end
      end
    WAIT3:
      begin
        if (rdy) begin
          next_state = WAIT1;
          clr_rdy = 1'b1;
          set_cmd_rdy = 1'b1;
        end
      end
  endcase
end

endmodule
