module CommTB();

wire TX;
reg clk, rst_n, RX, snd_resp, clr_cmd_rdy;
wire [15:0] data;
wire [7:0] cmd;
reg [7:0] resp;
wire resp_sent, cmd_rdy;

UART_wrapper wrap(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .cmd(cmd), .data(data), 
             .cmd_rdy(cmd_rdy), .snd_resp(snd_resp), .resp_sent(resp_sent),
             .resp(resp), .clr_cmd_rdy(clr_cmd_rdy));

initial begin
  clk = 0;
  rst_n = 1;
  RX = 1'b1;
  snd_resp = 1'b0;
  clr_cmd_rdy = 1'b0;
end

always
  #10 clk <= ~clk;

always begin
  repeat(12)@(posedge clk);
  RX = ~RX;
end

always begin
  @(negedge clk)
  rst_n = 1'b0;
  @(posedge clk)
  #11000
  $stop();
end
endmodule
