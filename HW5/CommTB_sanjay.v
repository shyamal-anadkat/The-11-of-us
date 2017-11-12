module CommTB ();

reg[16:0] data_in;
reg [8:0] cmd_in;
reg snd_cmd, clk, rst_n;

wire [15:0] data_out;
wire [7:0] resp, cmd_out;
wire TX_RX, RX_TX, frm_snt, resp_rdy, cmd_rdy, resp_sent;

// CommMaster DUT
CommMaster iComm(.resp(resp), .resp_rdy(resp_rdy), .frm_snt(frm_snt), .TX(TX_RX),
			     .data(data_in), .cmd(cmd_in), .clk(clk), .rst_n(rst_n), .snd_cmd(snd_cmd),
				 .RX(RX_TX));

// Wrapper DUT. Not sure if we need snd_resp
UART_wrapper iwrapper(.clk(clk), .rst_n(rst_n), .RX(RX_TX), .TX(TX_RX), .cmd(cmd_out),
				      .data(data_out), .cmd_rdy(cmd_rdy), .snd_resp(), .resp_sent(resp_sent),
					  .resp(resp), .clr_cmd_rdy());

initial begin
	clk = 0;
	rst_n = 0;
	snd_cmd = 1;
	
	@(posedge clk);
	@(negedge clk);
	rst_n = 1;
	
	for (cmd_in = 9'h00; cmd_in < 9'hff; cmd_in = cmd_in + 1) begin
		for (data_in = 17'h0000; data_in < 17'hffff; data_in = data_in + 1) begin
			@(posedge cmd_rdy);
			
			if (cmd_out != cmd_in) begin
				$display("ERROR: Expected cmd 0x%h, Received 0x%h\n", cmd_in, cmd_out);
				$stop();
			end
			
			if (data_out != data_in) begin
				$display("ERROR: Expected cmd 0x%h, Received 0x%h\n", data_in, data_out);
				$stop();
			end
		end
	end
	
	$display("All tests passed");
	$stop();
end

always
	#10 clk = ~clk;
endmodule
