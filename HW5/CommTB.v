module CommTB ();

reg[15:0] data_in;
reg [7:0] cmd_in;
reg snd_cmd, clk, rst_n;

wire [15:0] data_out;
wire [7:0] resp, cmd_out;
wire TX_RX, RX_TX, frm_snt, resp_rdy, cmd_rdy, resp_sent;
reg [15:0] tst_data [3:0];
reg [7:0] tst_cmds [3:0];
integer i;

// CommMaster DUT
CommMaster iComm(.resp(resp), .resp_rdy(resp_rdy), .frm_snt(frm_snt), .TX(TX_RX),
		 .data(data_in), .cmd(cmd_in), .clk(clk), .rst_n(rst_n), .snd_cmd(snd_cmd),
		 .RX(RX_TX));

// Wrapper DUT. Not sure if we need snd_resp
UART_wrapper iwrapper(.clk(clk), .rst_n(rst_n), .RX(TX_RX), .TX(RX_TX), .cmd(cmd_out),
		      .data(data_out), .cmd_rdy(cmd_rdy), .snd_resp(), .resp_sent(resp_sent),
		      .resp(resp), .clr_cmd_rdy());

initial begin
	clk = 0;
	rst_n = 0;
	snd_cmd = 0;
	tst_data[0] = 16'h0060;
	tst_data[1] = 16'h0076;
	tst_data[2] = 16'hffff;
	tst_data[3] = 16'h0421;
	tst_cmds[0] = 8'h20;
	tst_cmds[1] = 8'h43;
	tst_cmds[2] = 8'hff;
	tst_cmds[3] = 8'h93;
end

always
	#10 clk = ~clk;

always begin
	@(posedge clk);
	@(negedge clk);
	rst_n = 1;
	
	for (i = 0; i < 4; i=i+1) begin
		cmd_in = tst_cmds[i];
		data_in = tst_data[i];
		snd_cmd = 1;
		@(posedge clk);
		snd_cmd = 0;
		@(posedge frm_snt);

		if (!cmd_rdy) begin
			$display("ERROR: cmd_rdy should be high");
			$stop();
		end

		if (cmd_out != cmd_in) begin
			$display("ERROR: Expected cmd 0x%h, Received 0x%h\n", cmd_in, cmd_out);
			$stop();
		end
			
		if (data_out != data_in) begin
			$display("ERROR: Expected data 0x%h, Received 0x%h\n", data_in, data_out);
			$stop();
		end
	end
	
	$display("All tests passed");
	$stop();
end
endmodule
