task Initialize();

clk = 1'b0;
RST_n = 1'b0;
cmd_to_copter = 8'h00;
data = 16'h0000;
send_cmd = 1'b0;
clr_resp_rdy = 1'b0;
@(posedge clk);
RST_n = 1'b1;
@(posedge clk);

endtask

task SendCmd;

endtask

task ChkResp;

endtask

task ChkPosAck;

endtask