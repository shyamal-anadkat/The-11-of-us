// Initialize task
task Initialize();
    clk = 1'b0;
    RST_n = 1'b0;
    cmd_to_copter = 8'h00;
    data = 16'h0000;
    send_cmd = 1'b0;
    clr_resp_rdy = 1'b0;
    @(posedge clk);
    @(negedge clk);
    RST_n = 1'b1;
    @(posedge clk);
endtask

// Send command task
task SendCmd(input [7:0] comd, input [15:0] dat);
    data = dat;
    cmd_to_copter = comd;
    send_cmd = 1'b1;
    @(posedge clk);
    send_cmd = 1'b0;
    if (comd != CALIBRATE)
    ChkSndResp;  // Snd Resp should be asserted before command is sent
    @(posedge cmd_sent);
    // From the delay in CALIBRATE, snd resp comes after cmd_sent
    if (comd == CALIBRATE)
    ChkSndResp;
endtask

// Check response command
task ChkResp(input [7:0] expected);
    @(posedge resp_rdy);
    if (resp != expected) begin
        $display("inertial interface command is incorrect. Was 0x%x should be 0x%x", resp, expected);
        $stop();
    end
    ClrRdy;
endtask

// check for ack response
task ChkPosAck;
    @(posedge resp_rdy);
    if (resp != 8'hA5) begin
        $display("Should have received a pos ack as response");
        $stop();
    end
    ClrRdy;
endtask

// Knock down rdy signal
task ClrRdy;
    clr_resp_rdy = 1'b1;
    @(posedge clk);
    clr_resp_rdy = 1'b0;
endtask

// check if act is equal to exp. If not print message and stop sim
task ChkVal16(input [15:0] act, input [15:0] exp, input [8*8:0] name);
  if (act != exp) begin
    $display("%s incorrectly set. Was 0x%x, should be 0x%x", name, act, exp);
    $stop();
  end
endtask

// block until send_resp is asserted
task ChkSndResp;
    @(posedge iDUT.send_resp);
endtask

// check if act is equal to exp. If not print message and stop sim
task ChkVal(input act, input exp, input [6*8:0] name);
  if (act != exp) begin
    $display("%s incorrectly set. Was 0x%x, should be 0x%x", name, act, exp);
    $stop();
  end
endtask

// make input its absolute value
task Abs16(inout [15:0] a);
  if (a < 0)
    a = -a;
endtask

// check if act and exp are within %perc of each other
task ChkPerc(input [15:0] act, input [15:0] exp, input[6:0] perc, input [6*8:0] name);
  reg[15:0] result;
  reg[15:0] denom;
  Abs16(act);
  Abs16(exp);

  // devide by the larger of the two
  if (act > exp) begin
    denom = act;
  end else begin
    denom = exp;
  end
  // do percent error and devide by largest number so its never > 1
  assign result = 100*(act - exp)/denom;
  Abs16(result);
  // subtract error from 100 to get percent accuracy
  if ((100 - result) < perc) begin
    $display("%s not converging. Was 0x%x, should be within %d percent of 0x%x. Actual perc %d", name, act, perc, exp, (100 - result));
    $stop();
  end
endtask

// check if act > exp. If not, print message and end sim
task ChkGtr(input [10:0] act, input [10:0] exp, input [10*8:0] name);
  if (act <= exp) begin
    $display("%s was not greater than expected value. Was 0x%x should be greater than 0x%x", name, act, exp);
    $stop();
  end
endtask

// check if act < exp. If not, print message and end sim
task ChkLess(input [10:0] act, input [10:0] exp, input [10*8:0] name);
  if (act >= exp) begin
    $display("%s was not less than expected value. Was 0x%x should be less than 0x%x", name, act, exp);
    $stop();
  end
endtask
