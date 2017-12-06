module flght_cntrl_tb();

reg clk, rst_n;
reg vld;
reg inertial_cal;
reg [15:0] d_ptch, d_roll, d_yaw;
reg [15:0] ptch, roll, yaw;
reg [8:0] thrst;


///////////////////////////////////////
// Declare wires for outputs of DUT //
/////////////////////////////////////
wire [10:0] frnt_spd,bck_spd,lft_spd,rght_spd;

  localparam CAL_SPEED = 11'h1B0;		// speed to run motors at during inertial calibration
  localparam MIN_RUN_SPEED = 13'h200;	// minimum speed while running  

initial begin
  clk = 0;
  rst_n = 0;
  inertial_cal = 1;
  d_ptch = 16'h0000;
  ptch = 16'h0000;
  d_roll = 16'h0000;
  roll = 16'h0000;
  d_yaw = 16'h0000;
  yaw = 16'h0000;
  thrst = 9'h123;
  vld = 1;
  repeat(2) @(negedge clk);
  rst_n = 1;
  @(negedge clk);
  if ((frnt_spd===CAL_SPEED) && (bck_spd===CAL_SPEED) && (lft_spd===CAL_SPEED) && (rght_spd===CAL_SPEED))
    $display("GOOD1: CAL_SPEED comes out when inertial_cal cal is high");
  else begin
    $display("ERROR: all 4 channels should be CAL_SPEED when inertial_cal is high");
	$stop();
  end
  
  inertial_cal = 0;
  @(negedge clk);
  
  if ((frnt_spd===(MIN_RUN_SPEED+thrst)) && (bck_spd===(MIN_RUN_SPEED+thrst)))
    $display("GOOD2: all terms zero, so thrst + MIN_RUN_SPEED should be answer");
  else begin
    $display("ERROR: speed should be simply thrst + MIN_RUN_SPEED since all other terms should be zero");
	$stop();
  end
  
  ptch = 16'h010;		// + error, but no saturation
  @(negedge clk);
 
  if ((frnt_spd===11'h2a9) && (bck_spd===11'h39d))
    $display("GOOD3: ptch_pterm and ptch_dterm are correct for + non-sat case");
  else begin
    $display("ERROR: frnt_spd should be 11'h2a9 and bck_spd should be 11'h39d");
	$display("INFO: your ptch_pterm should be 10'h00a and your ptch_dterm should be 12'h070");
	$stop();
  end 
  
  ptch = 16'h080;		// case of ptch_D_diff will saturate, but ptch_err will not
  @(negedge clk);
 
  if ((frnt_spd===11'h1fa) && (bck_spd===11'h44c))
    $display("GOOD4: ptch_pterm and ptch_dterm are correct for + sat of ptch_D_diff");
  else begin
    $display("ERROR: frnt_spd should be 11'h1fa and bck_spd should be 11'h44c\n");
	$display("INFO: your ptch_pterm should be 10'h050 and your ptch_dterm should be 12'h0d9\n");
	$stop();
  end 
  
  ptch = 16'h220;		// Now ptch_err_sat should saturate to 3ff
  @(negedge clk);
 
  if ((frnt_spd===11'h10c) && (bck_spd===11'h53a))
    $display("GOOD5: ptch_pterm and ptch_dterm are correct for + sat of ptch_err_sat to 1ff");
  else begin
    $display("ERROR: frnt_spd should be 11'h10c and bck_spd should be 11'h53a\n");
	$display("INFO: your ptch_pterm should be 10'h13e and your ptch_dterm should be 12'h0d9\n");
	$stop();
  end 
  
  ptch = 16'hFFF0;		// Now for negative error that should not saturate anything
  @(negedge clk);
 
  if ((frnt_spd===11'h39d) && (bck_spd===11'h2a9))
    $display("GOOD6: ptch_pterm and ptch_dterm are correct for - non-sat case");
  else begin
    $display("ERROR: frnt_spd should be 11'h39d and bck_spd should be 11'h2a9\n");
	$display("INFO: your ptch_pterm should be 10'h3f6 and your ptch_dterm should be 12'hf90\n");
	$stop();
  end
  
  ptch = 16'hFF80;		// Now for - error that should saturate ptch_D_diff but not ptch_err_sat
  @(negedge clk);
 
  if ((frnt_spd===11'h453) && (bck_spd===11'h1f3))
    $display("GOOD7: ptch_pterm and ptch_dterm are correct for - ptch_D_diff sat case");
  else begin
    $display("ERROR: frnt_spd should be 11'h453 and bck_spd should be 11'h1f3\n");
	$display("INFO: your ptch_pterm should be 10'h3b0 and your ptch_dterm should be 12'hf20\n");
	$stop();
  end 
  
  ptch = 16'hFDE0;		// Now for - error that should saturate ptch_err_sat
  @(negedge clk);
 
  if ((frnt_spd===11'h543) && (bck_spd===11'h103))
    $display("GOOD8: ptch_pterm and ptch_dterm are correct for - ptch_err_sat case");
  else begin
    $display("ERROR: frnt_spd should be 11'h543 and bck_spd should be 11'h103\n");
	$display("INFO: your ptch_pterm should be 10'h2C0 and your ptch_dterm should be 12'hf20\n");
	$stop();
  end 
  
  yaw = 16'hFF80;		// Now put yaw error negative such that we get +yaw contribution to frnt_spd
  @(negedge clk);
 
  if ((frnt_spd===11'h673) && (bck_spd===11'h233))
    $display("GOOD9: math makes sense for introduction of yaw");
  else begin
    $display("ERROR: frnt_spd should be 11'h673 and bck_spd should be 11'h233\n");
	$display("INFO: your ptch_pterm should be 10'h2C0 and your ptch_dterm should be 12'hf20\n");
	$display("INFO: now have non zero yaw term.  Are you subtracting yaw from frnt_spd/back_spd");
	$stop();
  end 
  
   yaw = 16'hFE00;		// Now make yaw term big enough that frnt_spd should saturate
   thrst = 9'h1E0;		// increasing thrust to make saturation happen
  @(negedge clk);
 
  if ((frnt_spd===11'h7ff) && (bck_spd===11'h3e0))
    $display("GOOD10: math makes sense for saturation of frnt_spd");
  else begin
    $display("ERROR: frnt_spd should be 11'h7ff and bck_spd should be 11'h3e0\n");
	$display("INFO: Perhaps your unsigned 11-bit saturation of output is wrong\n");
	$stop();
  end 
  
  vld = 0;		// now lower valid for a couple of clocks to ensure queue not always shifting
  repeat(2) @(negedge clk);
  vld = 1;		// raise vld back up and shift 7 more clocks to get diff queue full
  yaw = 16'h0000;
  ptch = 16'h0000;
  thrst = 9'h123;
  repeat(6) @(negedge clk);
  
  if ((frnt_spd===11'h393) && (bck_spd===11'h2b3))
    $display("GOOD11: first data coming from prev queue looks good");
  else begin
    $display("ERROR: frnt_spd should be 11'h393 and bck_spd should be 11'h2b3\n");
	$display("INFO: Is your prev_queue working right?\n");
	$stop();
  end 
  
  @(negedge clk);
  
  if ((frnt_spd===11'h403) && (bck_spd===11'h243))
    $display("GOOD12: second data coming from prev queue looks good");
  else begin
    $display("ERROR: frnt_spd should be 11'h403 and bck_spd should be 11'h243\n");
	$display("INFO: Is your prev_queue working right?\n");
	$stop();
  end  
  
  $display("YAHOO!! all tests passed...only checked pitch in detail\n");
  $display("        ensure your roll and yaw code are a match of pitch\n");
  $stop();
end

always
  #5 clk = ~clk;
  
//////////////////////
// Instantiate DUT //
////////////////////
  flght_cntrl iDUT(.clk(clk),.rst_n(rst_n),.vld(vld),.d_ptch(d_ptch),.d_roll(d_roll),
                   .d_yaw(d_yaw),.ptch(ptch),.roll(roll),.yaw(yaw),.thrst(thrst),
                   .inertial_cal(inertial_cal),.frnt_spd(frnt_spd),.bck_spd(bck_spd),
				   .lft_spd(lft_spd),.rght_spd(rght_spd));
		 
endmodule