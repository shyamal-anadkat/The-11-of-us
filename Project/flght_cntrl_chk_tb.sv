module flght_cntrl_chk_tb ();

	reg clk;
	integer i;
///////////////////////////////////|
reg [107:0] stim_mem [0:999];  
reg [107:0] stim;    
reg [43:0] response;
reg [43:0] expected_mem [0:999];
///////////////////////////////////|
logic vld;
/*
There are 1000 vectors of stimulus and
response. Read each file into a memory using
$readmemh.

Loop through the 1000 vectors and apply the
stimulus vectors to the inputs as specified.
Then wait till #1 time unit after the rise of clk
and compare the DUT outputs to the response
vector (self check). Do all 1000 vectors match?
*/


//////////////////////
// Instantiate DUT //
////////////////////
flght_cntrl iDUT(
	.clk(clk),
	.rst_n(stim[107]),
	.vld(vld),
	.inertial_cal(stim[105]),
	.d_ptch(stim[104:89]),
	.d_roll(stim[88:73]),
	.d_yaw(stim[72:57]),
	.ptch(stim[56:41]),
	.roll(stim[40:25]),
	.yaw(stim[24:9]),
	.thrst(stim[8:0]),
	.frnt_spd(response[43:33]),
	.bck_spd(response[32:22]),
	.lft_spd(response[21:11]),
	.rght_spd(response[10:0])
	);

initial begin 
	clk = 0;

  // Read in files
  $readmemh("flght_cntrl_stim.hex",stim_mem);
  $readmemh("flght_cntrl_resp.hex",expected_mem);

  // Go over the 1000 vectors in the files
  for(i = 0; i < 1000; i = i + 1) begin 
        // Get current stimulus
        stim = stim_mem[i];
        vld = stim[106];
        // make sure valid is only high one clock cycle not to mess with the queue, so I might suggest  manually 
        // setting vld to zero after the posedge of clk of the stimulus applied. 
        // Wait so we are #1 after rising clock edge
        @(posedge clk)
        vld = 1'b0;
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        #1
        // See if out DUT matches the expected output
        if (response !== expected_mem[i]) begin
        	$display("ERR: i = %d , expected: %h and response: %h not same.", 
        		i, expected_mem[i] , response);
        	$display("frnt is %d, should be %d", response[43:33], expected_mem[i][43:33]);
        	$display("bck is %d, should be %d", response[32:22], expected_mem[i][32:22]);
        	$display("lft is %d, should be %d", response[21:11], expected_mem[i][21:11]);
        	$display("rght is %d, should be %d", response[10:0], expected_mem[i][10:0]);
        	$stop();
        end
        @(negedge clk);
    end
    $display("***SUCCESS: TESTS PASSED***");
    $stop();
end

always
#5 clk = ~clk;

endmodule