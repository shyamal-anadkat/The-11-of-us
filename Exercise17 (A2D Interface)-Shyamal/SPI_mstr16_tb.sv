/* Author: Shyamal Anadkat <<->> SPI_mstr16 testbench */

//////////////////////////////////////////////////|
// Testbench for SPI Master                      ||
//////////////////////////////////////////////////|
module SPI_mstr16_tb();

//////////////////////////////////////////////////|
//// stimuli and required signals ////
//////////////////////////////////////////////////|
logic clk, rst_n;

logic SS_n, SCLK, MOSI, done;
logic [15:0] rd_data;

logic wrt, MISO;
logic [15:0] exp_resp;      
logic [15:0] rd_skip;
logic err;
integer i;

//////////////////////////////////////////////////|
//// instantiate slave and master iDUTs ////
//////////////////////////////////////////////////|
ADC128S iDUT_slave(
  .clk(clk),
  .rst_n(rst_n),
  .SS_n(SS_n),
  .SCLK(SCLK),
  .MISO(MISO),
  .MOSI(MOSI));

SPI_mstr16  iDUT_master(.clk(clk), 
  .rst_n(rst_n), 
  .cmd(16'h0000), 
  .wrt(wrt), .MISO(MISO), 
  .MOSI(MOSI), .SS_n(SS_n), 
  .SCLK(SCLK), .done(done), 
  .rd_data(rd_data));

//////////////////////////////////////////////////|
//// start testbench - initial block           ////
//////////////////////////////////////////////////|
initial begin 

  clk = 0;
  rst_n = 0;
  err = 1'b0;

  @(posedge clk);
  @(negedge clk)
  rst_n = 1;
  rd_skip = 0;

//////////////////////////////////////////////////|
//// exhaustive, self-checking test ////
//////////////////////////////////////////////////|
for(i = 0; i < 100 ; i = i + 1) begin

  @(posedge clk)  wrt = 1;
  @(posedge clk)  wrt = 0;
  @(posedge done) wrt = 1;
  @(posedge clk)  wrt = 0;
  @(posedge clk);


  if(done) begin

        //// display error and stop ////
        $display("Error in DONE");
        $stop;
      end
        @(posedge done);  // we have our data

        //// expected response from master ////
        exp_resp =  (16'hC00 - (i*8'h10));

        if(done) begin
          if(rd_data == exp_resp) begin
            //// do nothing ////
          end else begin 
            $display("ERROR iter: 
              %d, rd_data: %h, exp_resp: %h", 
              rd_skip, rd_data, exp_resp);
            err = 1'b1;
            $stop; 
          end
        end
        else begin
          $display("ERROR: done not asserted");
          $stop; 
        end
      end


//////////////////////////////////////////////////|
//// print out yahoo type message to make you happy ////
//////////////////////////////////////////////////|
if(!err)
  $display("SUCCESS: TEST PASSED.");
$stop; 

end 
always
#1 clk = ~clk;

endmodule