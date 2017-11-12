module A2D_tb();
logic strt_cnv, clk, rst_n;
logic [2:0] chnnl;
wire a2d_SS_n, SCLK, MOSI, MISO;
wire cnv_cmplt;
wire[11:0] res;
logic done;

A2D_intf intf(.clk(clk),.rst_n(rst_n), .strt_cnv(strt_cnv),.chnnl(chnnl),
         .cnv_cmplt(cnv_cmplt),.res(res),.SS_n(a2d_SS_n),.SCLK(SCLK),
         .MOSI(MOSI),.MISO(MISO),.done(done));

ADC128S adc(.clk(clk),.rst_n(rst_n),.SS_n(a2d_SS_n),.SCLK(SCLK),.MOSI(MOSI),.MISO(MISO));

initial begin
  clk = 1'b0;
  rst_n = 1'b1;
  strt_cnv = 1'b0;
  chnnl = 3'd0;
end

always begin
  #10 clk = ~clk;
end
integer i;
always begin
  @(negedge clk);
  @(posedge clk);
  rst_n = 1'b0;
  @(posedge clk);
  rst_n = 1'b1;
  for (i = 0; i <= 86; i=i+1) begin
    strt_cnv = 1'b1;
    chnnl = 3'd0;
    // Wait for two 16 bit transactions
    repeat(32)@(posedge SCLK);
    @(posedge cnv_cmplt);

    if (res != (12'hC00 - 12'h010*i)) begin
      $display("TEST FAILED: res should be Oh%h and is 0h%h", (12'hC00 - 12'h010*i), res);
      $stop();
    end
  end
  $display("TESTS PASSED");
  $stop();
end
endmodule
