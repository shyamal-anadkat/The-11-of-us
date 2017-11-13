module A2D_test (LED, SS_n, SCLK, MOSI, MISO, rst_n, clk);

output logic [7:0] LED;
output logic SS_n, SCLK, MOSI;

input rst_n, clk, MISO;

logic cnv_cmplt, strt_cnv;
logic [11:0] res;

reg rst_n_sync, rst_n_1;

always_ff@(posedge clk) begin
  rst_n_1 <= rst_n;
  rst_n_sync <= rst_n_1;
end


A2D_intf A2DINTF (
.cnv_cmplt(cnv_cmplt), 
.res(res), 
.SS_n(SS_n), 
.strt_cnv(strt_cnv), 
.chnnl(3'd0),  
.SCLK(SCLK), 
.MOSI(MOSI), 
.MISO(MISO), 
.clk(clk), 
.rst_n(rst_n_sync));

typedef enum reg {IDLE, DONE} State;
State state, next_state;

always_ff@(posedge clk or negedge rst_n_sync) begin
  if (~rst_n_sync) begin
    state <= IDLE;
  end else begin
    state <= next_state;
  end
end

always_comb begin
  next_state = IDLE;
  strt_cnv = 1'b0;
  case(state)
    IDLE: begin
      strt_cnv = 1'b1;
      next_state = DONE;
    end
    DONE: begin
      next_state = DONE;
      if (cnv_cmplt) 
        next_state = IDLE;
    end
  endcase
end


always_ff @(posedge clk or negedge rst_n_sync) begin 
if(!rst_n_sync) 
  LED <= 8'h00;
else if(cnv_cmplt)
  //upper 8 bits of conversion results
  LED <= res[11:4]; 
end

endmodule
