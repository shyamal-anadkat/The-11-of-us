module rst_synch(RST_n, clk, rst_n);

input RST_n, clk;
reg out1, out2;
output rst_n;

always_ff@(negedge clk, negedge RST_n) begin
    if (~RST_n) begin
	out1 <= 1'b0;
	out2 <= 1'b0;
    end
    else begin
	out1 <= 1'b1;
	out2 <= out1;
    end
end

assign rst_n = out2;

endmodule
