module PB_release(PB, rst_n, clk, released);

input PB, rst_n, clk;
output released;

reg out1, out2, out3;

always_ff@(posedge clk, negedge rst_n) begin
    if (~rst_n) begin
	out1 <= 1'b0;
	out2 <= 1'b0;
	out3 <= 1'b0;
    end
    else begin
	out1 <= PB;
	out2 <= out1;
	out3 <= out2;
    end
end

assign released = out2 & ~out3;

endmodule
