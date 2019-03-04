module clk_divide(
	input clk,
	output clk_out
);

reg [7:0]count;

always@(posedge clk)
begin
	if(count == 217)
	begin
		clk_out <= ~clk_out;
		count <= 0;
	end
	else count <= count + 1'b1;

end
endmodule
