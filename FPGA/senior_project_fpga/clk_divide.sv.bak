module clk_divide(
	input clk,
	output reg clk_out
);

reg [31:0]count;

always_ff@(posedge clk)
begin
	if(count == 567)
	begin
		count <= 0;
		clk_out <= ~clk_out;
	end
	else count <= count + 1'b1;
end
endmodule