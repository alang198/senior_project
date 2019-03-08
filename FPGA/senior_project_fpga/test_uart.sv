module test_uart(
	input clk,
	output logic uart_start,
	output logic [11:0][7:0]cmd_buf //12 bytes (6 words)
);

always@(posedge clk)
begin
	uart_start <= 1'b1;
	cmd_buf[0] <= 8'h14;
	cmd_buf[1] <= 8'h00;
	cmd_buf[2] <= 8'h00;
	cmd_buf[3] <= 8'h01;
	cmd_buf[4] <= 8'h98;
	cmd_buf[5] <= 0;
	cmd_buf[6] <= 0;
	cmd_buf[7] <= 0;
	cmd_buf[8] <= 0;
	cmd_buf[9] <= 0;
	cmd_buf[10] <= 0;
	cmd_buf[11] <= 0;
end
endmodule
