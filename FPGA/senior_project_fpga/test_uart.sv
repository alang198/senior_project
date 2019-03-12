module test_uart(
	input clk,
	output logic uart_start,
	output logic [11:0][7:0]cmd_buf //12 bytes (6 words)
);

always@(posedge clk)
begin
	uart_start <= 1'b1;
	cmd_buf[0] <= 8'h30;
	cmd_buf[1] <= 8'h24;
	cmd_buf[2] <= 8'h00;
	cmd_buf[3] <= 8'hB0;
	cmd_buf[4] <= 8'h5E;
	cmd_buf[5] <= 0;
	cmd_buf[6] <= 0;
	cmd_buf[7] <= 0;
	cmd_buf[8] <= 0;
	cmd_buf[9] <= 0;
	cmd_buf[10] <= 8'h07;
	cmd_buf[11] <= 0;
end
endmodule
