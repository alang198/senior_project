module memory_buffer(
	input clk,
	input read_pulse,
	input [7:0]spi_count,
	input [15:0]spi_byte_in,

	//output logic [15:0]word_out
	output logic [14:0]address,
	output logic write_pulse,
	output logic data_out

);

(* ramstyle = "M9K" *) logic [32768:0][15:0]mem_buf;
logic [31:0]write_index;
logic [31:0]read_index;

logic flag;

assign address = (read_pulse) ? read_index : write_index;

always_ff@(posedge clk)
begin
	if(write_pulse == 1) write_pulse <= 0;

	if((flag == 1) && (spi_count == 0)) flag <= 0; 
	//write data to buffer
	else if(spi_count == 15)
	begin
		data_out <= spi_byte_in;
		write_pulse <= 1;
		flag <= 1;
		if(write_index == 32768) write_index <= 0;
		else write_index <= write_index + 1'b1;
	end
end
/*
//read data in buffer
always_ff@(posedge clk)
begin
	if(read_pulse == 1'b1)
	begin
		word_out <= 
		
		if(read_index >= 65535) read_index <= 0;
		else read_index <= read_index + 2;
	end
end*/
endmodule
