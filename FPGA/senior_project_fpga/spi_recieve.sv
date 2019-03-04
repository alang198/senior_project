module spi_recieve(
	input logic clk,
	input logic data_in,
	input logic reset,
	output logic [15:0]bytes_recieved,
	output logic [15:0]byte_out,
	output logic [7:0]count, //count bits recieved
	output logic [14:0]address,
	output logic write_pulse
);

logic byte_flag; //choose which buffer to write to

logic [15:0]data0;
logic [15:0]data1;

//data clocked in on posedge
always@(posedge clk or posedge reset)
begin
	if(reset == 1'b1) bytes_recieved <= 0;
	
	else
	begin
		if(byte_flag == 1'b0) data0[15-count] <= data_in;
		else data1[15-count] <= data_in;
		
		if(count == 15)
		begin
			write_pulse <= 1'b1;
			
			count <= 0;
			byte_flag <= ~byte_flag;
			
			if(byte_flag == 1'b0) byte_out <= data0;
			else byte_out <= data1;
			
			bytes_recieved <= bytes_recieved + 2;
			
			if(address == 32767) address <= 0;
			else address <= address + 1'b1;
		end
		else 
		begin
			count <= count + 1'b1;
			write_pulse <= 1'b0;
		end
	end
	
end
endmodule
