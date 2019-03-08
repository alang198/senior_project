module spi_recieve(
	input clk,
	input data_in,
	input reset,
	input clk_50,
	output logic [15:0]bytes_recieved,
	output logic [15:0]byte_out,
	output logic [14:0]address,
	output logic write_pulse
);

logic [15:0]data;
logic [7:0]count; //count bits recieved

//data clocked in on posedge
always@(posedge clk or posedge reset)
begin
	if(reset == 1'b1) 
	begin
		bytes_recieved <= 0;
		address <= 0;
		count <= 0;
	end
	
	else
	begin
		data[15-count] <= data_in;
		
		
		if(count == 15)
		begin
			//write_pulse <= 1'b1;
			count <= 0;
			bytes_recieved <= bytes_recieved + 2;
			address <= address + 1'b1;
		end
		else 
		begin
			count <= count + 1'b1;
			//write_pulse <= 1'b0;
		end
	end	
end

always@(posedge clk_50)
begin
byte_out <= data;

if(count == 15) write_pulse <= 1'b1;
else write_pulse <= 1'b0;
end
endmodule
