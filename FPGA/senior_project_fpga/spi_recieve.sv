module spi_recieve(
	input clk,
	input data_in,
	input reset,
	input clk_50,
	output logic [15:0]bytes_recieved,
	output logic [15:0]data,
	output logic [14:0]address,
	output logic write_pulse
);

//logic [15:0]data;
logic [3:0]count; //count bits recieved

//the following code is sourced from fpga4fun
logic [2:0]spi_clk_reg;
//spi clock sync
always@(posedge clk_50) spi_clk_reg <= {spi_clk_reg[1:0], clk};
wire spi_clk_r = (spi_clk_reg[2:1] == 2'b01); //rising edge
wire spi_clk_f = (spi_clk_reg[2:1] == 2'b10); //falling edge
//data in
logic [1:0]data_in_reg;
always@(posedge clk_50) data_in_reg <= {data_in_reg[0], data_in};
wire data_in_synced = data_in_reg[1];

//recieve data
always@(posedge clk_50)
begin
	if(reset) 
	begin
		count <= 0;
		data <= 0;
	end
	else if(spi_clk_r)
	begin
		count <= count + 1'b1;
		data <= {data[14:0], data_in_synced};
	end
end

always@(posedge clk_50)
begin
	write_pulse <= spi_clk_r && (count == 15);
	if(reset == 1'b1)
	begin
		bytes_recieved <= 0;
		//data <= 0;
		//byte_out <= 0;
		address <= 0;
		//count <= 0;
	end
	else if(spi_clk_r && (count == 15))
	begin
		address <= address + 1'b1;
		bytes_recieved <= bytes_recieved + 2;
	end
end


//data clocked in on posedge
/*always@(posedge clk or posedge reset)
begin
	if(reset == 1'b1) 
	begin
		bytes_recieved <= 0;
		data <= 0;
		//byte_out <= 0;
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

assign byte_out[15:1] = data[15:1];
assign byte_out[0] = data_in;

always@(posedge clk_50)
begin

if(count == 15) write_pulse <= 1'b1;
else write_pulse <= 1'b0;
end*/
endmodule
