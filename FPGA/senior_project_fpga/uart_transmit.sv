module uart_transmit(
	input clk,
	input start,
	input [11:0][7:0]cmd_buf, //12 bytes (6 words)
	
	output logic data_out
);

logic [3:0]state;
logic [7:0]data_shift;
logic [3:0]counter;
logic [3:0]bytes_out;
logic t_byte_flag; //twelve byte flag

parameter idle = 0, shift = 1, stop1 = 2;

always@(posedge clk)
begin
	case(state)
		idle:
		begin
			if((start == 1) || (t_byte_flag == 1)) 
			begin
				t_byte_flag <= 1;
				
				data_out <= 0;
				data_shift <= cmd_buf[bytes_out];
				state <= shift;
			end
			else data_out <= 1;
		end
		
		shift:
		begin
			if(counter == 8) 
				begin
				data_out <= 1;
				counter <= 0;
				state <= stop1;
				end
			else
				begin
				data_out <= data_shift[counter];
				counter <= counter + 1;
				state <= shift;
				end
		end
			
		stop1:
		begin
			data_out <= 1;
			
			if(bytes_out == 11) 
			begin
				bytes_out <= 0;
				t_byte_flag <= 0;
			end
			else bytes_out <= bytes_out + 1;
			
			state <= idle;
		end
		endcase
end

endmodule
