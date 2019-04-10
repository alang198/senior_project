module ide_processor(
	input clk,
	input [1:0]cs,
	input [2:0]da,
	input rd,
	input wr,
	input reset,
	inout [15:0]data_pins,
	
	output logic intrq,
	
	input dmack,
	output logic dmarq,
	
	output logic iordy,
	
	output logic uart_start,
	output logic [11:0][7:0]cmd_buf, //12 bytes (6 words)
	
	input [15:0]bytes_in,
	input [15:0]word_in,
	output logic buffer_read,
	output logic reset_bytes_transmitted,
	output logic [15:0]read_addr
);

//In datasheet DIOR & DIOW refer to RD and WR
//LSB used for 8 bit transfer (used for control stuff)
//Data command uses 16 bit
//"asserted" in datasheet = 0 (most of the time)
//data becomes valid on falling edge, is latched on rising (RD&WR)
//two modes (pio (8 or 16 bit) & dma (always 16 bit))

assign selected = drive_sel[4];

//logic [15:0]data_in; //let's work with the in/out as little as possible
logic [15:0]data_out = 16'bz;

logic dout_en;
//lifted almost directly from ice; want something that confirms works for tristate
assign dout_en = ~read_cur & ((~cs[0])|((~cs[1])&(da[2:1]==2'b11))|~dmack) & ~drive_sel[4]; 
assign data_pins = ((dout_en == 1'b1)) ? data_out : 16'bz; //tristate buffer
//assign data_in = data_pins;

reg int_rq = 1'b0; //logic controller for int_rq
assign intrq = ((drive_sel[4] == 1'b0) && (dev_control[1] == 1'b0)) ? int_rq : 1'bz;
assign iordy = 1'bz; //always high impedance

logic dma_en; 
logic dma_flag; //0 = pio/off; 1 = DMA
logic dma_rq; //controls dmarq level

assign dma_en = dma_flag & ~drive_sel[4]; //controls if datalines are asserted or not
assign dmarq = (dma_en) ? dma_rq : 1'bz;


//8 bit registers
//bit field info from SPI datasheet
wire [15:0]status_alt;
reg [15:0]error_reg; //the completetion status of the most recent command
reg [15:0]int_reason; //determines the state of recievablity
reg [15:0]sector_num; //stores disc format & status (always store 8'h8x since 8 in upper half denotes GD-ROM) (disc format == 0 on drive open)
reg [15:0]byte_count; //expected data length of commands such as CD_READ is stored here

reg [15:0]drive_sel; //an optional register that goes unused (drvhead used though)
reg [15:0]status;
//status_alt: same as status only DMA status info does not get cleared upon access
//error_reg: sense key, media change (eject), drive not ready (and last command was invalidated), first two bits seem ignorable; sense key definitions on page 19
//int_reason: also checks with status; check truth table on page 17 for more details
//status: busy, ready, drive fault, seek processing, data prep done, error correctable, check error
reg [15:0]features; //bit 0 determines if we're in PIO (0) or DMA mode (1); usually set before a CD_READ operation or something similar
reg [15:0]sector_count; //write only; refer to truth table on page 17

//read cycle refers to transmitting to DC
//write cycle refers to reading from DC (opposite of what you expect)

reg [7:0]command; //stores IDE command recieved from DC
reg [7:0]dev_control; //this one appears to be write only
reg wflag;
reg rflag;

reg [31:0]debug_count;

reg [31:0]state;
reg cmd_done;

//state machine parameters
parameter ide_wait = 0, process_command = 1, recieve_spi = 2, transmit_spi = 4, dma_transfer = 8,
dma_transfer_last = 16, recieve_data = 32, do_test_unit = 64, do_set_mode = 128, pio_transfer_setup = 256,
send_to_dc = 512, pio_transfer = 1024, pio_transfer_last = 2048, set_mode_recieve = 4096, do_req_mode = 8192,
transmit_req_mode_rev = 16384, transmit_zeros = 32768, do_cmd71 = 65536, do_cmd71_transmit = 131072,
do_req_error = 262144, recieve_data_dma = 524288, recieve_data_dma1 = 1048576;

//reg [15:0]buffer_count;
reg [15:0]count_to;

//wire reset_cmd = 1'b1;

assign status_alt = status;

reg [15:0]bytes_transfered;
reg [7:0]spi_cmd; //store SPI command


logic read_pulse;
logic read_old;
logic read_cur;

//update buffer pulse
//logic read_pulse_dma;
//logic read_dma_old;
//logic read_dma_cur;
//assign read_pulse_dma = ~read_cur & read_old;

//cd_read stuff
logic [15:0]sectors_remaining;
logic [15:0]dma_bytes_sent;
logic [15:0]dma_bytes_to_send;
logic [7:0]dma_transfers_done;

//assign read_cur = rd;

//generate read pulse
assign read_pulse = read_old & ~read_cur; //rising edge confirmed 
always_ff@(posedge clk)
begin
	read_old <= read_cur;
	read_cur <= rd;
end

logic write_pulse;
logic write_old;
logic write_cur;

assign write_pulse = ~write_old & write_cur;
always_ff@(posedge clk)
begin
	write_old <= write_cur;
	write_cur <= wr;
end


logic [15:0]buffer_data_out;

always_ff@(posedge clk)
begin

	//if(drive_sel[4] == 1'b1) state <= error; //this should never happen

	//sending data to Dreamcast
	//look up table on page 12 of SPI format datasheet

	if(~dmack) data_out <= buffer_data_out;
	else
	begin
		case({cs[0], cs[1], da})
			//these don't appear to do anything
			//x in verilog = don't care, simple stuff
			//5'b11xxx: data_out <= 16'bz;
			//5'b100xx: data_out <= 16'bz;
			//5'b1010x: data_out <= 16'bz;
			//these do things
			5'b10110: data_out <= status_alt;
			5'b01000: data_out <= buffer_data_out;
			5'b01001: data_out <= error_reg;
			5'b01010: data_out <= int_reason;
			5'b01011: data_out <= sector_num;
			5'b01100: data_out <= {8'h00, byte_count[7:0]};
			5'b01101: data_out <= {8'h00, byte_count[15:8]};
			5'b01110: data_out <= drive_sel;
			5'b01111: 
			begin
				//status[7] <= 1'b0;
				if(drive_sel[4] == 0) int_rq <= 1'b0; //reading status clears this; alt status does not
				data_out <= status;
			end
			default: data_out <= 0;
			//there will never be a case where both CS lines are asserted.
		endcase
	end

	
	
	//latching data from Dreamcast
	//check if we're on the falling edge
	if(write_pulse) //rising edge confirmed
	begin
		
		//look up table page 12 SPI datasheet
		case({cs[0], cs[1], da})
			5'b10110: dev_control[2:1] <= data_pins[2:1]; //neid and sreset
			
			//according to the datasheet this is unused; however, according to ice it's used to set sector count
			5'b01010: int_reason <= {8'h00, data_pins[7:0]}; 
			
			5'b01001: features <= {8'h00, data_pins[7:0]};
			5'b01100: byte_count[7:0] <= data_pins[7:0];
			5'b01101: byte_count[15:8] <= data_pins[7:0];
			5'b01110: drive_sel <= {8'h00, data_pins[7:0]};
			5'b01111: 
			begin
				state <= process_command;
				command <= data_pins[7:0];
				status[7] <= 1'b1; //set busy
			end
		endcase
	end
	
	
	//reset might need a seperate module to prolong the effects; stay tuned
	if(reset)
	begin
		//init the control regs
		error_reg <= 16'h0000;
		features <= 16'h0000;
		sector_count <= 16'h0000;
		sector_num <= 16'h0081; //find out what values can go here; assume GD. REQ_STAT gets it's data from here
		drive_sel <= 16'h0000;
		int_reason <= 16'h0000;
		status <= 16'h0050;
		data_out <= 16'h0000;
		state <= ide_wait;
	end

	case(state)
	
	//process commands to Dreamcast (PIO can be read or write, DMA only read)
	//set DRQ (in status); assert INTRQ after clearing busy bit	
	//INTRQ assert means 1
	//ice uses seccnt to refer to interupt reason for SOME REASON
	ide_wait:
	begin
		reset_bytes_transmitted <= 0;
	end
	
	process_command:
	begin
	
		case(command)
			8'hA0:
			begin
				//SPI command
				int_reason[1:0] <= 2'b01; //clear IO and set CoD
				
				state <= recieve_spi;
				
				count_to <= 5; //spi command is 6 words
				status <= 8'h58; //set DRQ, seek processing & ready
				write_buffer_count <= 0;
				cmd_done <= 1'b0;
				addr_count <= 0;
				
			end
			
			//set features (just need to set error and status then raise interrupt)
			8'hEF:
			begin
				error_reg <= 0;
				status <= 8'h50;
				int_rq <= 1'b1;
				
				state <= ide_wait;
			end
			
			//soft reset
			8'h08:
			begin
				
				status <= 8'h50;
				dma_rq <= 0;
				int_rq <= 0;
				features <= 0;
				error_reg <= 0;
				int_reason <= 0;
				
				reset_bytes_transmitted <= 0;
				bytes_transfered <= 0;
				state <= ide_wait;
			end
			
			default:
			begin
				error_reg <= 8'h04;
				status <= 8'h51;
				int_rq <= 1'b1;
				
				state <= ide_wait;
			end
		endcase
	end
	
	recieve_spi:
	begin
	
		if(cmd_done == 1)
		begin
			status[7] <= 1'b1; //set busy
			status[3] <= 1'b0; //clear DRQ
			spi_cmd <= cmd_buf[0]; //get SPI command
			
			state <= transmit_spi;
			
			cmd_done <= 0;
			reset_bytes_transmitted <= 1;
		end
		else state <= recieve_spi;
	end
	
	transmit_spi:
	begin
		//uart_start <= 1'b1;
		reset_bytes_transmitted <= 0;
		
		//check if we need to manually set byte_count (e.g. cmd 71) or go to unique state
		if(spi_cmd == 8'h71) 
		begin
			byte_count <= 6;
			state <= do_cmd71;
		end
		else if(spi_cmd == 8'h12) 
		begin
			state <= do_set_mode; //set mode
			byte_count <= 10;
		end
		else if(spi_cmd == 8'h30)
		begin
			//byte_count <= cmd_buf[10] << 11;
			sectors_remaining <= (cmd_buf[9] << 8) | cmd_buf[10];
			state <= recieve_data_dma;
			uart_start <= 1'b1;
		end
		else if(spi_cmd == 8'h13)  //req_error
		begin
			state <= do_req_error;
			int_rq <= 1'b1;
			buffer_data_out <= 0;
			bytes_transfered <= 0;
		end
		else if(spi_cmd == 8'h14) byte_count <= 408; //get_toc
		else if(spi_cmd == 8'h15) byte_count <= 6; //req_ses
		else if((spi_cmd == 8'h70) || (spi_cmd == 8'h00)) state <= do_test_unit; //test unit
		else if(spi_cmd == 8'h11) state <= do_req_mode; //req_mode (need to check transfer length in next state)
		else if(spi_cmd == 8'h11) byte_count <= 10; //req_mode
		else if(spi_cmd == 8'h13) byte_count <= 10; //req_error
		else if((spi_cmd == 8'h40) && (cmd_buf[1] != 8'h00)) byte_count <= 14; //cd_scd
		
		if((spi_cmd != 8'h12) && (spi_cmd != 8'h70) && (spi_cmd != 8'h00) && (spi_cmd != 8'h11) && (spi_cmd != 8'h71) && (spi_cmd != 8'h13) && (spi_cmd != 8'h30)) 
		begin
			
			if(debug_count == 50000000)
			begin
				state <= recieve_data;
				uart_start <= 1'b1;
			end
			else debug_count <= debug_count + 1'b1;
		end
	end
	
	do_req_error:
	begin
		int_reason <= 8'h02;
		byte_count <= 10;
		status <= 8'h58;

		if(bytes_transfered >= byte_count) state <= pio_transfer_last;
		if((read_pulse == 1'b1) && ({cs[0], cs[1], da} == 5'b01000) && ~drive_sel[4]) bytes_transfered <= bytes_transfered + 2;
	end
	
	do_cmd71:
	begin
		int_reason <= 8'h02; //I/O set, CoD clear
		status <= 8'h58;
		
		int_rq <= 1'b1;
		
		bytes_transfered <= 0;
		
		buffer_data_out <= 16'h06BA; //cmd71 pt 1
		
		state <= do_cmd71_transmit;
	end
	
	do_cmd71_transmit:
	begin
		if(bytes_transfered == 6) state <= pio_transfer_last;
		
		else if((read_pulse == 1'b1) && ({cs[0], cs[1], da} == 5'b01000) && ~drive_sel[4])
		begin
			if(bytes_transfered == 2) buffer_data_out <= 16'hCA0D;
			else if(bytes_transfered == 4) buffer_data_out <= 16'h1F6A;
			
			bytes_transfered <= bytes_transfered + 2;
			
			
			state <= do_cmd71_transmit;
		end
	end
	
	recieve_data:
	begin
		uart_start <= 1'b0;
		//we've recieved the amount we expect
		if((bytes_in >= byte_count)) state <= send_to_dc;
	end
	
	recieve_data_dma:
	begin
		uart_start <= 1'b0;
		reset_bytes_transmitted <= 1'b0;
		dma_bytes_sent <= 0;
		
		if(sectors_remaining == 0) state <= pio_transfer_last;
		else if(sectors_remaining > 15) //send 15 sectors
		begin
			sectors_remaining <= sectors_remaining - 15;
			dma_bytes_to_send <= 30720;
			state <= recieve_data_dma1;
		end
		
		else //send however many sectors are left
		begin
			sectors_remaining <= 0;
			dma_bytes_to_send <= sectors_remaining << 11;
			state <= recieve_data_dma1;
		end
		
	end
	
	recieve_data_dma1:
	begin
		if(bytes_in >= dma_bytes_to_send) state <= send_to_dc;
	end
	
	send_to_dc:
	begin
		//check if PIO or DMA transfer
		if(features[0] == 1'b1) 
		begin
			state <= dma_transfer; //DMA
			buffer_read <= 1;
			read_addr <= 1;
		end
		else state <= pio_transfer_setup;
		
		//buffer_read <= 1'b1;
		bytes_transfered <= 0;
	end
	
	dma_transfer:
	begin
		dma_flag <= 1'b1; //enable DMA
		dma_rq <= 1'b1; //set dmarq level to high to start DMA transfer
		
		if(buffer_read == 1'b1)
		begin
			//buffer_read <= 0;
			buffer_data_out[15:8] <= word_in[7:0];
			buffer_data_out[7:0] <= word_in[15:8];
		end
		
		//last word
		if(dma_bytes_sent >= dma_bytes_to_send)
		begin
			state <= dma_transfer_last;
			
			dma_rq <= 1'b0; //negate DMARQ
			//dma_flag <= 1'b0; //disable DMA
		end
		
		//update value from buffer
		else if((read_pulse) && (~dmack)) 
		begin
			//buffer_read <= 1'b1;
			dma_bytes_sent <= dma_bytes_sent + 2;
			
			read_addr <= read_addr + 1'b1;
		end
		
		
	end
	
	dma_transfer_last:
	begin
		buffer_read <= 0;
		if(sectors_remaining != 0) uart_start <= 1'b1;
		reset_bytes_transmitted <= 1'b1;
		
		state <= recieve_data_dma;
		sector_num <= 8'h82;
	end
	
	do_test_unit:
	begin
		error_reg <= 0;
		int_reason <= 3;
		status <= 8'h50;
		
		int_rq <= 1'b1;
		
		state <= ide_wait;
	end
	
	pio_transfer_setup:
	begin
		int_reason <= 8'h02; //I/O set, CoD clear
		status <= 8'h58;
		
		int_rq <= 1'b1;
		read_addr <= 1;
		
		buffer_read <= 1'b1;
		bytes_transfered <= 0;
		reset_bytes_transmitted <= 1;
		state <= pio_transfer;
	end
	
	pio_transfer:
	begin
		reset_bytes_transmitted <= 0;
	
		if(buffer_read == 1'b1)
		begin
			//buffer_read <= 1'b0;
			buffer_data_out[15:8] <= word_in[7:0];
			buffer_data_out[7:0] <= word_in[15:8];
		end
	
		if((bytes_transfered >= byte_count))
		begin
			state <= pio_transfer_last;
		end
		
	
		if((read_pulse == 1'b1) && ({cs[0], cs[1], da} == 5'b01000) && ~drive_sel[4])
		begin
			bytes_transfered <= bytes_transfered + 2;
			buffer_read <= 1'b1;
			
			if(read_addr == 32767) read_addr <= 0;
			else read_addr <= read_addr + 1'b1;
		end
	end
	
	pio_transfer_last:
	begin
		status <= 8'h50;
		error_reg <= 0;
		int_reason <= 8'h03;
		
		int_rq <= 1'b1;
		buffer_read <= 0;
		
		state <= ide_wait;
	end
	
	do_set_mode:
	begin
		int_reason <= 0;
		status <= 8'h58;
		
		int_rq <= 1'b1;
		
		count_to <= 4;
		state <= set_mode_recieve;
	end
	
	set_mode_recieve:
	begin
		
		if(cmd_done == 1'b1)
		begin
			status <= 8'h50;
			int_reason <= 8'h03;
			error_reg <= 0;
			int_rq <= 1'b1;
			
			state <= ide_wait;
		end
	end
	
	/*CHECK FIRST WORD*/
	do_req_mode:
	begin
		int_reason <= 8'h02; //I/O set, CoD clear
		status <= 8'h58;
		
		int_rq <= 1'b1;
		
		bytes_transfered <= 0;
		
		//send "REV 5.07"
		if(cmd_buf[2] == 8'h12)
		begin
			byte_count <= 8;
			state <= transmit_req_mode_rev;
			
			buffer_data_out <= 16'h6552; //"Re"
		end
		//send zeros
		else
		begin
			byte_count <= 10;
			state <= transmit_zeros;
			buffer_data_out <= 0;
		end
	end
	
	transmit_req_mode_rev:
	begin
		if(bytes_transfered == 8) state <= pio_transfer_last;
		
		if((read_pulse == 1'b1) && ({cs[0], cs[1], da} == 5'b01000) && ~drive_sel[4])
		begin
			if(bytes_transfered == 2) buffer_data_out <= 16'h2076; //"v "
			else if(bytes_transfered == 4) buffer_data_out <= 16'h2E35; //"5."
			else if(bytes_transfered == 8) buffer_data_out <= 16'h3730; //"07"
			
			bytes_transfered <= bytes_transfered + 2;
			
			
			state <= transmit_req_mode_rev;
		end
	end
	
	transmit_zeros:
	begin
		buffer_data_out <= 0;
		data_out <= 0;
	
		if(bytes_transfered >= byte_count) state <= pio_transfer_last;
		
		else if((read_pulse == 1'b1) && ({cs[0], cs[1], da} == 5'b01000) && ~drive_sel[4])
		begin
			bytes_transfered <= bytes_transfered + 2;
			state <= transmit_zeros;
		end
	end
	
	
	default:
	begin
		status <= status;
		//state <= ide_wait;
	end
	
	endcase


	//recieve SPI
	if(({cs[0], cs[1], da} == 5'b01000) && write_pulse && (drive_sel[4] == 0))
	begin
	
		write_buffer_count <= write_buffer_count + 1'b1;
		
		addr_count <= addr_count + 2;
	
		cmd_buf[addr_count] <= data_pins[7:0];
		cmd_buf[addr_count + 1] <= data_pins[15:8];
		
		if((write_buffer_count == count_to))
		begin
			write_buffer_count <= 0;
			cmd_done <= 1;
		end
	end
	
end

logic debug_flag;
logic [7:0]addr_count;
logic [7:0]write_buffer_count;
logic wr_flag;


endmodule
