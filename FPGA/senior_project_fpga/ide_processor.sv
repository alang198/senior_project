module ide_processor(
	input clk,
	input [1:0]cs,
	input [2:0]da,
	input rd,
	input wr,
	input reset,
	inout [15:0]data_pins,
	
	output logic intrq,
	output logic dmack,
	input logic dmarq,
	output logic iordy,
	
	output logic uart_start,
	input logic [15:0]bytes_in
);

//In datasheet DIOR & DIOW refer to RD and WR
//LSB used for 8 bit transfer (used for control stuff)
//Data command uses 16 bit
//"asserted" in datasheet = 0
//data becomes valid on falling edge, is latched on rising (RD&WR)
//two modes (pio (8 or 16 bit) & dma (always 16 bit))
//PIO will ALWAYS be 16 bit, the DC does not support 8bit PIO
logic [15:0]data_in; //let's work with the complicated in/out as little as possible
logic [15:0]data_out;

assign data_pins = ((rd == 1'b0)) ? data_out : 16'bz; //tristate buffer
assign intrq = (dev_control[1] == 1'b1) ? int_rq : 1'bz;
assign iordy = 1'bz; //always high impedance

//all these registers are normally 8bit;
//however, for convienence we're going to make them 16
//bit field info from SPI datasheet
//field goes from bit 7 to 0
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
reg dma_flag; //0 = pio; 1 = DMA
reg int_rq;
reg [15:0]state;
reg cmd_done;
parameter ide_wait = 0, process_command = 1, recieve_spi = 2, transmit_spi = 4;

reg [15:0]buffer_count;
reg [15:0]count_to;

wire reset_cmd = 1'b1;

assign status_alt = status;

reg [15:0]data; //data that goes out

reg [15:0]bytes_transferred;

always_ff@(posedge clk)
begin

	//sending data to Dreamcast
	//look up table on page 12 of SPI format datasheet
	if(rd == 1'b0) 
	begin
		//status[7] <= 1'b0;
		case({cs, da})
			//these don't appear to do anything
			//x in verilog = don't care, simple stuff
			5'b11xxx: data_out <= 16'bz;
			5'b100xx: data_out <= 16'bz;
			5'b1010x: data_out <= 16'bz;
			//these do things
			5'b10110: data_out <= status_alt;
			5'b01001: data_out <= error_reg;
			5'b01010: data_out <= int_reason;
			5'b01011: data_out <= sector_num;
			5'b01100: data_out <= {8'h00, byte_count[7:0]};
			5'b01101: data_out <= {8'h00, byte_count[15:8]};
			5'b01110: data_out <= drive_sel;
			5'b01111: 
			begin
				status[7] <= 1'b0;
				int_rq <= 1'b0; //reading status clears this; alt status does not
				data_out <= status;
			end
			//there will never be a case where both CS lines are asserted.
		endcase
	end

	case(state)
	
	ide_wait:
	begin
	
		//reset might need a seperate module to prolong the effects; stay tuned
		if(reset)
		begin
			//init the control regs
			//datasheet gives different values than iceGDROM, let's go with iceGDROM values
			//as the datasheet values probably get set to the iceGDROM values at a later point (realistically speaking)
			//status_alt <= 16'h0080;
			error_reg <= 16'h0000;
			features <= 16'h0000;
			sector_count <= 16'h0000;
			sector_num <= 16'h0000; //find out what values can go here; assume GD. REQ_STAT gets it's data from here
			drive_sel <= 16'h0000;
			int_reason <= 16'h0003;
			//status <= 16'h0080;
		end
			
			
		//latching data from Dreamcast
		//check if we're on the falling edge
		if(wr == 1'b0) wflag <= 1'b1;
		else if((wr == 1'b1) && (wflag == 1'b1)) //rising edge confirmed
		begin
			wflag <= 1'b0;
			
			//look up table page 12 SPI datasheet
			case({cs, da})
				5'b10110: dev_control[2:1] <= data_in[2:1]; //neid and sreset
				
				//according to the datasheet this is unused; however, according to ice it's used to set sector count
				5'b01010: sector_count <= {8'h00, data_in[7:0]}; 
				
				5'b01001: features <= {8'h00, data_in[7:0]};
				5'b01100: byte_count[7:0] <= data_in[7:0];
				5'b01101: byte_count[15:8] <= data_in[7:0];
				5'b01110: drive_sel <= {8'h00, data_in[7:0]};
				5'b01111: 
				begin
					state <= process_command;
					command <= data_in[7:0];
					status[7] <= 1'b1; //set busy
				end
			endcase
			else status <= 16'h0040;
			
		end
	end

	//process commands to Dreamcast (PIO can be read or write, DMA only read)
	//set DRQ (in status); assert INTRQ after clearing busy bit	
	//INTRQ assert means 1
	//ice uses seccnt to refer to interupt reason for SOME REASON; since I'm not stupid I won't be doing that
	//PIO out only used for cmd 71; set status to 8'h58
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
			end
			8'hEF:
			begin
				//set features
				if(
			end
		endcase
	end
	
	recieve_spi:
	begin
	
		if(cmd_done == 1)
		begin
			status[7] <= 1'b1; //set busy
			status[3] <= 1'b0; //clear DRQ
			state <= process_spi;
		end
	end
	
	transmit_spi:
	begin
		uart_start <= 1'b1;
		
		//check if we need to manually set byte_count (e.g. cmd 71)
	end
	
	recieve_data:
	begin
		//we've recieved the amount we expect or one sector
		if((bytes_recieved == byte_count) || (bytes_recieved == 2048)) state <= send_to_dc;
	end
	
	send_to_dc:
	begin
		//check if PIO or DMA transfer
		
	end
	
	endcase
end

//recieve SPI
always_ff@(wr)
begin
	
	if(cmd_done == 1'b1) cmd_done <= 0;
	
			
	if(({cs, da} == 5'b01000) && (wr == 1'b1))
	begin
		//data buffer[pos] <= data_in;
		
		if(buffer_count >= count_to)
		begin
			buffer_count <= 0;
			cmd_done <= 1;
		end
		else buffer_count <= buffer_count + 1;
	end
end
endmodule