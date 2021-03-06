
//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================

module senior_project_fpga(
//////////// CLOCK //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// SW //////////
	SW,

	//////////// SDRAM //////////
	/*DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_DQM,
	DRAM_RAS_N,
	DRAM_WE_N,*/

	//////////// EPCS //////////
	/*EPCS_ASDO,
	EPCS_DATA0,
	EPCS_DCLK,
	EPCS_NCSO,*/

	//////////// Accelerometer and EEPROM //////////
	/*G_SENSOR_CS_N,
	G_SENSOR_INT,
	I2C_SCLK,
	I2C_SDAT,*/

	//////////// ADC //////////
	/*ADC_CS_N,
	ADC_SADDR,
	ADC_SCLK,
	ADC_SDAT,*/

	//////////// 2x13 GPIO Header //////////
	GPIO_2,
	GPIO_2_IN,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	GPIO_0,
	GPIO_0_IN,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	GPIO_1,
	GPIO_1_IN 
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input 		          		CLOCK_50;

//////////// LED //////////
output		     [7:0]		LED;

//////////// KEY //////////
input 		     [1:0]		KEY;

//////////// SW //////////
input 		     [3:0]		SW;

//////////// SDRAM ////////// 
/*output		    [12:0]		DRAM_ADDR;
output		     [1:0]		DRAM_BA;
output		          		DRAM_CAS_N;
output		          		DRAM_CKE;
output		          		DRAM_CLK;
output		          		DRAM_CS_N;
inout 		    [15:0]		DRAM_DQ;
output		     [1:0]		DRAM_DQM;
output		          		DRAM_RAS_N;
output		          		DRAM_WE_N;*/

//////////// EPCS //////////
/*output		          		EPCS_ASDO;
input 		          		EPCS_DATA0;
output		          		EPCS_DCLK;
output		          		EPCS_NCSO;*/

//////////// Accelerometer and EEPROM //////////
/*output		          		G_SENSOR_CS_N;
input 		          		G_SENSOR_INT;
output		          		I2C_SCLK;
inout 		          		I2C_SDAT;*/

//////////// ADC //////////
/*output		          		ADC_CS_N;
output		          		ADC_SADDR;
output		          		ADC_SCLK;
input 		          		ADC_SDAT;*/

//////////// 2x13 GPIO Header //////////
inout 		    [12:0]		GPIO_2;
input 		     [2:0]		GPIO_2_IN;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_0;
input 		     [1:0]		GPIO_0_IN;

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_1;
input 		     [1:0]		GPIO_1_IN;



//=======================================================
//  REG/WIRE declarations
//=======================================================
wire reset; //reset input
wire [15:0]D; //data lines
wire DMARQ; //dma request
wire CD_EMPH; //CD empty status output
wire RD; //read select
wire DMACK; //dma acknowledge
wire [2:0]DA; //select stuff for ATA bus
wire [1:0]CS; //chip select
wire CD_BCLK; //one of the CD clocks (1.41MHz)
wire CD_SD; //CD serial data in
wire DOOR_OPEN; //status for disc lid (1 = open)
wire WR; //write select
wire IORDY; //this could be the busy line (accoding to dc.gif this is an input (to dc) so that assumption seems reasonable)
wire INTRQ; //interrupt request
wire CD_LRCLK; //another CD clock (44.1KHz)
wire CDDA_CLK; //YET another CD clock (33.8688MHz)


//=======================================================
//  Structural coding
//=======================================================

//A side of the bus, lowest to highest
assign reset = GPIO_0[0];

//assign GPIO_0[2] = D[7];
//assign GPIO_0[4] = D[8];
//assign GPIO_0[6] = D[5];
//assign GPIO_0[8] = D[10];

//assign GPIO_0[10] = D[3];
//assign GPIO_0[12] = D[12];
//assign GPIO_0[14] = D[1];
//assign GPIO_0[16] = D[14];

assign GPIO_0[18] = DMARQ;
assign RD = GPIO_0[20];
assign DMACK = GPIO_0[22];
assign GPIO_0[23] = CD_EMPH;

assign DA[0] = GPIO_0[26];
assign CS[0] = GPIO_0[28];


assign GPIO_0[31] = CD_SD;

//B side of the bus
assign GPIO_0[33] = DOOR_OPEN;

//assign GPIO_0[1] = D[6];
//assign GPIO_0[3] = D[9];
//assign GPIO_0[5] = D[4];
//assign GPIO_0[7] = D[11];

//assign GPIO_0[9] = D[2];
//assign GPIO_0[11] = D[13];
//assign GPIO_0[13] = D[0];
//assign GPIO_0[15] = D[15];

assign WR = GPIO_0[17];
assign GPIO_0[19] = IORDY;
assign GPIO_0[21] = INTRQ;
assign DA[1] = GPIO_0[24];

assign DA[2] = GPIO_0[25];
assign CS[1] = GPIO_0[27];

assign CD_SD = 0;
assign CD_EMPH = 0;
assign DOOR_OPEN = 0;

//this one is too fast for a clock divider; needs PLL
cdda_clk sonic_d_hedgehog(.inclk0(CLOCK_50), .c0(CDDA_CLK));
cd_bclk sonic_b_hedgehog(.inclk0(CLOCK_50), .c0(CD_BCLK));
cd_lrclk sonic_e_hedgehog(.inclk0(CLOCK_50), .c0(CD_LRCLK));
//using a PLL for all of them gets more accurate results in the end

reg reset_flag = 1'b0;
//reg reset_help = 1'b0;
//reg [31:0]reset_count;
////ensure we've seen a reset (i.e. power has been applied before driving clocks)
//always@(posedge CD_LRCLK)
//begin
//	
//	if(reset_count < 1000000) reset_count <= reset_count + 1'b1;
//	else reset_help <= 1'b1;
//end
//
//always@(posedge CD_LRCLK)
//begin
//	if(reset == 1'b0) 
//	begin
//		reset_flag <= 1'b1;
//		LED[7] <= 1'b1;
//	end
//end

//assign reset = 0;

assign LED[0] = reset;
assign LED[1] = GPIO_0[17];

//assign GPIO_0[30] = (reset_flag) ? CD_BCLK : 1'bz;
//assign GPIO_0[29] = (reset_flag) ? CD_LRCLK : 1'bz;
//assign GPIO_0[32] = (reset_flag) ? CDDA_CLK : 1'bz;

//assign GPIO_2[0] = CD_BCLK;
//assign GPIO_0[29] = CD_LRCLK;
assign GPIO_0[32] = CDDA_CLK;

//System Wire Declarations//
wire uart_start; //ide_processor -> uart_transmit
wire [11:0][7:0]cmd_buf; //ide_processor -> uart_transmit
wire [15:0]spi_bytes_in; //spi_reciever -> ide_processor
wire [14:0]mem_address; //memory_buffer -> ide_processor
wire mem_buf_read; //ide_processor -> memory_buffer
wire spi_reset_count; //ide_processor -> spi_reciever

wire uart_clk; //clk_divide -> uart_transmit

wire [15:0]spi_to_buf; //spi_recieve -> memory_buffer
wire [7:0]spi_bit_count; //spi_recieve -> memory_buffer

wire [15:0]buffer_word_out;
wire write_pulse;

wire [14:0]spi_address;
wire [14:0]ide_address;

wire uart_start_synced;
//************************//

assign GPIO_2[0] = 1'bz;

ide_processor anime_girls_in_glasses(.clk(CLOCK_50), .cs(CS), .da(DA), .rd(RD), .wr(WR), .reset(~reset), 
.data_pins({GPIO_0[15], GPIO_0[16], GPIO_0[11], GPIO_0[12], GPIO_0[7], GPIO_0[8], GPIO_0[3], GPIO_0[4], 
GPIO_0[2], GPIO_0[1], GPIO_0[6], GPIO_0[5], GPIO_0[10], GPIO_0[9], GPIO_0[14], GPIO_0[13]}), 
.intrq(INTRQ), .dmack(DMACK), .dmarq(DMARQ), .iordy(IORDY), .uart_start(uart_start),
.cmd_buf(cmd_buf), .bytes_in(spi_bytes_in), .word_in(buffer_word_out), .buffer_read(mem_buf_read),
.reset_bytes_transmitted(spi_reset_count), .read_addr(ide_address));

clk_divide za_warudo(.clk(CLOCK_50), .clk_out(uart_clk));

uart_transmit u_art_transmitting(.clk(uart_clk), .clk_50(CLOCK_50), .start(uart_start), .cmd_buf(cmd_buf), 
.data_out(GPIO_1[25]));

spi_recieve not_sega_packet_interface(.clk(GPIO_1[33]), .data_in(GPIO_1[29]), .bytes_recieved(spi_bytes_in), .clk_50(CLOCK_50),
.data(spi_to_buf), .address(spi_address), .reset(spi_reset_count), .write_pulse(write_pulse));

buffer_small actual_buffer(
	.address(mem_address),
	.clock(CLOCK_50),
	.data(spi_to_buf),
	.wren(write_pulse),
	.q(buffer_word_out)
	);
	
//wire clk_100;
//fast clk_twice(.inclk0(CLOCK_50), .c0(clk_100));

assign mem_address = (mem_buf_read) ? ide_address : spi_address;
	
/*memory_buffer cant_remember(.clk(CLOCK_50), .read_pulse(mem_buf_read), .spi_count(spi_bit_count),
.spi_byte_in(spi_to_buf), .address(mem_address), .write_pulse(write_pulse));*/

//test_uart test(.clk(CLOCK_50), .uart_start(uart_start), .cmd_buf(cmd_buf));

//sync n_sync(.clk_fpga(uart_clk), .clk_ps2(CLOCK_50), .flag_in(uart_start), .flag_out(uart_start_synced)); //clock domain crossing


/*mem_buf_ram actual_buffer(
	.address(mem_address),
	.clock(CLOCK_50),
	.data(spi_to_buf),
	.wren(write_pulse), //1 = write
	.q(buffer_word_out)
	);*/

endmodule
