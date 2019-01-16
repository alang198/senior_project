/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include <string.h>
#include <stdlib.h>

//Handle Types (put these in read/write APIs to serial ports)
SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;

//Config funtion prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);

//global variables
uint8_t image_num = 0;
char gdi_name[4] = ".gdi";
uint8_t cmd_process_state = 0;
unsigned int junk = 0;

//related to managing the .gdi files
uint16_t lba[21] = {0};
uint8_t data_type[21] = {0};
FIL files[21];
uint8_t file_count = 0;
uint8_t cmd_buf[11] = {0};
uint8_t data_buf[2352] = {0};
uint8_t data_buf1[2352] = {0};
uint8_t gdda_playing = 0;

FIL gdi_file;

//Read .gdi
uint8_t open_gdi(void);
void process_cmd(uint8_t);
void get_toc(void);
void req_ses(void);
void transmit_71(void);
void test_unit(void);
void request_mode(void);
void req_error(void);
void read_cd(uint8_t);
void get_scd(void);
void cd_play(void);

int main(void)
{
	//fatfs related variables
	FATFS sd_card;
	uint8_t gdi_open_result;
	char gdi_string[64] = "";
	
	//other variables
	uint8_t count = 0;
	char * tok_back;
	char token[2] = " ";
	uint8_t uart_buf = 0;
	
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	MX_SPI2_Init();
	MX_SPI1_Init();
	MX_UART4_Init();

	//mount successful
	if(f_mount(&sd_card, SD_Path, 1) == FR_OK){
		
		//scan file system
		while(1){
			gdi_open_result = open_gdi();
			if((gdi_open_result == 0) || (gdi_open_result == 1)) break;
		}
		//reached end, no cycling yet
		if(gdi_open_result == 1){
			//display "no disc"
			return 0;
		}
		
		//begin reading .gdi file
		f_gets(gdi_string, 64, &gdi_file); //get number of files
		file_count = atoi(gdi_string);
		
		count = 0; //zero count
		
		while(count < file_count){
			f_gets(gdi_string, 64, &gdi_file); //read line by line
			
			tok_back = strtok(gdi_string, token); //get file number (and discard will keep track using array position)
			tok_back = strtok(NULL, token); //get LBA (store in int array)
			lba[count] = atoi(tok_back);		  //store LBA
			tok_back = strtok(NULL, token); //get datatype (4 = data, 0 = audio) and store
			data_type[count] = atoi(tok_back);  //store datatype
			tok_back = strtok(NULL, token); //get sector size (can discard, always 2352)
			tok_back = strtok(NULL, token); //get file name, create entry in file pointer array and open
			if(f_open(&files[count], gdi_string, FA_READ) != FR_OK){
				//display "invalid gdi"
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); //turn on error LED
				return 1;
			} //don't need offset (it's always 0), go back to f_gets and repeat for the rest of the files

			
			//add 150 offset to tracks 1 & 3
			lba[0] = lba[0] + 150;
			lba[2] = lba[2] + 150;
			
			count = count + 1;
		}
	}
	//could not mount
	else{
		//display "no disc"
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //turn on error LED
		return 1;
	}


	while (1){
		//poll for UART message from FPGA
		uart_buf = 0;
		HAL_UART_Receive(&huart4, &uart_buf, 1, HAL_MAX_DELAY);
		//message recieved
		if(uart_buf != 0){
			HAL_UART_Receive(&huart4, cmd_buf, 11, HAL_MAX_DELAY); //get rest of the cmd
			process_cmd(uart_buf);
		}
	}
	

}
//this function handles the opening of the .gdi file
uint8_t open_gdi(){
	DIR directory; //empty directory variable
	FRESULT result;
	char path_to_search[16] = "";
	char path_to_change[16] = "";
	char file_exten[4];
	char image_num_ch[4];
	FILINFO fno;
	
	strcpy(path_to_search, "disc");
	
	image_num = image_num + 1;
	sprintf(image_num_ch, "%d", image_num); //convert int to string
	
	strcat(path_to_search, image_num_ch);
	
	result = f_opendir(&directory, path_to_search);
	//opened directory
	if(result == FR_OK){
		result = f_readdir(&directory, &fno); //read first directory item
		if(fno.fname[0] == 0){
			return 1; //the folder is empty
		}
		//check if file is .gdi
		else{
			file_exten[3] = 0;
			file_exten[2] = fno.fname[strlen(fno.fname) - 1];
			file_exten[1] = fno.fname[strlen(fno.fname) - 2];
			file_exten[0] = fno.fname[strlen(fno.fname) - 3];
			
			//the file is .gdi
			if(strcmp(file_exten, gdi_name) == 0){
				strcat(path_to_search, "/");
				strcat(path_to_search, fno.fname);
				f_open(&gdi_file, path_to_search, FA_READ);
				
				strcpy(path_to_change, "/disc");
				strcat(path_to_change, image_num_ch);
				f_chdir(path_to_change); //change logical directory
				
				return 0;
			}
			//the file is not .gdi, move to next folder
			else{
				return 2;
			}
		}
	}
	//did not open directory, reached end of valid folders
	else{
		image_num = 0; //go back to start
		return 1;
	}
	
}

//This function handles the command recieved from the FPGA
void process_cmd(uint8_t cmd){
	
	switch(cmd){
		//TEST_UNIT (note: actual command is 0x00, ice lists it as 0x70 for some reason though)
		case 0x00:
		case 0x70:
			test_unit();
			break;
		
		//REQ_STAT (get CD status) not used in ice (used in NullDC)
		case 0x10:
			break;
		
		//REQ_MODE (get various settings)
		case 0x11:
			request_mode();
			break;
		
		//SET_MODE (make various settings)
		//DO ON FPGA
		//case 0x12:
		//	break;
		
		//REQ_ERROR
		case 0x13:
			req_error();
			break;
		
		//GET_TOC (get ALL TOC data from disc image)
		case 0x14:
			//TOC data is located immeditately after the System ID
			//128 4 byte data elements (unused track info written as all F
			get_toc();
			break;
		
		//REQ_SES (get specified session data)
		case 0x15:
			req_ses();
			break;
		
		//CD_OPEN (open disc tray, the system is a top loader so like this would ever be used LOL) goes unused in ice
		//actually does nothing
		//case 0x16:
		//	break;
		
		//CD_PLAY (play CD, I'm assuming this refers to GDDA)
		case 0x20:
			cd_play();
			break;
		
		//CD_SEEK (seek for playback position)
		case 0x21:
			break;
		
		//CD_SCAN (perform scan) goes unused in ice and nulldc
		//case 0x22:
		//	break;
		
		//CD_READ (read the CD (stream data to FPGA))
		//CD_READ2 (read the CD (pre-read)) goes unused in ice (that seems weird, look into it more)
		//CD_READ2 acts as normal read, but places laser mechanism at specified point for quick access later
		//NOTE: standard 3 track games will load audio data through here rather than messing with the cd audio commands
		//treat as normal read
		case 0x30:
		case 0x31:
			read_cd(cmd);
			break;
		
		//CD_SCD (get subcode)
		case 0x40:
			get_scd();
			break;
		
		//The mysterious command 0x71, it seems as if nobody really knows what it does
		//just that it expects a specific byte stream in return
		//nullDC uses a 400 byte return while ice uses a 6 byte return, both work.
		case 0x71:
			transmit_71();
			break;
	}
}

//retrieve TOC from track 3 of .gdi
void get_toc(){
	
	uint16_t calc = 0;
	uint32_t toc_buf[102] = {0};
	
	//high density TOC
	if((cmd_buf[0] & 0x01) == 0x01){
		//NOTE: GET_TOC only gets 102 entires of the 128 (the rest are reserved so the system does not need them)
		//NOTE: since track 1&2 are related to the single density part so fill them with 0xF
		toc_buf[0] = 0xFFFFFFFF; //brute force, don't feel like looking up how to properly do this
		toc_buf[1] = 0xFFFFFFFF;
		
		f_lseek(&files[2], 0x114); //move to position 0x114 in the file "track3.bin"
		f_read(&files[2], &toc_buf[2], 408, &junk); //get that data
		memcpy(&data_buf, &toc_buf, 408);
		HAL_SPI_Transmit(&hspi1, data_buf, 408, HAL_MAX_DELAY); //transmit over SPI
	}
	//low density TOC
	else{
		//this one isn't so convienient, gotta convert LBAs to FADs
		//First 32bit word & first 16 bit half always the same
		data_buf[0] = 0x41;
		data_buf[1] = 0x00;
		data_buf[2] = 0x00;
		data_buf[3] = 0x96;
		
		data_buf[4] = 0x01;
		data_buf[5] = 0x00;
		//find second half
		calc = lba[1] + 150;
		data_buf[6] = (calc >> 8); //MSB
		data_buf[7] = (calc & 0xFF); //LSB
		
		//set next 388 bytes to 0xFF
		memset(&data_buf[8], 0xFF, 388);
		
		//Lead out stuff; first two.5 32bit words are the same
		data_buf[396] = 0x41;
		data_buf[397] = 0x01;
		data_buf[398] = 0x00;
		data_buf[399] = 0x00;
		
		data_buf[400] = 0x01;
		data_buf[401] = 0x02;
		data_buf[402] = 0x00;
		data_buf[403] = 0x00;
		
		data_buf[404] = 0x01;
		data_buf[405] = 0x00;
		//find second half
		calc = lba[1] + 676;
		data_buf[406] = (calc >> 8);
		data_buf[407] = (calc & 0xFF);
		
		HAL_SPI_Transmit(&hspi1, data_buf, 408, HAL_MAX_DELAY);
	}
}

//get the session information
void req_ses(){
	
	//not incorperating gdda yet, so write 0x02 to status half (half = half byte)
	if(gdda_playing == 0) data_buf[0] = 0x02; //disc status = standby
	else data_buf[0] = 0x03; //cd playback going
	
	data_buf[1] = 0x00; //field is empty
	
	//all discs have only two sessions (data&audio)
	//return session number 1
	if(cmd_buf[1] == 0){
		data_buf[2] = 0x01;
		data_buf[3] = 0x08;
		data_buf[4] = 0x61;
		data_buf[5] = 0xB4;
	}
	//return session number 2
	else{
		data_buf[2] = 0x01;
		data_buf[3] = 0x00;
		data_buf[4] = 0x00;
		data_buf[5] = 0x96;
	}
	HAL_SPI_Transmit(&hspi1, data_buf, 6, HAL_MAX_DELAY);
}

//reply to cmd 71
void transmit_71(){
	
	//use reply from ice
	data_buf[0] = 0xBA;
	data_buf[1] = 0x06;
	data_buf[2] = 0x0D;
	data_buf[3] = 0xCA;
	data_buf[4] = 0x6A;
	data_buf[5] = 0x1F;
	
	HAL_SPI_Transmit(&hspi1, data_buf, 6, HAL_MAX_DELAY);
}

//Test unit, sets various control registers on the FPGA
void test_unit(){
	
	//this can be done on the FPGA side
	data_buf[0] = 0x00; //error register
	data_buf[1] = 0x03; //interrupt reason
	data_buf[2] = 0x50; //Status register (ready and "seek done" set)
	
	HAL_SPI_Transmit(&hspi1, data_buf, 3, HAL_MAX_DELAY);
}
//Request mode (get various data regarding the CD)
void request_mode(){
	//according to ice there are only two things the system will request
	//put in others later though just in case
	//System Version, need to return: Rev 5.07
	//NullDC returns: "SE      Rev 6.43990408"; check o-scope
	//"SE      " for Drive info; "Rev 6.43" sys. ver; "990408" sys. date
	const char version[8] = "Rev 5.07"; 
	
	if(cmd_buf[1] == 18){
		memcpy(data_buf, version, 8);
		HAL_SPI_Transmit(&hspi1, data_buf, 8, HAL_MAX_DELAY);
	}
	//return everything aside from Drive info, system ver & system date
	else{
		memset(data_buf, 0x00, 10);
		HAL_SPI_Transmit(&hspi1, data_buf, 10, HAL_MAX_DELAY);
	}
}
//return error code
void req_error(){
	//nothing can go wrong in MY system
	//just send zeros
	data_buf[0] = 0xF0;
	memset(&data_buf[1], 0x00, 9);
	HAL_SPI_Transmit(&hspi1, data_buf, 10, HAL_MAX_DELAY);
}
//read the image file
void read_cd(uint8_t cmd){
	
	uint32_t lba_addr = 0; //address recieved from Dreamcast
	uint32_t byte_addr = 0; //address to send to FATFS
	uint32_t transfer_length = 0; //number of sectors to transfer
	uint8_t cdda_flag = 0; //send either 2352 bytes or 2048 bytes
	uint8_t count = 0;
	uint8_t file_index = 0;
	
	//check command type for getting transfer length
	if(cmd == 0x31){
		transfer_length = (cmd_buf[5] << 8) | (cmd_buf[6]);
	}
	else{
		transfer_length = (cmd_buf[7] << 16) | (cmd_buf[8] << 8) | (cmd_buf[9]);
	}
	
	//check if address is MSF (minutes, seconds, frames) or FAD; if MSF convert to FAD
	//is MSF
	if((cmd_buf[0] & 0x01) == 0x01){
		lba_addr = (cmd_buf[1] * 60) + cmd_buf[2]; //convert minutes to seconds
		lba_addr = (lba_addr * 75) + cmd_buf[3]; //convert seconds to frames (75 frames/sec)
	}
	//is FAD
	else{
		lba_addr = (cmd_buf[1] << 16) | (cmd_buf[2] << 8) | (cmd_buf[3]);
	}
	
	//check expected data type (Dreamcast will only request either CDDA or mode1)
	//check if cdda, if so then ignore data sel
	if((cmd_buf[0] & 0x0E) == 0x01){
		cdda_flag = 1;
	}
	//check data sel, check if "other"
	else{
		cdda_flag = ((cmd_buf[0] >> 4) & 0x01);
	}
	
	//cdda or other requests all 2352 bytes back; anything else requests only user data 2048 bytes
	//mode1; 12 sync bytes, 4 header bytes, 2048 user data, 288 EDC/ECC
	
	//find which file to check
	while(count < file_count){
		if(lba[count] <= lba_addr) break;
		count = count + 1;
	}
	
	file_index = count;
	count = 0;
	
	//convert lba_addr to byte_addr
	byte_addr = (lba_addr - lba[file_index]) * 2352;
	
	//begin reading data
	f_lseek(&files[file_index], byte_addr);
	while(count < transfer_length){
		f_read(&files[file_index], data_buf1, 2352, &junk);
		
		//check if we only send user data or not
		if(cdda_flag == 0x00){
			memcpy(data_buf, &data_buf1[16], 2048);
			HAL_SPI_Transmit(&hspi1, data_buf, 2048, HAL_MAX_DELAY);
		}
		else{
			HAL_SPI_Transmit(&hspi1, data_buf1, 2352, HAL_MAX_DELAY);
		}
		
		count = count + 1;
	}
	
	//reset file pointer
	f_lseek(&files[file_index], (f_tell(&files[file_index]) - f_tell(&files[file_index])));
}
//return specified subcode data
void get_scd(){
	
	//NOTE: subcode updated each frame (13.3ms; 1/75th of a second)
	
	data_buf[0] = 0x00; //reserved
	data_buf[1] = 0x15; //"no audio status info" (NullDC always outputs this; while ice does either cd playback or byte not supported (0x00)

	//check data format
	//all subcode info (100 bytes
	if(cmd_buf[0] == 0x00){
		data_buf[2] = 0x00;
		data_buf[3] = 0x64; //100 bytes
		
		//get 96 bytes of subcode data
	}
	//subcode q data 14 bytes
	else{
		//set control to 4 (this is strange as 4 indicates data rather than audio)
		//I think it's because NullDC always returns the value for track1 which is data
		data_buf[2] = 0x00; //data length MSB
		data_buf[3] = 0x0E; //data length LSB
		data_buf[4] = 0x41; //control/ADR
		data_buf[5] = 0x01; //TNO
		data_buf[6] = 0x01; //"X"? index of some sort
		
		data_buf[7] = 0x00; //elapsed FAD within track
		data_buf[8] = 0x00;
		data_buf[9] = 0x00;
		
		data_buf[10] = 0x00; //reserved
		
		data_buf[11] = 0x00;
		data_buf[12] = 0x00;
		data_buf[13] = 0x96; //start of track1
		
		HAL_SPI_Transmit(&hspi1, data_buf, 14, HAL_MAX_DELAY);
	}
	
}
//play GDDA audio; NEEDS DMA
void cd_play(){
	
	//read data normally, don't worry about extra shit
	//check parameter
	
	//convert to FAD if nessecary
	
	//find appropriate file
	
	//process repeat times
	
	//read and transmit (SPI2)
	//interrupt vector needed, when buffer is empty read more frames replenish buffer
}
/** System Clock Configuration
*/
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SDIO init function */
static void MX_SDIO_SD_Init(void){

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 3;

}

/* SPI1 init function */
static void MX_SPI1_Init(void){

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void){

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void){

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
