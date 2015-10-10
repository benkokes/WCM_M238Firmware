/*
 * flash.c
 *
 * Created: 9/2/2015 10:15:53 PM
 *  Author: Ben
 */ 
#include <avr/common.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h> //for TRUE FALSE
#include <stdlib.h>
#include <stdio.h>
//#include "common.h"
#include "flash.h"
#include "spi.h"
#include "uart.h"

/* -- flash_init --------------------------------------------------------
**
**	Description: Initialize/Reset flash and all pointers
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void flash_init(void)
{

	FLASH_HOLD_DIS; //HOLD high (Hold disabled)
	FLASH_WP_DIS; //WP high (Write enabled)
}


/* -- read_flash_byte -----------------------------------------------------------
**
**	Description: Read a byte from external flash
**
**	Params:	word - address to read from
**	Returns: byte - data
** -----------------------------------------------------------------------*/
uint8_t read_flash_byte(uint32_t address)
{
	uint8_t data;
	
	if (address > EXTMEM_SIZE){
		PORTB |=(1<<PORTB1);
		_delay_ms(250);
		PORTB &=~(1<<PORTB1);
		return false;
	}
	SPCR &= ~(1<<DORD);//MSB first
	FLASH_CS_EN;
	_delay_us(1);
	spi_transfer(FL_READ); //transmit read opcode
	spi_transfer((uint8_t)(address>>16));  //send A24-A17 address byte first
	spi_transfer((uint8_t)(address>>8));   //send MSByte address byte next
	spi_transfer((uint8_t)(address));      //send LSByte address last
	data = spi_transfer(0x00); //get data byte
	_delay_us(1);
	FLASH_CS_DIS;//PORTB = _BV(SLAVESELECT);
	
	return data;
}


/* -- read_page_flash --------------------------------------------------
**
**	Description: Read data from flash
**
**	Params:	byte* - pointer to data buffer
**			word - address to read from
**			word - length of data to read
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t read_page_flash(uint8_t* pdata, uint32_t address, uint16_t len)
{
	if(address > EXTMEM_SIZE ){
		while(1){
			PORTB |=(1<<PORTB1);
			_delay_ms(150);
			PORTB &=~(1<<PORTB1);
			_delay_ms(150);
		}
		return false;
	}

	address = address & (EXTMEM_SIZE-1);
	FLASH_CS_EN;//PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	SPCR &= ~(1<<DORD);//MSB first
	spi_transfer(FL_READ); //transmit read opcode
	spi_transfer((uint8_t)(address>>16));  //send A24-A17 address byte first
	spi_transfer((uint8_t)(address>>8));   //send MSByte address byte next
	spi_transfer((uint8_t)(address));      //send LSByte address last

	for (uint16_t i = 0; i <= len; i++)
	*pdata++ = spi_transfer(0x00);
	_delay_us(1);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);

	return true;
}


/* -- write_flash -------------------------------------------------------
**
**	Description: Write data to FLASH
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t write_flash(uint32_t address, uint8_t data)
{
	uint8_t dummy=0;
	if (address > EXTMEM_SIZE){
		PORTB |=(1<<PORTB1);
		_delay_ms(100);
		PORTB &=~(1<<PORTB1);
		_delay_ms(100);
		return false;
	}
	address = address & (EXTMEM_SIZE-1);
	write_enable_flash();

	while((flash_status_register() & 0x02)==0)
	{};

	SPCR &= ~(1<<DORD);//MSB first
	
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	
	dummy=spi_transfer(FL_WRITE); //write instruction
	dummy=spi_transfer((uint8_t)(address>>16));  //send A24-A17 address byte first
	dummy=spi_transfer((uint8_t)(address>>8));   //send MSByte address byte next
	dummy=spi_transfer((uint8_t)(address));      //send LSByte address last
	
	dummy=spi_transfer(data); //write data byte
	
	_delay_us(1);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
	
	while((flash_status_register() & 0x01)==1)
	{};

	return true;
}

/* -- write_page_flash --------------------------------------------------
**
**	Description: Write data to FLASH
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**			word - length of data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t write_page_flash(uint32_t dataddress, uint8_t* data_pg, uint16_t buffstart, uint16_t datlen)
{
	uint8_t dummy=0;
	uint16_t i=0;
	//char localbuf[32];
	//sprintf(localbuf,"ad:%3ld,L:%3d\r\n", dataddress, datlen);
	//uart_puts(localbuf);
	if (dataddress > EXTMEM_SIZE || datlen > FLASH_PAGE_SIZE){
		while(1){
			PORTB |=(1<<PORTB1);
			_delay_ms(500);
			PORTB &=~(1<<PORTB1);
			_delay_ms(500);
			//return false;
		}
	}
	//address = address & (EXTMEM_SIZE-1);

	write_enable_flash();
	//_delay_us(150);

	while((flash_status_register() & 0x02)==0)
	{};
	
	SPCR &= ~(1<<DORD);//MSB first
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	
	dummy =spi_transfer(FL_WRITE); //write instruction
	dummy =spi_transfer((uint8_t)(dataddress>>16));  //send A24-A17 address byte first
	dummy =spi_transfer((uint8_t)(dataddress>>8));   //send MSByte address byte next
	dummy =spi_transfer((uint8_t)(dataddress));      //send LSByte address last
	
	for (i = 0; i < datlen; i++){
		dummy = spi_transfer(data_pg[buffstart+i]); //write data byte
	}
	_delay_us(1);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);

	while((flash_status_register() & 0x01)==1)
	{};
	
	return true;
}

/* -- write_enable_flash ------------------------------------------------
**
**	Description: Enable writing to the FLASH
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void write_enable_flash(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(2);
	spi_transfer(FL_WREN); //write enable
	_delay_us(12);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
}

void write_disable_flash(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	spi_transfer(FL_WRDI); //write enable
	_delay_us(10);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
}

/*-- Deep Power down mode--------------------------------------------------
**
**  Description: Deep powerdown. No read/write possible till wakeup (RDID) issued
**
**  No paramets or returns
**----------------------------------------------------------------------*/
void flash_deep_power_down(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(5);
	spi_transfer(FL_DPD); //Deep Powerdown
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
	_delay_us(45);
}

/* -- Sleep Release -------------------------------------------------------
**
**	Description:FLASH wake from deep sleep sequence
**
**	Params:	None
**
**	Returns: Device ID
** -----------------------------------------------------------------------*/
uint8_t flash_wakeup(void)
{
	uint8_t data=0;
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);

	_delay_us(2);  //20nS min required

	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
	_delay_us(45);
	write_enable_flash();
	while((flash_status_register() & 0x02) == 0)
	{}
	write_enable_flash();
	
	return data;
}

/*-- Status Register Query--------------------------------------------------
**
**  Description: Query Status Register.
**
**  ReturnL Status Register contents
**----------------------------------------------------------------------*/
uint8_t flash_status_register(void)
{
	uint8_t data=0;
	SPCR &= ~(1<<DORD);//MSB first
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	spi_transfer(FL_RDSR); //write enable
	data = spi_transfer(0xFF); //get data byte
	_delay_us(1);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
	return data;
}

/*-- Configuration Register Query--------------------------------------------------
**
**  Description: Query Configuration Register.
**
**  ReturnL Status Register contents
**----------------------------------------------------------------------*/
uint16_t flash_config_register(void)
{
	uint8_t dataL=0, dataH=0;
	SPCR &= ~(1<<DORD);//MSB first
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	spi_transfer(FL_RDCR); //write enable
	dataL = spi_transfer(0xFF); //get data byte
	dataH = spi_transfer(0xFF);
	_delay_us(1);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
	return (dataL | (uint16_t)(dataH<<8));
}

/*-- Sector Erase (4K block)--------------------------------------------------
**
**  Description: Sector Erase function.
**
**  Return: No mo' data
**----------------------------------------------------------------------*/
uint8_t flash_4kSector_erase(uint32_t address)
{
	uint8_t dummy=0;
	if (address > EXTMEM_SIZE){
		PORTB |=(1<<PORTB1);
		_delay_ms(100);
		PORTB &=~(1<<PORTB1);
		_delay_ms(100);
		return false;
	}
	address = address & (EXTMEM_SIZE-1);
	
	write_enable_flash();

	while((flash_status_register() & 0x02)==0)
	{};

	SPCR &= ~(1<<DORD);//MSB first
		
	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
		
	dummy=spi_transfer(FL_SE); //write instruction
	dummy=spi_transfer((uint8_t)(address>>16));  //send A24-A17 address byte first
	dummy=spi_transfer((uint8_t)(address>>8));   //send MSByte address byte next
	dummy=spi_transfer((uint8_t)(address));      //send LSByte address last
		
	_delay_us(1);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
		
	while((flash_status_register() & 0x01)==1)
	{};

	return true;
	
}

uint8_t flash_chip_erase(void)
{
	uint8_t dummy =0;
	write_enable_flash();

	while((flash_status_register() & 0x02)==0)
	{};

	FLASH_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	
	dummy=spi_transfer(FL_CE); //write instruction

	_delay_us(1);
	FLASH_CS_DIS; //PORTB = _BV(SLAVESELECT);
	
	while((flash_status_register() & 0x01)==1)
	{};

	return true;
}

uint16_t writeEMstorage(uint8_t* datacache, uint16_t length)  // this version for buffer-in-ram size of  256 bytes
{
	int16_t pageremaining=0, pageoverflow=0, next_buf_size=0;
	//volatile uint8_t status_reg=0;
	uint8_t edgemarker[20] = {15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15};  //use 16 length for normal operation
	//char uart_buf[32];
	
	flash_wakeup();

	if((extmembyteindex>>12) - ((extmembyteindex + length)>>12)) //clear the next sector (4K if the additional data runs into it
	{
		if((extmembyteindex + length) >= (EXTMEM_SIZE-1)){
			flash_4kSector_erase(1);
			//uart_puts_p(PSTR("StartErase\r\n"));
		}else{	
			flash_4kSector_erase(((extmembyteindex + length)>>12)*4096); 
			//uart_puts_p(PSTR("RunningErase\r\n"));
		}
	}

	pageremaining = 0x100 -(extmembyteindex & 0xFF);
	pageoverflow = length-pageremaining;  //positive values are overflows
	
	//sprintf(uart_buf, "R:%3u,O:%6d\r\n",pageremaining,pageoverflow);
	//uart_puts(uart_buf);
	
	if((pageoverflow == 0) & (pageremaining == 0x100)) //incoming data fits exactly in flash page.
	{
		//uart_puts("FullPage\r\n");
		//uart_puts_p(PSTR("OK:FullPage\r\n"));
		write_page_flash(extmembyteindex, datacache, 0, 256);		//write data to flash
		extmembyteindex = (extmembyteindex+256) & (EXTMEM_SIZE-1); 	//update flash counter to the next free space
		//uart_puts("Complete256PageWritten\r\n");
		next_buf_size=256;  //space to fill the remaining page on next write.
	}
	else if(pageremaining >=length) //incoming data fits within page, no overflow
	{
		//uart_puts_p(PSTR("OK:Page underflow\r\n"));
		//uart_puts("PartialPage\r\n");
		write_page_flash(extmembyteindex, datacache, 0, length); //length fits, write it.
		extmembyteindex = (length+extmembyteindex) & (EXTMEM_SIZE-1); //update byte position
		if(pageremaining == length)
		next_buf_size = 256;
		else
		next_buf_size = pageremaining-length;//return page space remaining
		//uart_puts("LessThan256PageWritten\r\n");
	}
	else if(pageremaining < length) //incoming data overflows remaining page. need to index to next page
	{
		//uart_puts_p(PSTR("OK:Page overflow\r\n"));
		//uart_puts("PartialPage\r\n");
		write_page_flash(extmembyteindex, datacache, 0, pageremaining-1); //length fits, write it.
		extmembyteindex = (pageremaining+extmembyteindex) & (EXTMEM_SIZE-1); //update byte position
		
		write_page_flash(extmembyteindex, datacache, 0, length-pageremaining); //length fits, write it.
		extmembyteindex = ((length-pageremaining)+extmembyteindex) & (EXTMEM_SIZE-1); //update byte position
		
		next_buf_size = 256 - (length - pageremaining);//return page space remaining
		//uart_puts("LessThan256PageWritten\r\n");
	}
	else//pages have somehow overflown, but not met the above conditions. weird
	{
		//error condition, shouldn't be able to get here
		enable_UART0();//turn on UART module
		//uart_puts_p(PSTR("HozedWriteEvent\r\n"));//uart_puts("HozedWriteEvent\r\n");
		sprintf((char*)edgemarker,"%d,%d,%d,%d\r\n",length, pageremaining, pageoverflow, next_buf_size );
		uart_puts((char*)edgemarker);
		_delay_ms(50);
		disable_UART0();//turn off UART module
	}

// We can't do the end marker because  we cannot erase just the 16 bytes.
/*
	if((extmembyteindex+16) >EXTMEM_SIZE)  //insert end-of-data marker in case of power loss.
		write_page_flash(0, edgemarker,0, 16);
	else
		write_page_flash(extmembyteindex, edgemarker,0, 16);
*/	
	flash_deep_power_down();
	return next_buf_size;
}//end writeEEstorage(..

/*
*data_buf - Buffer which data will be read into
skip_dist- offset from most recent measurement event. skip_dist=1 for most recent data
element_offset - location of individual sensor data within 16 byte frame. Add 1 to the data_cache index location to obtain  correct position
*/
void retrieveEM_data(uint8_t *data_buff, uint8_t skip_dist, uint8_t element_offset)
{
	uint8_t scalefactor=0;
	uint16_t i=0,maxval_index=0, minval_index=0;
	int16_t max_val=-32767, min_val=32767, temp_val=0;
	uint16_t valuerange=0;
	uint32_t  fl_idx=0;
	
	fl_idx=extmembyteindex-1;  //el youa no mess-o witha da reeeel eehndex, also, we need to go back to the last recorded element.
	flash_wakeup();
	if((element_offset==13)||(element_offset==14)){
		max_val =0; min_val=0xFF;
		for(i=0; i<120; i++){
			data_buff[i] = read_flash_byte(((fl_idx-(16-element_offset))-(16*i*skip_dist))&(EXTMEM_SIZE-1));
			data_buff[i+128] = 0;
			
			if(data_buff[i]>= max_val){
				max_val = data_buff[i];
				maxval_index = i*skip_dist;
			}
			if(data_buff[i] < min_val){
				min_val = data_buff[i];
				minval_index = i*skip_dist;
			}
		}
		}else{// all others have 2 bytes in the buffer
		for(i=0; i<120; i++){
			data_buff[i] = read_flash_byte(((fl_idx-(16-element_offset))-(16*i*skip_dist))&(EXTMEM_SIZE-1));//low byte
			data_buff[i+128] = read_flash_byte(((fl_idx-(15-element_offset))-(16*i*skip_dist))&(EXTMEM_SIZE-1));//high byte
			
			temp_val = (((int)data_buff[i+128]<<8) | ((int)data_buff[i]));

			if(temp_val>= max_val){
				max_val = temp_val;
				maxval_index = i*skip_dist;
			}
			if(temp_val< min_val){
				min_val = temp_val;
				minval_index = i*skip_dist;
			}

		}
	}

	valuerange= abs(max_val-min_val);
	
	// Below scales the values for graphing.
	if((element_offset==13)||(element_offset==14)){ //for the 8bit values
		if(valuerange==0){
			scalefactor = 1;
			for(i=0; i<=120; i++){
				data_buff[i] = 50;//(min_val-data_buff[i]);
			}
			}else if(valuerange> 100){
			scalefactor = valuerange/100;
			for(i=0; i<120; i++){
				data_buff[i] = (data_buff[i]-min_val)/scalefactor;
			}
			}else{
			scalefactor = 100/valuerange;
			for(i=0; i<120; i++){
				data_buff[i] = (data_buff[i]-min_val)*scalefactor;
			}
		}
		max_val = max_val -128;  //rescale here to give proper reading to display.
		min_val = min_val - 128;
		data_buff[124] =0; // set high bit of max_val to zero;
		data_buff[126] =0; // set high bit of min_val to zero;
		}else{  //for the 16bit values
		
		if(valuerange==0){
			scalefactor = 1;
			for(i=0; i<=120; i++){
				//temp_val = (data_buff[i+128]<<8) | data_buff[i];
				data_buff[i] = 50; //(min_val-temp_val);
			}
			}else if(valuerange> 100){
			scalefactor = valuerange/100;
			for(i=0; i<120; i++){
				temp_val = (int)(data_buff[i+128]<<8) | (int)data_buff[i];
				data_buff[i] = (temp_val-min_val)/scalefactor;
			}
			}else{
			scalefactor = 100/valuerange;
			for(i=0; i<=120; i++){
				temp_val = (int)(data_buff[i+128]<<8) | (int)data_buff[i];
				data_buff[i] = (temp_val-min_val)*scalefactor;
			}
		}

	}
	data_buff[121] = maxval_index;
	data_buff[122] = minval_index;
	data_buff[123] = (uint8_t)(max_val&0x00FF);
	data_buff[124] = (uint8_t)(max_val)>>8;
	data_buff[125] = (uint8_t)(min_val&0x00FF);
	data_buff[126] = (uint8_t)(min_val)>>8;
	data_buff[127] = scalefactor;
	flash_deep_power_down();
}