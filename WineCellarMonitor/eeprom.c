
/* ------------------------------------------------------------------------
**	 Module:	eeprom.c: 	
**	 Author:    Mike Hankey
**	 Hardware 	AVR ATmega328
**	 Software:	gcc 4.3.3 AVR Studio 4.18 Build 700
**
**	 DESCRIPTION: Handles eeprom communications
**						  
**    Version: 1.0
**
**    Copyright © 2010, Mike Hankey
**    All rights reserved. [BSD License]
**    http://www.JaxCoder.com/
** 
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. All advertising materials mentioning features or use of this software
**    must display the following acknowledgement:
**    This product includes software developed by the <organization>.
** 4. Neither the name of the <organization> nor the
**    names of its contributors may be used to endorse or promote products
**    derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** 
** ------------------------------------------------------------------------*/
#include <avr/common.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h> //for TRUE FALSE
#include <stdlib.h>
#include <stdio.h>
//#include "common.h"
#include "eeprom.h"
#include "spi.h"
#include "uart.h"

/* -- GetEEPROMStartPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	None
**	Returns: word - start location in eeprom
** -----------------------------------------------------------------------*/
uint32_t GetEEPROMStartPointer(void)
{
	return eeprom_start;
}

/* -- GetEEPROMAvailPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	None
**	Returns: word - Next available location in eeprom
** -----------------------------------------------------------------------*/
uint32_t GetEEPROMAvailPointer(void)
{
	return eeprom_avail;
}

/* -- UpdateEEPROMPointer ----------------------------------------------
**
**	Description: Updates the avail pointer
**
**	Params:	word - Offset
**	Returns: None
** -----------------------------------------------------------------------*/
void UpdateEEPROMPointer(uint32_t offset)
{
	eeprom_avail += offset;
}

/* -- eeprom_init --------------------------------------------------------
**
**	Description: Initialize/Reset eeprom and all pointers
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void eeprom_init(void)
{
	eeprom_start = 0;
	eeprom_avail = 0;
	eeprom_end = eeprom_start + EEPROM_SIZE;
	EEPROM_HOLD_DIS; //HOLD high (Hold disabled)
	EEPROM_WP_DIS; //WP high (Write enabled)
}

/* -- read_eeprom -----------------------------------------------------------
**
**	Description: Read a byte from eeprom
**
**	Params:	word - address to read from
**	Returns: byte - data
** -----------------------------------------------------------------------*/
uint8_t read_eeprom(uint32_t address)
{
	uint8_t data;
	
	if (address > EEPROM_SIZE){
		PORTB |=(1<<PORTB1);
		_delay_ms(250);
		PORTB &=~(1<<PORTB1);
		return false;
	}
	SPCR &= ~(1<<DORD);//MSB first
	_delay_us(1);
	EEPROM_CS_EN;
	spi_transfer(READ); //transmit read opcode
	spi_transfer((uint8_t)(address>>16));  //send A24-A17 address byte first
	spi_transfer((uint8_t)(address>>8));   //send MSByte address byte next
	spi_transfer((uint8_t)(address));      //send LSByte address last
	data = spi_transfer(0x00); //get data byte
	
	EEPROM_CS_DIS;//PORTB = _BV(SLAVESELECT);
	_delay_us(1);
	return data;
}

/* -- read_page_eeprom --------------------------------------------------
**
**	Description: Read data from eeprom
**
**	Params:	byte* - pointer to data buffer
**			word - address to read from
**			word - length of data to read
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t read_page_eeprom(uint8_t* pdata, uint32_t address, uint16_t len)
{
	if(address > EEPROM_SIZE ){
		while(1){
		PORTB |=(1<<PORTB1);
		_delay_ms(150);
		PORTB &=~(1<<PORTB1);
		_delay_ms(150);
		}
		return false;
	}

	address = address & (EEPROM_SIZE-1);
	EEPROM_CS_EN;//PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	SPCR &= ~(1<<DORD);//MSB first
	spi_transfer(READ); //transmit read opcode
	spi_transfer((uint8_t)(address>>16));  //send A24-A17 address byte first
	spi_transfer((uint8_t)(address>>8));   //send MSByte address byte next
	spi_transfer((uint8_t)(address));      //send LSByte address last

	for (uint16_t i = 0; i <= len; i++)
		*pdata++ = spi_transfer(0x00);
	_delay_us(1);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);

	return true;
}

/* -- write_epprom -------------------------------------------------------
**
**	Description: Write data to EEPROM
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t write_eeprom(uint32_t address, uint8_t data)
{
	uint8_t dummy=0;
	if (address > EEPROM_SIZE){
		PORTB |=(1<<PORTB1);
		_delay_ms(100);
		PORTB &=~(1<<PORTB1);
		_delay_ms(100);
		return false;
	}
	address = address & (EEPROM_SIZE-1);
	write_enable_eeprom();

	while((eeprom_status_register() & 0x02)==0)
		{};

	SPCR &= ~(1<<DORD);//MSB first
	
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	
	dummy=spi_transfer(WRITE); //write instruction
	dummy=spi_transfer((uint8_t)(address>>16));  //send A24-A17 address byte first
	dummy=spi_transfer((uint8_t)(address>>8));   //send MSByte address byte next
	dummy=spi_transfer((uint8_t)(address));      //send LSByte address last
  
    dummy=spi_transfer(data); //write data byte
	
	_delay_us(1);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
	
	while((eeprom_status_register() & 0x01)==1)
		{};

	return true;
}

/* -- write_page_epprom --------------------------------------------------
**
**	Description: Write data to EEPROM
**
**	Params:	byte* - pointer to data buffer
**			word - address to write data
**			word - length of data
**	Returns: bool - true on success false otherwise
** -----------------------------------------------------------------------*/
uint8_t write_page_eeprom(uint32_t dataddress, uint8_t* data_pg, uint16_t buffstart, uint16_t datlen)
{
	uint8_t dummy=0;
	uint16_t i=0;
	//char localbuf[32];
	//sprintf(localbuf,"ad:%3ld,L:%3d\r\n", dataddress, datlen);
	//uart_puts(localbuf);
	if (dataddress > EEPROM_SIZE || datlen > EEPROM_PAGE_SIZE){
		while(1){
		PORTB |=(1<<PORTB1);
		_delay_ms(500);
		PORTB &=~(1<<PORTB1);
		_delay_ms(500);
		//return false;
		}
	}
	//address = address & (EEPROM_SIZE-1);

	write_enable_eeprom();
	//_delay_us(150);

	while((eeprom_status_register() & 0x02)==0)
		{};
	
	SPCR &= ~(1<<DORD);//MSB first		
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	
	dummy =spi_transfer(WRITE); //write instruction
	dummy =spi_transfer((uint8_t)(dataddress>>16));  //send A24-A17 address byte first
	dummy =spi_transfer((uint8_t)(dataddress>>8));   //send MSByte address byte next
  	dummy =spi_transfer((uint8_t)(dataddress));      //send LSByte address last
  
  	for (i = 0; i < datlen; i++){
	    dummy = spi_transfer(data_pg[buffstart+i]); //write data byte
	  }
	_delay_us(1);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);

	while((eeprom_status_register() & 0x01)==1)
	{};
	
	return true;
}

/* -- write_enable_eeprom ------------------------------------------------
**
**	Description: Enable writing to the EEPROM
**
**	Params:	None
**	Returns: None
** -----------------------------------------------------------------------*/
void write_enable_eeprom(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(2);
	spi_transfer(WREN); //write enable
	_delay_us(10);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
}

void write_disable_eeprom(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	spi_transfer(WRDI); //write enable
	_delay_us(10);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
}

/*-- Deep Power down mode--------------------------------------------------
**
**  Description: Deep powerdown. No read/write possible till wakeup (RDID) issued
**
**  No paramets or returns
**----------------------------------------------------------------------*/
void eeprom_deep_power_down(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	spi_transfer(DPD); //write enable
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
	_delay_us(10);
}

/* -- Sleep Release -------------------------------------------------------
**
**	Description:EEPROM wake from deep sleep sequence
**
**	Params:	None
**
**	Returns: Device ID
** -----------------------------------------------------------------------*/
uint8_t eeprom_wakeup(void)
{
	volatile uint8_t data=0;
	
	SPCR &= ~(1<<DORD);//MSB first
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);

	_delay_us(1);
	spi_transfer(RDID); //write instruction
	spi_transfer(0x55);   //send A24-A17 address byte first
	spi_transfer(0x55);   //send MSByte address byte next
	spi_transfer(0x55);   //send LSByte address last
	
	data = spi_transfer(0x00); //write dummy to get Electronic Signature from eeprom
	_delay_us(5);
	//if(data ==0x29)
	//	PORTB |=(1<<PORTB1);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
	write_enable_eeprom();
	while((eeprom_status_register() & 0x02) == 0)
	{}
		write_enable_eeprom();
	
	return data;
}

/*-- Status Register Query--------------------------------------------------
**
**  Description: Query Status Register. 
**
**  ReturnL Status Register contents
**----------------------------------------------------------------------*/
uint8_t eeprom_status_register(void)
{
	uint8_t data=0;
	SPCR &= ~(1<<DORD);//MSB first
	EEPROM_CS_EN; //PORTB &= ~_BV(SLAVESELECT);
	_delay_us(1);
	spi_transfer(RDSR); //write enable
	data = spi_transfer(0xFF); //get data byte
	_delay_us(1);
	EEPROM_CS_DIS; //PORTB = _BV(SLAVESELECT);
	return data;
}

/* -- eeprom_copy ---------------------------------------------------------
**
**	Description: Copies len of eeprom to dest. memory
**
**	Params:	byte* - Destination address
**			word - eeprom source address
**			word - length of chunk to copy.
**	Returns: None
** -----------------------------------------------------------------------*/
uint8_t eeprom_copy(uint8_t* dest, uint32_t src, uint16_t len)
{
	if ((src + len > EEPROM_SIZE) || (len > EEPROM_PAGE_SIZE))
		return false;

	read_page_eeprom(dest, src, len);

	return true;
}

uint16_t writeEEstorage(uint8_t* datacache, uint16_t length)  // this version for buffer-in-ram size of  256 bytes
{
	int16_t pageremaining=0, pageoverflow=0, next_buf_size=0;
	//volatile uint8_t status_reg=0;
	uint8_t edgemarker[20] = {15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15};  //use 16 length for normal operation
	//char uart_buf[32];
	
	eeprom_wakeup();

	pageremaining = 0x100 -(eeprombyteindex & 0xFF);
	pageoverflow = length-pageremaining;  //positive values are overflows
	
	//sprintf(uart_buf, "R:%3u,O:%6d\r\n",pageremaining,pageoverflow);
	//uart_puts(uart_buf);
	
	if((pageoverflow == 0) & (pageremaining == 0x100)) //incoming data fits exactly in eeprom page.
	{
		//uart_puts("FullPage\r\n");
		//uart_puts_p(PSTR("OK:FullPage\r\n"));
		write_page_eeprom(eeprombyteindex, datacache, 0, 256);		//write data to eeprom
		eeprombyteindex = (eeprombyteindex+256) & (EEPROM_SIZE-1); 	//update eeprom counter to the next free space
		//uart_puts("Complete256PageWritten\r\n");
		next_buf_size=256;  //space to fill the remaining page on next write.
	}
	else if(pageremaining >=length) //incoming data fits within page, no overflow
	{
		//uart_puts_p(PSTR("OK:Page underflow\r\n"));
		//uart_puts("PartialPage\r\n");
		write_page_eeprom(eeprombyteindex, datacache, 0, length); //length fits, write it.
		eeprombyteindex = (length+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
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
		write_page_eeprom(eeprombyteindex, datacache, 0, pageremaining-1); //length fits, write it.
		eeprombyteindex = (pageremaining+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
		
		write_page_eeprom(eeprombyteindex, datacache, 0, length-pageremaining); //length fits, write it.
		eeprombyteindex = ((length-pageremaining)+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
		
		next_buf_size = 256 - (length - pageremaining);//return page space remaining
		//uart_puts("LessThan256PageWritten\r\n");
	}
	else//pages have somehow overflown, but not met the above conditions. weird 
	{
		//error condition, shouldn't be able to get here
		PRR &= ~(1<<PRUSART0);//turn on UART module
		uart_puts_p(PSTR("HozedWriteEvent\r\n"));//uart_puts("HozedWriteEvent\r\n");
		sprintf((char*)edgemarker,"%d,%d,%d,%d\r\n",length, pageremaining, pageoverflow, next_buf_size );
		uart_puts((char*)edgemarker);
		_delay_ms(50);
		PRR |= (1<<PRUSART0);//turn off UART module
	}

	if((eeprombyteindex+16) >EEPROM_SIZE)  //insert end-of-data marker in case of power loss.
		write_page_eeprom(0, edgemarker,0, 16);
	else
		write_page_eeprom(eeprombyteindex, edgemarker,0, 16);
	
	eeprom_deep_power_down();
	return next_buf_size;
}//end writeEEstorage(..


/*
uint16_t writeEEstorage(uint8_t* datacache, uint16_t length)  // this version for buffer-in-ram size of 512 bytes
{
	int16_t pageremaining=0, pageoverflow=0, next_buf_size=0;
	uint8_t OverflowPagetouch=0, i=0;
	//volatile uint8_t status_reg=0;
	uint8_t edgemarker[16] = {15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15};
	//char uart_buf[32];	
	eeprom_wakeup();

	pageremaining = 0x100 -(eeprombyteindex & 0xFF);
	pageoverflow = length-pageremaining;  //positive values are overflows
	OverflowPagetouch = (pageoverflow>256)?2:1;
	
	//sprintf(uart_buf, "R:%3u,O:%6d,T:%1d\r\n",pageremaining,pageoverflow,OverflowPagetouch );
	//uart_puts(uart_buf);
		
	if((pageoverflow == 0 || pageoverflow == 0x100) & (pageremaining == 0x100)) //incoming data fits completely in eeprom page.
	{
		for(i=0; i<(length/0xFF); i++) //check to determine page write quantity
			{
			//uart_puts("FullPage\r\n");
			write_page_eeprom(eeprombyteindex, datacache, i*256, 256-1);		//write data to eeprom
			eeprombyteindex = (eeprombyteindex+256) & (EEPROM_SIZE-1); 	//update eeprom counter
			//uart_puts("Complete256PageWritten\r\n");
			}
		next_buf_size=512;  //space to fill the remaining page on next write.
	}
	else if(pageoverflow <= 0) //incoming data fits within page, no overflow
	{
		//uart_puts("PartialPage\r\n");
		write_page_eeprom(eeprombyteindex, datacache,0,length-1); //length fits, write it.
		eeprombyteindex = (length+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
		if(pageoverflow==0)
			next_buf_size=512;
		else	
			next_buf_size = 512-length;//return page space remaining
		//uart_puts("LessThan256PageWritten\r\n");	
	}
	else//pages have overflown -- pageoverflow > 0
	{
		if(OverflowPagetouch==1)
		{
			//uart_puts("OverFlow1\r\n");
			write_page_eeprom(eeprombyteindex, datacache,0,pageremaining-1); //appropriate writes, spanning the end of one page and beginning of next
			eeprombyteindex = (pageremaining+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
			write_page_eeprom(eeprombyteindex, datacache,pageremaining ,pageoverflow-1);
			eeprombyteindex = (pageoverflow+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
			next_buf_size = 256-pageoverflow ;//return page space remaining.
			//uart_puts("Overflow1PageWritten\r\n");
		}
		else if(OverflowPagetouch==2)
		{
			//uart_puts("OverFlow2\r\n");
			write_page_eeprom(eeprombyteindex, datacache,0,pageremaining-1); // consumes the end of one page, a full next page and the beginning of a 3rd page
			eeprombyteindex = (pageremaining+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
			write_page_eeprom(eeprombyteindex, datacache,pageremaining,256-1);
			eeprombyteindex = (256+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
			write_page_eeprom(eeprombyteindex, datacache,pageoverflow,(pageoverflow-256)-1);
			eeprombyteindex = ((pageoverflow-256)+eeprombyteindex) & (EEPROM_SIZE-1); //update byte position
			//uart_puts("Overflow2PageWritten\r\n");
			next_buf_size = 512-pageoverflow;
		}
		else
		{
			//error condition, shouldn't be able to get here
			uart_puts_p("HozedWriteEvent\r\n");//uart_puts("HozedWriteEvent\r\n");
		}
	}

	if((eeprombyteindex+16) >EEPROM_SIZE)  //insert end-of-data marker in case of power loss.
		write_page_eeprom(0, edgemarker,0, 16);
	else
		write_page_eeprom(eeprombyteindex, edgemarker,0, 16);
		
	eeprom_deep_power_down();
	return next_buf_size;
}//end writeEEstorage(..
*/

/*
*data_buf - Buffer which data will be read into
skip_dist- offset from most recent measurement event. skip_dist=1 for most recent data
element_offset - location of individual sensor data within 16 byte frame. Add 1 to the data_cache index location to obtain  correct position
*/
void retrieve_data(uint8_t *data_buff, uint8_t skip_dist, uint8_t element_offset)
{
	uint8_t scalefactor=0;
	uint16_t i=0,maxval_index=0, minval_index=0;
	int16_t max_val=-32767, min_val=32767, temp_val=0;
	uint16_t valuerange=0;
	uint32_t  eeidx=0;
	
	eeidx=eeprombyteindex-1;  //el youa no mess-o witha da reeeel eehndex, also, we need to go back to the last recorded element.
eeprom_wakeup();
	if((element_offset==13)||(element_offset==14)){
		max_val =0; min_val=0xFF;
		for(i=0; i<120; i++){
			data_buff[i] = read_eeprom(((eeidx-(16-element_offset))-(16*i*skip_dist))&(EEPROM_SIZE-1));
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
			data_buff[i] = read_eeprom(((eeidx-(16-element_offset))-(16*i*skip_dist))&(EEPROM_SIZE-1));//low byte
			data_buff[i+128] = read_eeprom(((eeidx-(15-element_offset))-(16*i*skip_dist))&(EEPROM_SIZE-1));//high byte
			
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
	
eeprom_deep_power_down();	
}