/*
* sharplcd.c
*
* Created: 1/30/2014 4:28:53 PM
*  Author: bkokes
*/
#include <stdio.h>
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "sharplcd.h"
#include "spi.h"
#include "glcdfont.h"
#include "uart.h"


//requires minimum spacing of 26 rows if lower case used.
void largechardraw_str(uint8_t rowaddr, char *textbuf)
{
	uint8_t lcdlinebuf[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //16 bytes per line, one extra in case /0.
	volatile uint16_t fontcurrentbyte=0, currentcharbyteoffset=0;
	volatile uint16_t fontbitindex=0, lcdbyteindex=0, strcharindex=0, lcdbitindex=0,fontrownum=0;
	uint8_t i=0;

	
	SPCR &= ~(1<<DORD);//MSB first
	PORTC |= (1<<PORTC3); //LCD CS high.
	_delay_us(6);
	spi_transfer(0b10000000);
	while(fontrownum<26)
	{
		currentcharbyteoffset = (textbuf[strcharindex]-32)*26;
		
		fontcurrentbyte = pgm_read_word(&fontarray16x26[currentcharbyteoffset+fontrownum]);
		for(fontbitindex=0; fontbitindex< 18; fontbitindex++) //load one font byte into the LCD buffer +2 spaces
		{
			if(fontbitindex < 16)  //load bits from Font byte into LCD buffer byte
			{
				if(fontcurrentbyte & (1<<(15-fontbitindex))){ //isolate specific bit and test for 1 or 0
					lcdlinebuf[lcdbyteindex] &= ~(1<<lcdbitindex); //black pixel
				}else{
					lcdlinebuf[lcdbyteindex] |= (1<<lcdbitindex); //white pixel
				}
			}
			if(fontbitindex>15) //load spaces into LCD buffer byte
			{
				lcdlinebuf[lcdbyteindex] |= (1<<lcdbitindex); //white pixel
			}
			
			if(lcdbitindex ==7 ){
				lcdbitindex =0;
				lcdbyteindex++;
			}else{
				lcdbitindex++;
				//if(lcdbyteindex>16)
				//	break;
			}
		}
		
		if(lcdbyteindex < 15)
		{
			strcharindex++;
			fontbitindex=0;
		}else{
			lcdlinebuf[15] |= (3<<lcdbitindex); //required because 2 end bits are left over from building the line.  2 end bits are set to white.

			SPCR |= (1<<DORD);//LSB First
			spi_transfer(rowaddr+fontrownum); //Initial row address(static) + row offset
			
			for(i=0; i<16; i++){
				spi_transfer(lcdlinebuf[i]); //LCD Data
			}
			spi_transfer(0x00);//pad after the command+address+data package
			
			fontrownum++;
			strcharindex=0;
			fontbitindex=0;
			lcdbyteindex=0;
			lcdbitindex=0;
			fontcurrentbyte=0;
		}
		
	}//End while(fontrownum...
	spi_transfer(0x00);
	_delay_us(3);
	PORTC &= ~(1<<PORTC3); //LCD CS low.
	SPCR &= ~(1<<DORD);//MSB first
}


void drawline_str(uint8_t rowaddr, char *textbuf)
{
	
	uint8_t lcdlinebuf[17]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //16 bytes per line, one extra in case /0.
	unsigned int fontcurrentbyte=0, currentcharbyteoffset=0;
	unsigned int fontbyteindex=0, lcdbyteindex=0, fontcharindex=0, lcdbitindex=0,fontrownum=0;
	uint8_t i=0;
	
	SPCR &= ~(1<<DORD);//MSB first
	PORTC |= (1<<PORTC3); //LCD CS high.
	_delay_us(6);
	spi_transfer(0b10000000);
	while(fontrownum<8)
	{
		//printf("FontCharIndex:%3d, FontByteIndex:%3d, LCDBitIndex:%3d, LCDByteIndex:%3d FontRowNum:%3d \r\n", fontcharindex,fontbyteindex,lcdbitindex,lcdbyteindex,fontrownum);
		currentcharbyteoffset = textbuf[fontcharindex]*6;
		//currentchar = pgm_read_byte(fontarray + (textbuf*5)+ fontcharindex);//currentchar = fontarray[textbuf[fontcharindex]*5];
		fontcurrentbyte = pgm_read_byte(fontarray6x7+currentcharbyteoffset+fontbyteindex);//fontcurrentbyte = fontarray[currentcharbyteoffset+fontbyteindex];
		
		//lcdlinebuf[lcdbyteindex] |= ((fontarray[currentcharbyteoffset+fontbyteindex] & (1<<fontrownum))<<lcdbitindex);
		
		if(fontbyteindex==5){//inserts space between characters
			//lcdlinebuf[lcdbyteindex] &= ~(1<<lcdbitindex); //black pixel
			lcdlinebuf[lcdbyteindex] |= (1<<lcdbitindex); //white pixel
			//uart_puts(" ");
			}else{
			if(((fontcurrentbyte & (1<<fontrownum))<<lcdbitindex)){ //if(((fontarray[currentcharbyteoffset+fontbyteindex] & (1<<fontrownum))<<lcdbitindex)){
				lcdlinebuf[lcdbyteindex] &= ~(1<<lcdbitindex); //black pixel
				//lcdlinebuf[lcdbyteindex] |= (1<<lcdbitindex); //white pixel
				//uart_puts("*");
				}else{
				
				lcdlinebuf[lcdbyteindex] |= (1<<lcdbitindex); //white pixel
				//lcdlinebuf[lcdbyteindex] &= ~(1<<lcdbitindex); //black pixel
				//uart_puts(" ");
			}
		}

		if(fontbyteindex<5){ //Font bit & byte indexer
			fontbyteindex++;
			}else{
			fontbyteindex=0; //done with current character.
			fontcharindex++; //move onto next char in the text buffer
		}
		
		if(lcdbitindex<7){ //LCD Bit & Byte indexer
			lcdbitindex++;
			}else{
			lcdbitindex=0;
			lcdbyteindex++;
			
			if(lcdbyteindex>15){
				SPCR |= (1<<DORD);//LSB First
				spi_transfer(rowaddr+fontrownum); //Initial row address(static) + row offset
				//SPCR &= ~(1<<DORD);//MSB first
				for(i=0; i<16; i++){
					spi_transfer(lcdlinebuf[i]); //LCD Data
				}
				spi_transfer(0x00);//pad after the command+address+data package
				fontrownum++;
				fontbyteindex=0;
				fontcharindex=0;
				lcdbitindex=0;
				lcdbyteindex=0;
				//uart_puts("\r\n");
			}//end if(lcdb..
		}//end else(lcdb...
		
	}//End while(fontrownum...
	spi_transfer(0x00);
	_delay_us(3);
	PORTC &= ~(1<<PORTC3); //LCD CS low.
}

void clearlcd(void)
{
	SPCR &= ~(1<<DORD);//MSB first
	PORTC |= (1<<PORTC3); //LCD CS high.
	_delay_us(3);
	spi_transfer(0b10100000);
	spi_transfer(0x00);
	_delay_us(3);
	PORTC &= ~(1<<PORTC3); //LCD CS low.
	_delay_ms(5);
	
}

void graph_screen(char* scr_title, char* units, uint8_t* databuf)// always assumes 120bytes loaded into databuf
{
	uint16_t rownum=1, i=0;
	uint16_t currentcharbyteoffset=0;
	uint8_t lcdrowbuf[20], columnindex=0;
	
	SPCR &= ~(1<<DORD);//MSB first
	PORTC |= (1<<PORTC3); //LCD CS high.
	_delay_us(6);
	spi_transfer(0b10000000);
	lcdrowbuf[0]=0;
	//conversion here to fit dataset into graph space
	while(rownum<100)
	{
		//Draws chart legends
		currentcharbyteoffset = scr_title[(100-rownum)/6]*6;//shifts to the correct character
		lcdrowbuf[0] = pgm_read_byte(&fontarray6x7[currentcharbyteoffset+((6-rownum)%6)]); //element 3 counts backwards through the array
		//lcdbyteindex++;
		
		for(columnindex=0; columnindex<120; columnindex++) //draws chart
		{
			if(databuf[columnindex]>=rownum){
				lcdrowbuf[(columnindex/8)+1] &= ~(1<<(columnindex%8));
				//uart_puts("*");
				}else{
				lcdrowbuf[(columnindex/8)+1] |= (1<<(columnindex%8));//white pixel
				//uart_puts(" ");
			}
			
		}//end for(col..

		SPCR |= (1<<DORD);//LSB First
		spi_transfer(rownum); //Initial row address(static) + row offset
		for(i=0; i<16; i++)
		{
			spi_transfer(lcdrowbuf[i]); //LCD Data
		}
		spi_transfer(0x00);//pad after the command+address+data package
		//uart_puts("\r\n");
		//lcdbitindex=0;
		//lcdbyteindex=0;
		rownum++;
				
	}//end while(row...
	spi_transfer(0x00);
	_delay_us(3);
	PORTC &= ~(1<<PORTC3); //LCD CS low.
	drawline_str(102,units); // inserts X-Axis label under graph
	SPCR &= ~(1<<DORD);//MSB first
}
