/*
Sensor I2C Addresses: 7-bit(8Bit)
KXCJ9: 0x0F(0x1E)
SHT21: 0x40(0x80)
BH1750:0x5C(0xB8)
BMP180:0x77(0xEE)

SPI Devices:
LCD Sharp LS013B7DH03
EEPROM Microchip 5AA1024 --131072 byte capacity
*/
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define UART_BAUD_RATE	125000

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <compat/ina90.h>
#include <util/delay.h>
#include "twimaster.h"
#include "i2c_core.h"
#include "uart.h"
#include "spi.h"
#include "KXCJ9.h"
#include "bh1750.h"
#include "bmp180.h"
#include "sht21.h"
#include "sharplcd.h"
#include "eeprom.h"
#include "time.h"
#include "DataStructures.h"

/*Global Variables */
time curtime; //time structure used in SRAM
time starttime;
volatile uint32_t globalsecondcount=0; //yay! 136 years worth of seconds...
volatile uint32_t screentimeout=0;
volatile uint32_t eeprombyteindex=0, startTimeinSec=0;
volatile uint16_t intEEpromindex=0;
uint16_t eetimectr=0;
extern volatile unsigned char msginbuf;
volatile uint8_t valid_sw2=1;
volatile uint8_t valid_sw1=1;
uint8_t data_cache[257];
int8_t maxtemp13 =0, mintemp13 =0;
uint32_t maxtemploc13 =0, mintemploc13 =0;

/*Global Flags*/
volatile uint8_t sleepmode=0;
volatile uint8_t sensormeasure=0;
volatile uint8_t savesensordata=0;
volatile uint8_t checkbatt=0;
volatile uint8_t showscreen=0;
volatile uint8_t blankscreen=0;


void PortInit(void)
{
	/*************
	*Port B Setup*
	*************/
	DDRB |= (1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB5); // SCK, MOSI are outputs. SS MUST be configured as Output.
	PORTB |= (1<<PORTB0)|(1<<PORTB3)|(1<<PORTB4)|(1<<PORTB5);
	/*
	oB0:EEPROM_Hold	oB1:LED1(eeprom side)
	oB2:LED2(uCside)oB3:MOSI
	iB4:MISO		oB5:SCK
	oB6:OSC			oB7:OSC
	*/

	/*************
	*Port C Setup*
	*************/
	DDRC |=(1<<DDC0)|(1<<DDC2)|(1<<DDC3);
	PORTC |=(1<<PORTC0)|(1<<PORTC3);
	/*
	oC0:LCDDispMode	iC1:SW2-R
	oC2:GPIO1		oC3:LCD_CS
	iC4:SDA			iC5:SCL
	xC6:RESET
	*/

	/*************
	*Port D Setup*
	*************/
	DDRD  |= (1<<DDD1)|(1<<DDD3)|(1<<DDD4)|(1<<DDD6)|(1<<DDD7);
	PORTD |= (1<<PORTD1)|(1<<PORTD3)|(1<<PORTD4)|(1<<PORTD6)|(1<<PORTD7);
	/*
	iD0:UARTRX		oD1:UARTTX
	iD2:AccelInt    oD3:LCDDisp_EXT
	oD4:ALS_DVI		iD5:SW1-L
	oD6:EEPROM_WP	oD7:EEPROM_CS
	*/
}

/***********************************
*Analog-to-Digital Converter setup *
***********************************/
void adc_init(void)
{
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //128 divisor from main clock for ADC
	ADCSRB |= (0<<ADTS2)|(0<<ADTS0)|(0<<ADTS0); //ADC free running mode (No trigger)
	//DIDR0 = 0b00001001; // disable digital buffers for ADC lines 0 and 3
	ADMUX |= (1<<REFS0)|(1<<MUX3)|(1<<MUX2)|(1<<MUX1);
	ADMUX &= ~(1<<ADLAR);  //No Left adjust
	//ADCSRA |=(1<<ADEN); //Enable ADC
}

void RTCInit(void)
{
	//Disable timer2 interrupts
	TIMSK2  = 0;
	//Enable asynchronous mode
	ASSR  |= (1<<AS2);
	//set initial counter value
	TCNT2=0;
	//set prescaler 1024 (8 seconds)
	TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20);
	//wait for registers update
	while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)));
	//clear interrupt flags
	TIFR2  |= (1<<TOV2);
	//enable TOV2 interrupt
	//TIMSK2  |= (1<<TOIE2);
}

void pcint1_init(void) //Right switch
{
	PCICR |= (1<<PCIE1); //Allow interrupts for pins PCINT14 through PCINT8
	PCMSK1 = (1<<PCINT9); //Enable interrupts specifically for pin PCINT9, disable the rest.
}

void pcint2_init(void) //Left Switch
{
	PCICR |= (1<<PCIE2);//Allow interrupts for pins PCINT23 through PCINT16
	PCMSK2 = (1<<PCINT21); //Enable interrupts specifically for pin PCINT21, disable the rest.
}
void enablePWM_OC2B(void)
{
	TCCR2A |= (0<<COM2B1) | (1<<COM2B0);
}

void lcd_init(void)
{
	PORTC |= (1<<PORTC3); //LCD CS high.
	PORTC &= ~(1<<PORTC3); //LCD CS low.  deselect
	PORTC |= (1<<PORTC0); //LCD EXTMODE: 1=Hardware toggle
}

void EEstore_initial_time(uint32_t secondstostore)
{
	
	eeprom_write_dword(0, secondstostore);
	
}

uint8_t writeTime_intEEprom(uint32_t accum_time) //1024bytes EEPROM
{
	eeprom_write_dword((uint32_t*)intEEpromindex, accum_time);
	
	intEEpromindex = ((intEEpromindex+4) & (0x03FF)); 
	if(intEEpromindex == 0)
		intEEpromindex =4;
	
	return 1;  //return true
}

uint16_t retrieve_EEtime_index(void)
{
	uint32_t prevtime=0, currtime=0;
	uint16_t maxtime_index =0, eeindex=0;

	for(eeindex=4; eeindex<1024; eeindex=eeindex+4)
	{
		currtime=eeprom_read_dword((uint32_t*) &eeindex);
		if(currtime>prevtime){
			//maxtime=curtime;
			maxtime_index = eeindex;
		}
		prevtime=currtime;
	}
	return maxtime_index;
}

void eetime_dump(void)
{
		char sub_buf[16];
		uint32_t curtime1=0;
		uint16_t maxtime_index =0;

		for(maxtime_index=0; maxtime_index<1024; maxtime_index+=4)
		{
			uart_puts("\r\n");
			curtime1=eeprom_read_dword((uint32_t*)maxtime_index);
			sprintf(sub_buf,"%10ld,", curtime1);
			//sprintf(sub_buf,"%3d,", read_eeprom((eeprombyteindex-i) -1));
			uart_puts(sub_buf);
		}

		uart_puts("\r\n");
	
} 

void data_buf_dump(uint8_t *data_cache1)  // used immediately after data has been pulled from the EEPROM
{
	char sub_buf[6];
	uint16_t i=0;
	for(i=0; i<512; i++)
	{
		if((i%16)==0)
		uart_puts_p("\r\n");
		sprintf(sub_buf,"%3d,", data_cache1[i]);
		uart_puts(sub_buf);
	}
	uart_puts_p("\r\n");
}

void clear_buffer(uint8_t *data_cache1)
{
	uint16_t i;
	for(i=0; i<512; i++)
	{
		data_cache1[i] =0;
	}
}

void eeprom_dump(void)  //Read contents from EEPROM and push out UART.
{
	char sub_buf[6];
	uint32_t i=0;
	uint8_t tempval;
	eeprom_wakeup();
	for(i=0; i<512; i++)
	{
		if((i%16)==0)
		uart_puts_p("\r\n");
		tempval=read_eeprom(i);
		sprintf(sub_buf,"%3d,", tempval);
		//sprintf(sub_buf,"%3d,", read_eeprom((eeprombyteindex-i) -1));
		uart_puts(sub_buf);
	}

	eeprom_deep_power_down();
	uart_puts_p("\r\n");
}

uint32_t find_data_boundary(void)
{
	uint32_t ee_addr = 0;
	uint8_t element_accum=0;
	uint8_t tempval = 0;
	
	for(ee_addr=0; ee_addr <EEPROM_SIZE; ee_addr++)
	{
		tempval = read_eeprom(ee_addr);
	
		if(tempval == 0x0F)
			element_accum++;
		else
			element_accum=0;
				
		if(element_accum >15) // boundary condition found, return location.
			return ee_addr;
	
		if((ee_addr % 14) == 0)
		{
			if((int8_t)tempval>maxtemp13){
				maxtemp13=(int8_t)tempval;
				maxtemploc13 = ee_addr;
			}
			if(mintemp13<(int8_t)tempval){
				mintemp13=(int8_t)tempval;
				mintemploc13 = ee_addr;
			}
		}
		
		if((ee_addr%8192) ==0)
			PORTB ^= (1<<PORTB2);
	}
	PORTB  &= ~(1<<PORTB2); //if the LED toggle exited the function high, turn it off.
	return 0; // possible error-- No boundary condition found.
}

int check_batt_level(void)
{
	unsigned int adc = 0;
	
	PRR &= ~(1<<PRADC);  //Poweron the ADC module
	ADCSRA |=(1<<ADEN); //Enable ADC
	/* change the adc multiplexer channel */
	
	//ADMUX = 0b0000111; // Single Ended ADC7 read
	//ADMUX = 0b01011110;	 // AVCC as VREF and 1.1VDC bandgap read
	ADMUX |= (1<<REFS0)|(1<<MUX3)|(1<<MUX2)|(1<<MUX1); //AVCC reference to the 1.1v bandgap

	/* start a conversion */
	ADCSRA |= (1<<ADSC);
	
	/* wait until the conversion is finished */
	while (ADCSRA & (1<<ADSC));
	adc = ADCL;
	adc |= ADCH << 8;

	/*** Perform a second time to get correct values ***/

	/* start a conversion */
	ADCSRA |= (1<<ADSC);
	
	/* wait until the conversion is finished */
	while (ADCSRA & (1<<ADSC));
	adc = ADCL;
	adc |= ADCH << 8;

	ADCSRA &= ~(1<<ADEN); //Disable ADC
	
	PRR |= (1<<PRADC);//power off ADC module
	
	return (1125300L / adc); // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

}// End Check_batt...

int int_round(double x) {
	   assert(x >= INT_MIN-0.5);
	   assert(x <= INT_MAX+0.5);
	   if (x >= 0)
	   return (int) (x+0.5);
	   return (int) (x-0.5);
}

void calc_RP(sensordata *recent_measure)
{
	float tempX = (float)recent_measure->Xaxis12/1024;
	float tempY = (float)recent_measure->Yaxis12/1024;
	float tempZ = (float)recent_measure->Zaxis12/1024;
	
	recent_measure->pitch = int_round(atan2(tempX, sqrt(pow(tempY,2)+pow(tempZ,2)))*180/3.1415926);
	recent_measure->roll = int_round(atan2(tempY,tempZ)*180/3.1415926);
	//recent_measure->pitch = atan(tempX/sqrt(pow(tempY,2))+pow(tempZ,2))*(180.0/3.14159);
	//recent_measure->roll = atan2(tempY/sqrt(pow(tempX,2))+pow(tempZ,2))*(180.0/3.14159);
}

uint32_t initialsettime(char *mainbuf, time *t){
	uint8_t temptime[15]= { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	char cursorpos[20]= {'_','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-',' '};	
	uint8_t num=0, loc=0, cursorloc=0;
	char *header=NULL;
	uint32_t secsSinceEpoc =0;
	
	sprintf_P(mainbuf, PSTR("Send Time over UART: "));
	drawline_str(20, mainbuf);
	sprintf_P(mainbuf, PSTR("$T,yy,mm,dd,hh,mm,ss*"));
	drawline_str(30, mainbuf);
	sprintf_P(mainbuf, PSTR("     - or -          "));
	drawline_str(40, mainbuf);
	sprintf_P(mainbuf, PSTR("Set time with buttons"));
	drawline_str(50, mainbuf);
	sprintf_P(mainbuf, PSTR("<-Toggle    Advance->"));
	drawline_str(70, mainbuf);
	sprintf_P(mainbuf, PSTR("YYYY MM DD HH MM SS  "));
	drawline_str(90, mainbuf);
	
	PCICR &= ~(1<<PCIE1)&~(1<<PCIE2);//disable PC1INT_vect & PC2INT_vect
	
	while(loc<14)
	{
		if(PIND & 0x20) //lefty
		{
			if (num<9)
				num++;
			else
				num =0;
			
			temptime[loc] = num;
			_delay_ms(150);
		}
		if(PINC & 0x02) //righty
		{

			loc++; 
			cursorloc++;
			if((cursorloc == 4)||(cursorloc == 7)||(cursorloc == 10) ||(cursorloc == 13) ||(cursorloc == 16) )
				cursorloc++;
			
			cursorpos[cursorloc] = '_';
			num=0;
			_delay_ms(150);
		}
		sprintf(mainbuf, "%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d",cursorpos[0],
			cursorpos[1],cursorpos[2],cursorpos[3],cursorpos[4],cursorpos[5],cursorpos[6],cursorpos[7],
			cursorpos[8],cursorpos[9],cursorpos[10],cursorpos[11],cursorpos[12],cursorpos[13],cursorpos[14],
			cursorpos[15],cursorpos[16],cursorpos[17],cursorpos[18],cursorpos[19]);
		drawline_str(100, cursorpos);
		sprintf(mainbuf, "%1d%1d%1d%1d %1d%1d %1d%1d %1d%1d %1d%1d %1d%1d  ",temptime[0],temptime[1],temptime[2],temptime[3],temptime[4],temptime[5],temptime[6],temptime[7],temptime[8],temptime[9],temptime[10],temptime[11],temptime[12],temptime[13]);
		drawline_str(110, mainbuf); //show time as its being inputed
		
		if(msginbuf>0) //uart has message!!
		{
			uart_gets(mainbuf, 22);
			if(mainbuf[0]== '$')
			{
				if(mainbuf[1]== 'T'){
					header = strtok(mainbuf, ",");
					t->year = atoi(strtok(NULL, ","));
					t->month = atoi(strtok(NULL, ","));
					t->wday = atoi(strtok(NULL, ","));
					t->hour = atoi(strtok(NULL, ","));
					t->minute = atoi(strtok(NULL, ","));
					t->second = atoi(strtok(NULL, "*"));
					loc = 0xFF;

					secsSinceEpoc = getSecsSinceEpoch(1970,t->month,t->wday, (t->year) - 1970, t->hour,t->minute,t->second);
					return secsSinceEpoc;
				}else{
					sprintf_P(mainbuf, PSTR("Malformed Command"));
					drawline_str(100, mainbuf);
					_delay_ms(2000);
					//drawline_str(100,"                 ");
					sprintf_P(mainbuf, PSTR("                 "));
					drawline_str(100, mainbuf);
				}
			}
		}
	}// end while(loc<13)
	
	PCICR |= (1<<PCIE1)|(1<<PCIE2); //reenable PC1INT_vect & PC2INT_vect
	
	if (loc < 0xFF){
		t->year = ((uint16_t)temptime[0] *1000)+((uint16_t)temptime[1] *100)+((uint16_t)temptime[2] *10)+((uint16_t)temptime[3] *1);
		t->month =(temptime[4] *10)+(temptime[5] *1);
		t->wday =(temptime[6] *10)+(temptime[7] *1);
		t->hour = (temptime[8] *10)+(temptime[9] *1);
		t->minute = (temptime[10] *10)+(temptime[11] *1);
		t->second = (temptime[12] *10)+(temptime[13] *1);
	secsSinceEpoc = getSecsSinceEpoch(1970,t->month,t->wday, t->year - 1970, t->hour,t->minute,t->second);
	return secsSinceEpoc;
	}
	
	return 0;
}

void startup_screen(char *maintxtbuf)
{
	uint8_t buttoncheck=0;
			 
	sprintf_P(maintxtbuf, PSTR("Start Up!           "));
	drawline_str(20, maintxtbuf);
	sprintf_P(maintxtbuf, PSTR("FindingTimeBoundry..."));
	drawline_str(30, maintxtbuf);

	intEEpromindex = retrieve_EEtime_index();  //retrieve time offset location in intEEprom.
	startTimeinSec = eeprom_read_dword(0);

	gmtime(eeprom_read_dword((uint32_t*)&intEEpromindex)+startTimeinSec , &curtime);//need to populate curtime struct with retrieved time (base +offset)
	gmtime(startTimeinSec, &starttime);//need to retrieve initial time and populate starttime struct.

	sprintf_P(maintxtbuf, PSTR("FindingDataBoundry...")); //drawline_str(40, "FindingDataBoundry...");
	drawline_str(40, maintxtbuf);
	
	eeprombyteindex = find_data_boundary();//function returns last location of boundary array. 

	if(eeprombyteindex>0)
		eeprombyteindex -=15;//Program expects eeprombyteindex to be the next free space, if there is a boundary present, refer to the memory location after the last good data.
	
	clearlcd();

	sprintf(maintxtbuf, "IntEE loc:%4d       ",intEEpromindex); //Internal EEprom fill.
	drawline_str(10, maintxtbuf);
	
	//drawline_str(20, "Start Time:          ");
	sprintf_P(maintxtbuf, PSTR("Start Time:          "));
	drawline_str(20, maintxtbuf);
	sprintf(maintxtbuf, "%4d/%2d/%2d/%2d/%2d/%2d  ",starttime.year + 1900,starttime.month + 1,starttime.mday,starttime.hour,starttime.minute,starttime.second); //Initial Time
	drawline_str(30, maintxtbuf);

	//drawline_str(40, "Most recent time:    ");
	sprintf_P(maintxtbuf, PSTR("Most recent time:    "));
	drawline_str(40, maintxtbuf);
	sprintf(maintxtbuf, "%4d/%2d/%2d/%2d/%2d/%2d  ",curtime.year + 1900,curtime.month + 1,curtime.mday,curtime.hour,curtime.minute,curtime.second); //current time
	drawline_str(50, maintxtbuf);

	sprintf(maintxtbuf, "ExtEE loc:%6lu     ", eeprombyteindex); //Location of most-current-data marker
	drawline_str(60, maintxtbuf);

	sprintf(maintxtbuf, "MeasEvtsRemain:%5lu ",(EEPROM_SIZE-eeprombyteindex)/16); //Current external extEEProm fill(if any)
	drawline_str(70, maintxtbuf);
/*
	sprintf(maintxtbuf, "Est extEE loop:%2ld   ",296-(globalsecondcount%296)); //time to next measure
	drawline_str(80, maintxtbuf);
*/
	//drawline_str(100, "<-RST         Cont->");
	sprintf_P(maintxtbuf, PSTR("<-RST         Cont->"));
	drawline_str(100, maintxtbuf);

	PCICR &= ~(1<<PCIE1)&~(1<<PCIE2);//disable PC1INT_vect & PC2INT_vect

	while(buttoncheck==0){
		if(PIND & 0x20){ //left is reset. Start all counters from 0 and show time screen
			//intEEpromindex = globalsecondcount = eeprombyteindex =0;//TODO: reset time and data counters
			clearlcd();
			startTimeinSec = initialsettime(maintxtbuf, &starttime);
			EEstore_initial_time(startTimeinSec);
			globalsecondcount=0;
			intEEpromindex = 4; //reset internal EEprom for time save
			eeprombyteindex = 0; //reset external EEprom byte index
			buttoncheck=1;
			_delay_ms(50);
		}
		if(PINC & 0x02){//right is continue, use existing timers and data offsets
			buttoncheck=2; //any value to exit and continue
			
			_delay_ms(50);
		}
		_delay_ms(10);
	}//end while()
	
	PCICR |= (1<<PCIE1) | (1<<PCIE2);//reenable PC1INT_vect & PC2INT_vect
}

void stats_screen(char *mainbuf,  sensordata *recent_measure, time *t )
{
	//drawline_str(1,  "Primo Victoria!       ");
	
	sprintf_P(mainbuf, PSTR("Elapsed Time        "));
	drawline_str(10, mainbuf);
	sprintf_P(mainbuf, PSTR("Accel Measure(X,Y,Z)"));
	drawline_str(30, mainbuf);
	sprintf_P(mainbuf, PSTR("Press/Temp(Pa,C)    "));
	drawline_str(50, mainbuf);
	sprintf_P(mainbuf, PSTR("Humid/Temp(RH,C)    "));
	drawline_str(70, mainbuf);
	sprintf_P(mainbuf, PSTR("Light/Batt(lux,V)   "));
	drawline_str(90, mainbuf);
	
	sprintf(mainbuf, "R:%4d,P:%4d ",(int)recent_measure->roll,(int)recent_measure->pitch);
	drawline_str(1, mainbuf);
	sprintf(mainbuf, "Yr:%-2d D:%-2d Hr:%-2d M:%-2d",t->year,t->mday,t->hour,t->minute);
	drawline_str(20, mainbuf);
	sprintf(mainbuf, "%5d,%5d,%5d     ",recent_measure->Xaxis12, recent_measure->Yaxis12, recent_measure->Zaxis12);
	drawline_str(40, mainbuf);
	sprintf(mainbuf, "%10u,%10d ",(int)recent_measure->Pressure12BMP,(int)recent_measure->temperature12BMP);
	drawline_str(60, mainbuf);
	sprintf(mainbuf, "%10d,%10d ",recent_measure->humidity12SHT, recent_measure->temperature12SHT);
	drawline_str(80, mainbuf);
	sprintf(mainbuf, "%10u,%10d ",recent_measure->lightLevel16,recent_measure->BattLevel10);
	drawline_str(100, mainbuf);
	sprintf(mainbuf, "RTC Enabled: %d       ",(TIMSK2 & 0x01)); //Clock running?
	drawline_str(110, mainbuf);
	sprintf(mainbuf, "eByteIdx:%6lu       ",eeprombyteindex);
	drawline_str(120, mainbuf);
}

void HLscreen(char *maintxtbuf)
{
		clearlcd();//shutdown screens

		sprintf_P(maintxtbuf, PSTR("MaxTemp:             "));
		drawline_str(10, maintxtbuf);
		sprintf(maintxtbuf, " %3d*c ",maxtemp13); //Max Temp observed since time 0
		largechardraw_str(20, maintxtbuf);
		sprintf(maintxtbuf, "Hours Ago:%ld          ",((eeprombyteindex-maxtemploc13)/16)/3600); //last maxtemp measure
		drawline_str(42, maintxtbuf);

		sprintf_P(maintxtbuf, PSTR("MinTemp:             "));
		drawline_str(60, maintxtbuf);
		sprintf(maintxtbuf, " %3d*c ",mintemp13); //Min temp observed since time 0
		largechardraw_str(70, maintxtbuf);
		sprintf(maintxtbuf, "Hours Ago:%ld          ",((eeprombyteindex-mintemploc13)/16)/3600); //last mintemp measure
		drawline_str(92, maintxtbuf);
		
		sprintf(maintxtbuf, "Next Meas:%5lusec   ",296-(globalsecondcount%296)); //time to next measure
		drawline_str(110, maintxtbuf);
		sprintf(maintxtbuf, "ScreenOff in:%2ldsec ",screentimeout-globalsecondcount); //time to next measure
		drawline_str(120, maintxtbuf);

}

int main(void)
{
	char maintxtbuf[35]; // Screen can only handle 21.3 characters(put back to 24 after debug).
	//char screen_buf[24];
		//uint8_t data_cache[515];

	uint32_t ExtEE_startaddrR=0,ExtEE_endaddrR=0;
	sensordata recent_measure;
	uint16_t DataCacheIndex=0;  //the uC buffer index, never larger than the dynamic_bufl
	uint16_t EEpagespace_remain=256; //Space remaining in current eeprom page. cant use 512 because not enough uC RAM.
	char *header=NULL;
	uint8_t ejectfromwhile=0;
	
	cli();
	
	//Power reduction register -- ensure SPI is on
	PRR &= ~(1<<PRSPI);
	
	//Power reduction register: Turn off TIMER0, TIMER1 and USART
	PRR |= (1<<PRTIM0)|(1<<PRTIM1);//|(1<<PRUSART0);
	
	//PowerSave State
	SMCR|=(1<<SM1)|(1<<SM0); //Power-save
	
	//I/O Init
	PortInit();
	_delay_ms(10);
	
	//UART init
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));

	//Initialize I2C Bus
	twi_init();
	
	//Initialize the Timer2
	RTCInit();
	//enablePWM_OC2B(); //required to service VCOM of LCD, but convenient to init here.
	
	//Initialize PCINT9 for button2
	pcint1_init();
	
	//Initialize PCINT21 for button1
	pcint2_init();
	
	//SPI Initialization
	init_spi();
	
	//Analog to digital converter - for battery measurement
	adc_init();
	
	//Enable global interrupts
	sei();
	
	//LCD Init
	lcd_init();
	//_delay_ms(10);
	
	//Fresh slate!
	clearlcd();
	
	uart_puts_p(PSTR("Startup!\r\n"));
	_delay_ms(1);
	
	//Accelerometer Init
	//KXCJ9_wake_init(0xFF,0x01);
	SensorInitKXCJ9_int();
	//uart_puts("Accelerometer Sensor Int Complete\r\n");
	
	//Pressure Sensor Init
	bmp180_init();
	//uart_puts("Pressure Sensor Int Complete\r\n");
	
	//Humidity Sensor Init
	SHT2x_SoftReset();
	//uart_puts("Humidity Sensor Int Complete\r\n");
	
	_delay_ms(1);//Delay needs to happen after the SHT init for some reason
	
	//Light Sensor Init
	bh1750_init();
	//uart_puts("Light Sensor Int Complete\r\n");
	
	//Startup screen
	clearlcd();//shutdown screens

	eeprom_wakeup(); //make sure EEprom is ready for action!

	startup_screen(maintxtbuf); //all the startup screen graphics
	
	//Sleep the EEPROM
	eeprom_deep_power_down();

	//Enable RTC
	TIMSK2  |= (1<<TOIE2);
	
	clearlcd();
	_delay_ms(50); // allow the UART buffer to flush (debug)
	drawline_str(64, " Program Running...  ");

	PRR = (1<<PRUSART0);//disable power to UART module
	
	while(1)
	{
		SMCR|=(1<<SE);
		asm volatile("sleep"::);
		SMCR &= ~(1<<SE);
		
		if((sensormeasure==1) &&(showscreen<1))// if active, no sensor measuring if screens showing.
		{
			SingleReadKXCJ9(&recent_measure);
			recent_measure.Pressure12BMP = bmp180_getpressure()/10; //range is 30000 to 110000, which is why we need the  /10 to fit into an int16
			recent_measure.temperature12BMP = bmp180_gettemperature();
			SHT2x_MeasureHM(HUMIDITY,&recent_measure);
			SHT2x_MeasureHM(TEMP,&recent_measure);
			recent_measure.lightLevel16 = bh1750_getlux();
			recent_measure.humidity12SHT= (unsigned int)SHT2x_CalcRH(recent_measure.humidity12SHT);
			recent_measure.temperature12SHT = (int)SHT2x_CalcTemperatureC(recent_measure.temperature12SHT);
			if(checkbatt==1)
				recent_measure.BattLevel10 = check_batt_level();
			//Keep track of min/max temperature and the time of occurence
			if((int8_t)recent_measure.temperature12SHT>maxtemp13){
				maxtemp13=(int8_t)recent_measure.temperature12SHT;
				maxtemploc13 = globalsecondcount;
			}
			if(mintemp13<(int8_t)recent_measure.temperature12SHT){
				mintemp13=(int8_t)recent_measure.temperature12SHT;
				mintemploc13 = globalsecondcount;
			}
			
			//http://theboredengineers.com/2012/09/the-quadcopter-get-its-orientation-from-sensors/
			calc_RP(&recent_measure);
			
			sensormeasure = 0;
			
			data_cache[DataCacheIndex+0] = (recent_measure.Xaxis12 & 0x00FF); data_cache[DataCacheIndex+1] = ((recent_measure.Xaxis12 & 0xFF00)>>8);
			data_cache[DataCacheIndex+2] = (recent_measure.Yaxis12 & 0x00FF); data_cache[DataCacheIndex+3] = ((recent_measure.Yaxis12 & 0xFF00)>>8);
			data_cache[DataCacheIndex+4] = (recent_measure.Zaxis12 & 0x00FF); data_cache[DataCacheIndex+5] = ((recent_measure.Zaxis12 & 0xFF00)>>8);
			data_cache[DataCacheIndex+6] = (recent_measure.Pressure12BMP & 0x00FF); data_cache[DataCacheIndex+7] = ((recent_measure.Pressure12BMP & 0xFF00)>>8);
			data_cache[DataCacheIndex+8] = (recent_measure.humidity12SHT & 0x00FF); data_cache[DataCacheIndex+9] = ((recent_measure.humidity12SHT & 0xFF00)>>8);
			data_cache[DataCacheIndex+10] = (recent_measure.lightLevel16 & 0x00FF); data_cache[DataCacheIndex+11] = ((recent_measure.lightLevel16 & 0xFF00)>>8);
			data_cache[DataCacheIndex+12] = ((recent_measure.temperature12BMP)/10+127); data_cache[DataCacheIndex+13] = ((int8_t)(recent_measure.temperature12SHT));
			data_cache[DataCacheIndex+14] = (recent_measure.BattLevel10 & 0x00FF); data_cache[DataCacheIndex+15] = ((recent_measure.BattLevel10 & 0xFF00)>>8);
			
			DataCacheIndex = DataCacheIndex + 16; //increment data_cache location
			
			if(DataCacheIndex>=EEpagespace_remain)  // Write memory when data_cache full.
				savesensordata=1; //write it all out to sensor data!
		}
		
		if(savesensordata==1)//Write sensor data to EEprom
		{
			if(DataCacheIndex>1){
				EEpagespace_remain = writeEEstorage(data_cache, DataCacheIndex); //write data to external EEprom, and return remaining space in eeprom page
				writeTime_intEEprom(globalsecondcount);						//Write time to INTERNAL EEprom
				
				DataCacheIndex=0;
			}
			savesensordata=0;
		}

		if(showscreen==1)  // shows Min/Max temp and time it happened
		{
			HLscreen(maintxtbuf);
		}
		
		else if(showscreen==2)
		{
			retrieve_data(data_cache,1,14);  //Temperature
			clearlcd();
			sprintf(maintxtbuf, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("Temperature-dgC ",maintxtbuf, data_cache);//show graph screen of temperature
			sprintf(maintxtbuf, "Lo:%4d, Hi:%4d    ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			sprintf(maintxtbuf, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		
		else if(showscreen==3)
		{
			retrieve_data(data_cache,1,9);

			clearlcd();
			sprintf(maintxtbuf, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("  Humidity(%RH)  ",maintxtbuf, data_cache);//show graph screen of humidity
			sprintf(maintxtbuf, "Lo:%4d, Hi:%4d    ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			sprintf(maintxtbuf, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		
		else if(showscreen==4)
		{
			retrieve_data(data_cache,1,7);
			clearlcd();
			sprintf(maintxtbuf, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("  Pressure(mPa)  ", maintxtbuf, data_cache);//show graph screen of humidity
			sprintf(maintxtbuf, "Lo:%4u, Hi:%4u    ",((uint16_t)(data_cache[126])<<8 )|(uint16_t)(data_cache[125]&0x00FF),((uint16_t)(data_cache[124])<<8 )|(uint16_t)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			sprintf(maintxtbuf, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		else if(showscreen==5)
		{
			retrieve_data(data_cache,1,11);
			clearlcd();
			sprintf(maintxtbuf, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("   ALS(lux)      ", maintxtbuf, data_cache);//show graph screen of humidity
			sprintf(maintxtbuf, "Lo:%4d, Hi:%4d    ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			sprintf(maintxtbuf, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		else if(showscreen==6)
		{			
			clearlcd();
			//gmtime(globalsecondcount, &curtime); 
			elapsed_time(globalsecondcount, &curtime);
			stats_screen(maintxtbuf, &recent_measure, &curtime); //Current state of sensors, All Text
		}
		else if(showscreen==7)
		{
			ejectfromwhile=0;
			PRR &= ~(1<<PRUSART0);//turn on UART module.
			clearlcd();

			sprintf_P(maintxtbuf, PSTR("To Rx ASCII data:    "));
			drawline_str(20, maintxtbuf);
			sprintf_P(maintxtbuf, PSTR("$A,S#,St#*           "));
			drawline_str(30, maintxtbuf);
			sprintf_P(maintxtbuf, PSTR("To Rx BINARY data:   "));
			drawline_str(50, maintxtbuf);
			sprintf_P(maintxtbuf, PSTR("$B,S#,St#*           "));
			drawline_str(60, maintxtbuf);
			sprintf_P(maintxtbuf, PSTR("<-Press to reset time"));
			drawline_str(70, maintxtbuf);
			PCICR &= ~(1<<PCIE1)&~(1<<PCIE2);//disable PC1INT_vect & PC2INT_vect
			while((PINC & 0x02)==0 || ejectfromwhile ==0){  //Switch 2 -- righty
				if(msginbuf>0)
				{
					int BufBytesRxd  = uart_gets(maintxtbuf, 18);
					drawline_str(10, maintxtbuf);
					if(maintxtbuf[0]== '$')
					{
						header = strtok(maintxtbuf, ",");
						ExtEE_startaddrR = atol(strtok(NULL, ","));
						ExtEE_endaddrR = atol(strtok(NULL, "*"));
						
						sprintf(maintxtbuf, "%c, S:%ld,St:%ld  %d",header[1], ExtEE_startaddrR,ExtEE_endaddrR, msginbuf);//was screen_buf
						drawline_str(100, maintxtbuf);
						
						if(header[1] == 'A'){//if(maintxtbuf[1] == 'A'){
							sprintf(maintxtbuf, "S:%ld,St:%ld\r\n", ExtEE_startaddrR,ExtEE_endaddrR);
							uart_puts(maintxtbuf);
							eeprom_wakeup();
							while(ExtEE_startaddrR < ExtEE_endaddrR)
							{
								sprintf(maintxtbuf, "%3d,", read_eeprom(ExtEE_startaddrR));
								uart_puts(maintxtbuf);
								ExtEE_startaddrR++;
							}
							uart_puts_P("CompleteRcvdCmd\r\n");
						}
						else if(header[1] == 'B'){
							eeprom_wakeup();
							while(ExtEE_startaddrR < ExtEE_endaddrR)
							{
								uart_putc(read_eeprom(ExtEE_startaddrR));
								ExtEE_startaddrR++;
							}
						}
						else{
							sprintf_P(maintxtbuf, PSTR("Malformed Command"));
							drawline_str(80, maintxtbuf);
							_delay_ms(2000);
						}
					}
					drawline_str(10, maintxtbuf);
					msginbuf--;
					sprintf(maintxtbuf, "MsgBufCt:%d, %d ", msginbuf, BufBytesRxd); //was screen_buf
					drawline_str(80, maintxtbuf);
					
					ExtEE_startaddrR=0;
					ExtEE_endaddrR=0;
					//header = NULL;
					eeprom_deep_power_down();
					
				}
				
				if(PIND & 0x20){ //left button will allow for time reset of currnet time count
					TIMSK2  &= ~(1<<TOIE2);//disable RTC
					clearlcd();
					sprintf_P(maintxtbuf, PSTR("                 "));
					drawline_str(20, maintxtbuf);
					sprintf_P(maintxtbuf, PSTR("                 "));
					drawline_str(30, maintxtbuf);
					sprintf_P(maintxtbuf, PSTR("                 "));
					drawline_str(40, maintxtbuf);
					globalsecondcount = initialsettime(maintxtbuf, &curtime);
					globalsecondcount = globalsecondcount - startTimeinSec; // just use the globalsec
				 ejectfromwhile=1;
				 	TIMSK2  |= (1<<TOIE2); //reenable RTC
				}
				
			}// End while(!PINC...
			clearlcd();
			PCICR |= (1<<PCIE1) | (1<<PCIE2);//Reenable PC1INT_vect & PC2INT_vect
			PRR |= (1<<PRUSART0); //turn off UART module
		}

		/*
		if((recent_measure.BattLevel10) < 2400){  //flash alert LED
		PORTB |=(1<<PORTB1); // This NEEDS to be the RED Led since the bandgap is too small for White.
		_delay_ms(1);
		PORTB &=~(1<<PORTB1);
		}
		*/
	}

}

//Overflow ISR
ISR(TIMER2_OVF_vect)
{
	globalsecondcount+=8;
	
	if((globalsecondcount%296)==0) //measure sensors every 5 minutes
		sensormeasure=1;
	
	if((globalsecondcount%3600)==0){ //Update screen every hour
		//clearlcd();
		//drawline_str(64, " Program Running...  ");
		showscreen=0;
	}
	
	if(globalsecondcount%129600)  //check battery level every 36hours.
		checkbatt=1;

	
	if(screentimeout==globalsecondcount){
		clearlcd();
		drawline_str(64, " Program Running...  ");
		showscreen=0;
	}
}

ISR(PCINT1_vect) //sw2-right
{
	

	if(valid_sw2){
		if(PINC & 0x02){
			valid_sw2=0;
			//showscreen = showscreen+1;
			if(showscreen>7)
				showscreen=1;
			else
				showscreen++;
					
			if(showscreen >2 )
				PCICR &= ~(1<<PCIE2);//disable PC2INT_vect
			else
				PCICR |= (1<<PCIE2);//enable PC2INT_vect
		}
	}else{
		if(!(PINC & 0x02))
		valid_sw2=1;
	}
	_delay_ms(10);
	
	if(showscreen>0)
		savesensordata=1; //Save the data in the data_cache so the memory space can be used for the screens.

	screentimeout= globalsecondcount+16;
	
}

ISR(PCINT2_vect) //sw1-left
{
	

	if(valid_sw1){
		if((PIND & 0x20)){
			TIMSK2  ^= (1<<TOIE2); // Enable or disable the RTC
			valid_sw1=0;
		}
	}else{
		if(!(PIND & 0x20))
		valid_sw1=1;
	}
	_delay_ms(10);
	
}