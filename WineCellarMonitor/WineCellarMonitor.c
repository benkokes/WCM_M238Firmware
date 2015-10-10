/*
Sensor I2C Addresses: 7-bit(8Bit)
KXCJ9: 0x0F(0x1E)
SHT21: 0x40(0x80)
BH1750:0x5C(0xB8)
BMP180:0x77(0xEE)

SPI Devices:
LCD Sharp LS013B7DH03
EEPROM Microchip 5AA1024 --131072 byte capacity (1Mbit)
-or-
FLASH Macronix MX25R6435F -- 8388608 byte capacity (64Mbit)
*/
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define UART_BAUD_RATE	500000//125000

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
#include "flash.h"
#include "time.h"
#include "DataStructures.h"

/*Global Variables */
time workingtime; //time structure used in SRAM
time starttime;
volatile uint32_t globalsecondcount=0; //yay! 136 years worth of seconds...
volatile uint32_t screentimeout=0;
volatile uint32_t extmembyteindex=0, startTimeinSec=0;
volatile uint16_t intEEpromindex=0;
uint16_t eetimectr=0;
extern volatile unsigned char msginbuf;
volatile uint8_t valid_sw2=1;
volatile uint8_t valid_sw1=1;
uint8_t data_cache[257] = {};
int8_t maxtemp13 =-120, mintemp13 =120;
uint32_t maxtemploc13 =0, mintemploc13 =0;
int16_t tempG_temp = 0;

/*Global Flags*/
volatile uint8_t sleepmode=0;
volatile uint8_t sensormeasure=0;
volatile uint8_t savesensordata=0;
volatile uint8_t checkbatt=0;
volatile uint8_t showscreen=0;
volatile uint8_t blankscreen=0;
volatile uint8_t ejectfromwhile=0;

char scr_buf[26];


void PortInit(void)
{
	/*************
	*Port B Setup*
	*************/
	DDRB |= (1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB5); // SCK, MOSI are outputs. SS MUST be configured as Output.
	PORTB |= (1<<PORTB0)|(1<<PORTB3)|(1<<PORTB4)|(1<<PORTB5);
	/*
	oB0:extmem_Hold	oB1:LED1(extmem side)
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
	oD6:extmem_WP	oD7:extmem_CS
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
	//Prescalar for 0.5 second
	//TCCR2B |= (1<<CS22)|(0<<CS21)|(0<<CS20);
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

void writeTime_intEEprom(uint32_t accum_time) //1024bytes EEPROM
{
	eeprom_write_dword((uint32_t*)intEEpromindex, accum_time);
	
	intEEpromindex = ((intEEpromindex+4) & (0x03FF));
	if(intEEpromindex == 0)
	intEEpromindex =4;
	
	eeprom_write_dword((uint32_t*)intEEpromindex, 0); //write boundary condition indicator .

}

uint8_t clearTime_intEEprom(void) //1024bytes EEPROM
{
	
	uint16_t time_index =0;

	for(time_index=0; time_index<1024; time_index+=4)
	{
		eeprom_write_dword((uint32_t*)time_index, 255);
	}
	
	return 1;  //return true
}

uint16_t retrieve_EEtime_index(void)
{
	uint32_t prevtime=0, currtime1=0;
	uint16_t maxtime_index =0, eeindex=0;

	for(eeindex=4; eeindex<1024; eeindex+=4)
	{
		currtime1=eeprom_read_dword((uint32_t*)eeindex);
		
		if(currtime1>prevtime){
			maxtime_index = eeindex;

			}else if(currtime1<prevtime){
			return maxtime_index; //once the values stop increasing, boundary found, return location.
		}
		
		prevtime=currtime1;
	}
	return 4; //either the values are all the same, or they increased in value till the end of internalEEprom
}
/*
void eetime_dump(void)
{
//char sub_buf[22];
uint32_t curtime1=0;
uint16_t maxtime_index =0;

for(maxtime_index=0; maxtime_index<1024; maxtime_index+=4)
{
//uart_puts_p(PSTR("\r\n"));
curtime1=eeprom_read_dword((uint32_t*)maxtime_index);
//sprintf(sub_buf,"%4d:%10ld,", maxtime_index,curtime1);
//sprintf(sub_buf,"%3d,", read_eeprom((eeprombyteindex-i) -1));
//uart_puts(sub_buf);
}

//uart_puts_p(PSTR("endtimedump\r\n"));
}
*/
void data_buf_dump(char* sub_buf, uint8_t *data_cache1)  // used immediately after data has been pulled from the EEPROM
{
	//char sub_buf[6];
	uint16_t i=0;
	for(i=0; i<512; i++)
	{
		if((i%16)==0)
		uart_puts_p(PSTR("\r\n"));
		snprintf(sub_buf,26,"%3d,", data_cache1[i]);
		uart_puts(sub_buf);
	}
	uart_puts_p(PSTR("\r\n"));
}

void clear_buffer(uint8_t *data_cache1)
{
	uint16_t i;
	for(i=0; i<512; i++)
	{
		data_cache1[i] =0;
	}
}

void flash_dump(char* sub_buf)  //Read contents from external flash and push out UART.
{
	//char sub_buf[6];
	uint32_t i=0;
	uint8_t tempval;
	flash_wakeup();
	for(i=0; i<256; i++)
	{
		if((i%16)==0)
		uart_puts_p(PSTR("\r\n"));
		tempval=read_flash_byte(i);
		snprintf(sub_buf,26,"%3d,", tempval);
		//sprintf(sub_buf,"%3d,", read_eeprom((eeprombyteindex-i) -1));
		uart_puts(sub_buf);
	}

	flash_deep_power_down();
	uart_puts_p(PSTR("\r\n"));
}

uint32_t find_data_boundary(void)
{
	uint32_t extmem_addr = 0;
	uint8_t element_accum=0;
	uint8_t tempval = 0, EM_val=0;
	int8_t tempvals = 0, boundary_found=0;
	//char tempbuf[32];
	for(extmem_addr=0; extmem_addr <EXTMEM_SIZE; extmem_addr++)
	{
		EM_val = read_flash_byte(extmem_addr);
		//sprintf(tempbuf,"%6ld,%3d\r\n", extmem_addr,EM_val);
		//uart_puts(tempbuf);
		if((extmem_addr % 16) == 0)
		element_accum =0; //reset the accumulator at the beginning of each frame location
		
		
		if(EM_val == 0xFF)
		element_accum++;
		else
		element_accum=0;
		
		if(element_accum >15){ // boundary condition found, return location.
			boundary_found=1;
			
		}
		if((extmem_addr % 16) == 13){
			//sprintf(tempbuf,"%6ld,%3d\r\n", extmem_addr,EM_val);
			//uart_puts(tempbuf);
			tempval = EM_val;
		}
		
		if((extmem_addr % 16) == 15 && (boundary_found==0))
		{
			tempvals = (int8_t)(tempval-128);
			
			if(tempvals>maxtemp13){
				maxtemp13=tempvals;
				maxtemploc13 = extmem_addr;
				//sprintf(tempbuf,"H%3ld,%3d\r\n", extmem_addr,EM_val);
				//uart_puts(tempbuf);
			}
			if(mintemp13>tempvals){
				mintemp13=tempvals;
				mintemploc13 = extmem_addr;
				//sprintf(tempbuf,"L%3ld,%3d\r\n", extmem_addr,EM_val);
				//uart_puts(tempbuf);
			}
			//sprintf(tempbuf,"%8ld,u%3d,s%3d\r\n", extmem_addr,(uint8_t)tempval,(int8_t)tempvals);
			//uart_puts(tempbuf);
		}
		if(boundary_found==1){
			/*
			sprintf(tempbuf,"H%4d,L%4d\r\n", maxtemp13, mintemp13);
			uart_puts(tempbuf);
			sprintf(tempbuf,"Hi%4ld,Li%4ld\r\n", maxtemploc13, mintemploc13);
			uart_puts(tempbuf);
			*/
			return extmem_addr;
		}
		if((extmem_addr%8192) ==0){
			PORTB ^= (1<<PORTB2);
		}
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
		snprintf(mainbuf,26, "%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d",cursorpos[0],
		cursorpos[1],cursorpos[2],cursorpos[3],cursorpos[4],cursorpos[5],cursorpos[6],cursorpos[7],
		cursorpos[8],cursorpos[9],cursorpos[10],cursorpos[11],cursorpos[12],cursorpos[13],cursorpos[14],
		cursorpos[15],cursorpos[16],cursorpos[17],cursorpos[18],cursorpos[19]);
		drawline_str(100, cursorpos);
		snprintf(mainbuf,26, "%1d%1d%1d%1d %1d%1d %1d%1d %1d%1d %1d%1d %1d%1d  ",temptime[0],temptime[1],temptime[2],temptime[3],temptime[4],temptime[5],temptime[6],temptime[7],temptime[8],temptime[9],temptime[10],temptime[11],temptime[12],temptime[13]);
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

	gmtime(eeprom_read_dword((uint32_t*)intEEpromindex) + startTimeinSec , &workingtime);//need to populate workingtime struct with retrieved time (base +offset)
	gmtime(startTimeinSec, &starttime);//need to retrieve initial time and populate starttime struct.

	sprintf_P(maintxtbuf, PSTR("FindingDataBoundry...")); //drawline_str(40, "FindingDataBoundry...");
	drawline_str(40, maintxtbuf);
	spi_highspeed();
	extmembyteindex = find_data_boundary();//function returns last location of boundary array.
	PORTB  &= ~(1<<PORTB2);  //make sure LED is off
	spi_regularspeed();
	if(extmembyteindex>0)
	extmembyteindex -=15;//Program expects extmembyteindex to be the next free space, if there is a boundary present, refer to the memory location after the last good data.
	
	clearlcd();

	snprintf(maintxtbuf,26, "IntEE loc:%4d       ",intEEpromindex); //Internal EEprom fill.
	drawline_str(10, maintxtbuf);
	
	//drawline_str(20, "Start Time:          ");
	sprintf_P(maintxtbuf, PSTR("Start Time:          "));
	drawline_str(20, maintxtbuf);
	snprintf(maintxtbuf,26, "%4d/%2d/%2d-%2d:%2d:%2d  ",starttime.year + 1900,starttime.month + 1,starttime.mday,starttime.hour,starttime.minute,starttime.second); //Initial Time
	drawline_str(30, maintxtbuf);

	//drawline_str(40, "Most recent time:    ");
	sprintf_P(maintxtbuf, PSTR("Most recent time:    "));
	drawline_str(40, maintxtbuf);
	snprintf(maintxtbuf,26, "%4d/%2d/%2d-%2d:%2d:%2d  ",workingtime.year + 1900,workingtime.month + 1,workingtime.mday,workingtime.hour,workingtime.minute,workingtime.second); //current time
	drawline_str(50, maintxtbuf);

	snprintf(maintxtbuf,26, "ExtEM loc:%6lu     ", extmembyteindex); //Location of most-current-data marker
	drawline_str(60, maintxtbuf);

	snprintf(maintxtbuf,26, "MeasEvtsRemain:%5lu ",(EXTMEM_SIZE-extmembyteindex)/16); //Current external extextmem fill(if any)
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
			//intEEpromindex = globalsecondcount = extmembyteindex =0;//TODO: reset time and data counters
			clearlcd();
			startTimeinSec = initialsettime(maintxtbuf, &starttime);
			EEstore_initial_time(startTimeinSec);
			globalsecondcount=0;
			intEEpromindex = 4; //reset internal EEprom for time save
			extmembyteindex = 0; //reset external extmem byte index
			buttoncheck=1;
			flash_4kSector_erase(1); //clear out first sector for freshie writes.
			maxtemp13 =-120;
			mintemp13 =120;
			maxtemploc13 =0;
			mintemploc13 =0;
			_delay_ms(50);
		}
		if(PINC & 0x02){//right is continue, use existing timers and data offsets
			buttoncheck=2; //any value to exit and continue
			
			_delay_ms(50);
		}
		_delay_ms(10);
	}//end while()
	_delay_ms(200);
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
	
	snprintf(mainbuf,26, "R:%4d,P:%4d ",(int)recent_measure->roll,(int)recent_measure->pitch);
	drawline_str(1, mainbuf);
	snprintf(mainbuf,26, "Yr:%-2d D:%-2d Hr:%-2d M:%-2d",t->year,t->mday,t->hour,t->minute);
	drawline_str(20, mainbuf);
	snprintf(mainbuf,26, "%5d,%5d,%5d     ",recent_measure->Xaxis12, recent_measure->Yaxis12, recent_measure->Zaxis12);
	drawline_str(40, mainbuf);
	snprintf(mainbuf,26, "%10u,%10d ",(int)recent_measure->Pressure12BMP,(int)recent_measure->temperature12BMP);
	drawline_str(60, mainbuf);
	snprintf(mainbuf,26, "%10d,%10d ",recent_measure->humidity12SHT, ((int16_t)recent_measure->temperature12SHT));
	drawline_str(80, mainbuf);
	snprintf(mainbuf,26, "%10u,%10d ",recent_measure->lightLevel16,recent_measure->BattLevel10);
	drawline_str(100, mainbuf);
	snprintf(mainbuf,26, "RTC Enabled: %d       ",(TIMSK2 & 0x01)); //Clock running?
	drawline_str(110, mainbuf);
	snprintf(mainbuf,26, "eByteIdx:%6lu       ",extmembyteindex);
	drawline_str(120, mainbuf);
}

void HLscreen(char *maintxtbuf)
{
	clearlcd();//shutdown screens

	sprintf_P(maintxtbuf, PSTR("MaxTemp:             "));
	drawline_str(10, maintxtbuf);
	snprintf(maintxtbuf,26, " %3d*c ",maxtemp13); //Max Temp observed since time 0
	largechardraw_str(20, maintxtbuf);
	snprintf(maintxtbuf,26, "Hours Ago:%ld          ",((extmembyteindex-maxtemploc13)/16)/12); //last maxtemp measure --16 bytes per frame, 1 frame per 5 min, 12 frames per hr.
	drawline_str(42, maintxtbuf);

	sprintf_P(maintxtbuf, PSTR("MinTemp:             "));
	drawline_str(60, maintxtbuf);
	snprintf(maintxtbuf,26, " %3d*c ",mintemp13); //Min temp observed since time 0
	largechardraw_str(70, maintxtbuf);
	snprintf(maintxtbuf,26, "Hours Ago:%ld          ",((extmembyteindex-mintemploc13)/16)/12); //last mintemp measure
	drawline_str(92, maintxtbuf);
	
	snprintf(maintxtbuf,26, "Next Meas:%5lusec   ",296-(globalsecondcount%296)); //time to next measure
	drawline_str(110, maintxtbuf);
	snprintf(maintxtbuf,26, "ScreenOff in:%2ldsec ",screentimeout-globalsecondcount); //time to next measure
	drawline_str(120, maintxtbuf);

}

int main(void)
{

	char maintxtbuf[35]; // Screen can only handle 21.3 characters(put back to 24 after debug).
	//char screen_buf[24];
	//uint8_t data_cache[515];
	//int16_t i=0;

	uint32_t ExtEM_startaddrR=0,ExtEM_endaddrR=0;
	sensordata recent_measure;
	uint16_t DataCacheIndex=0;  //the uC buffer index, never larger than the dynamic_bufl
	uint16_t EMpagespace_remain=256; //Space remaining in current extmem page. cant use 512 because not enough uC RAM.
	char *header=NULL;
	
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
	
	//uint8_t edgemarker[20] = {15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15};
	//write_page_flash(0, edgemarker, 0, 16);


	//LCD Init
	lcd_init();
	//_delay_ms(10);
	
	//Fresh slate!
	clearlcd();
	
	//uart_puts_p(PSTR("Startup!\r\n"));
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
	//clearTime_intEEprom();
	//eetime_dump();
	
	//Startup screen
	clearlcd();//shutdown screens


	flash_wakeup(); //make sure extmem is ready for action!
	//sprintf(maintxtbuf, "%x, %x       ", flash_status_register(), flash_config_register());
	/*
	flash_4kSector_erase(0);
	for(i=0; i<256; i++)
	{
	data_cache[i] = i;
	
	}
	//write_page_flash(0, data_cache, 0, 256);
	writeEMstorage(data_cache, 256);
	flash_deep_power_down();
	flash_wakeup();
	flash_dump(maintxtbuf);
	while(1){}

	//	drawline_str(14, maintxtbuf);
	//	while(1){}
	*/
	startup_screen(maintxtbuf); //all the startup screen graphics
	EMpagespace_remain = 256-(extmembyteindex & 0xFF);
	//Sleep the extmem
	flash_deep_power_down();

	//Enable RTC
	TIMSK2  |= (1<<TOIE2);
	
	clearlcd();
	_delay_ms(50); // allow the UART buffer to flush (debug)
	drawline_str(50, "SystemRunning...     ");
	drawline_str(60, "Screen Load in 5 sec ");
	disable_UART0();//disable power to UART module
	
	while(1)
	{
		SMCR|=(1<<SE);
		asm volatile("sleep"::);
		SMCR &= ~(1<<SE);
		
		if((sensormeasure==1) && (showscreen<1))// if active, no sensor measuring if screens showing.
		{
			SingleReadKXCJ9(&recent_measure);
			recent_measure.Pressure12BMP = bmp180_getpressure()/10; //range is 30000 to 110000, which is why we need the  /10 to fit into an int16
			recent_measure.temperature12BMP = bmp180_gettemperature();
			SHT2x_MeasureHM(HUMIDITY,&recent_measure);
			SHT2x_MeasureHM(TEMP,&recent_measure);
			recent_measure.lightLevel16 = bh1750_getlux();
			recent_measure.humidity12SHT= (unsigned int)SHT2x_CalcRH(recent_measure.humidity12SHT);
			recent_measure.temperature12SHT = (int16_t)SHT2x_CalcTemperatureC(recent_measure.temperature12SHT); //shift up to handle smaller storage type in buffer array
			if(checkbatt==1)
			recent_measure.BattLevel10 = check_batt_level();
			
			//Keep track of min/max temperature and the time of occurrence
			if(((int16_t)recent_measure.temperature12SHT)>maxtemp13){
				maxtemp13=(int16_t)recent_measure.temperature12SHT;
				maxtemploc13 = extmembyteindex;
			}
			if(mintemp13>((int16_t)recent_measure.temperature12SHT)){
				mintemp13=(int16_t)recent_measure.temperature12SHT;
				mintemploc13 = extmembyteindex;
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
			data_cache[DataCacheIndex+12] = ((recent_measure.temperature12BMP)/10+127); data_cache[DataCacheIndex+13] = ((uint8_t)recent_measure.temperature12SHT) + 128;
			data_cache[DataCacheIndex+14] = (recent_measure.BattLevel10 & 0x00FF); data_cache[DataCacheIndex+15] = ((recent_measure.BattLevel10 & 0xFF00)>>8);
			
			DataCacheIndex = DataCacheIndex + 16; //increment data_cache location to next free space
			
			if(DataCacheIndex>=EMpagespace_remain)  // Write memory when data_cache full.
			savesensordata=1; //write it all out to sensor data!
			
			enable_UART0();//turn on UART module
			snprintf(maintxtbuf,35,"%ld,%ld,%d, %d, %d, %x, %x\r\n",globalsecondcount, extmembyteindex, EMpagespace_remain , showscreen, savesensordata, PCICR, TIMSK2);
			uart_puts(maintxtbuf);
			_delay_ms(50);
			disable_UART0();//turn off UART module
		}
		
		if(savesensordata==1)//Write sensor data to extmem
		{
			if(DataCacheIndex>1){
				spi_highspeed();
				EMpagespace_remain = writeEMstorage(data_cache, DataCacheIndex); //write data to external extmem, and return remaining space in extmem page
				writeTime_intEEprom(globalsecondcount);						//Write time to INTERNAL EEprom
				spi_regularspeed();
				DataCacheIndex=0;
			}
			savesensordata=0;
		}

		if(showscreen==1)  // shows Min/Max temp and time it happened
		{
			//HLscreen(maintxtbuf);
			clearlcd();
			drawline_str(60,"   Placeholder!         ");
			
		}
		else if(showscreen==2)
		{
			retrieveEM_data(data_cache,1,14);  //Temperature
			clearlcd();
			snprintf(maintxtbuf,26, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("Temperature-dgC ",maintxtbuf, data_cache);//show graph screen of temperature
			snprintf(maintxtbuf,26, "Lo:%4d, Hi:%4d    ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			snprintf(maintxtbuf,26, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		else if(showscreen==3)
		{
			retrieveEM_data(data_cache,1,9);

			clearlcd();
			snprintf(maintxtbuf,26, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("  Humidity(%RH)  ",maintxtbuf, data_cache);//show graph screen of humidity
			snprintf(maintxtbuf,26, "Lo:%4d, Hi:%4d    ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			snprintf(maintxtbuf,26, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		
		else if(showscreen==4)
		{
			retrieveEM_data(data_cache,1,7);
			clearlcd();
			snprintf(maintxtbuf,26, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("  Pressure(mPa)  ", maintxtbuf, data_cache);//show graph screen of  Atmospheric pressure
			snprintf(maintxtbuf,26, "Lo:%4u, Hi:%4u    ",((uint16_t)(data_cache[126])<<8 )|(uint16_t)(data_cache[125]&0x00FF),((uint16_t)(data_cache[124])<<8 )|(uint16_t)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			snprintf(maintxtbuf,26, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		else if(showscreen==5)
		{
			retrieveEM_data(data_cache,1,11);
			clearlcd();
			snprintf(maintxtbuf,26, " new   TIME(%3u)  old",data_cache[127]);
			graph_screen("   ALS(lux)      ", maintxtbuf, data_cache);//show graph screen of  Ambient Light
			snprintf(maintxtbuf,26, "Lo:%4d, Hi:%4d    ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			snprintf(maintxtbuf,26, "Idx%4u,    %4u    ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		else if(showscreen==6)
		{
			clearlcd();
			//gmtime(globalsecondcount, &workingtime);
			elapsed_time(globalsecondcount, &workingtime);
			stats_screen(maintxtbuf, &recent_measure, &workingtime); //Current state of sensors, All Text
		}
		else if(showscreen==7)
		{
			ejectfromwhile=0;
			enable_UART0();//turn on UART module.
			clearlcd();
			//eetime_dump();

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
			gmtime(startTimeinSec+globalsecondcount, &workingtime);
			sprintf_P(maintxtbuf, PSTR("Current Time         "));
			drawline_str(100, maintxtbuf);
			snprintf(maintxtbuf,26, "%4d/%2d/%2d-%2d:%2d:%2d  ",workingtime.year + 1900,workingtime.month + 1,workingtime.mday,workingtime.hour,workingtime.minute,workingtime.second); //current time
			drawline_str(110, maintxtbuf);
			snprintf(maintxtbuf,26, "TotSec:%12ld  ",globalsecondcount);
			drawline_str(120, maintxtbuf);
			PCICR &= ~(1<<PCIE1)&~(1<<PCIE2);//disable PC1INT_vect & PC2INT_vect, switches act locally
			//_delay_ms(150);
			while(ejectfromwhile==0){
				
				if ((PINC & 0x02))  //Switch 2 -- righty allows me to exit out of  while(eject.. loop
					break;
				
				if(msginbuf>0)
				{
					int BufBytesRxd  = uart_gets(maintxtbuf, 19);
					//drawline_str(10, maintxtbuf);
					if(maintxtbuf[0]== '$')
					{
						uart_puts_p(PSTR("RcvdCmd\r\n"));
						header = strtok(maintxtbuf, ",");
						ExtEM_startaddrR = atol(strtok(NULL, ","));
						ExtEM_endaddrR = atol(strtok(NULL, "*"));
						
						snprintf(scr_buf,26, "%c, S:%ld,St:%ld  %d",header[1], ExtEM_startaddrR,ExtEM_endaddrR, msginbuf);//was screen_buf
						drawline_str(100, scr_buf);
						
						if(header[1] == 'A'){//if(maintxtbuf[1] == 'A'){
							snprintf(scr_buf,26, "S:%ld,St:%ld\r\n", ExtEM_startaddrR,ExtEM_endaddrR);
							uart_puts(scr_buf);
							flash_wakeup();
							while(ExtEM_startaddrR < ExtEM_endaddrR)
							{
								snprintf(scr_buf,26, "%3d,", read_flash_byte(ExtEM_startaddrR));
								uart_puts(scr_buf);
								ExtEM_startaddrR++;
							}
							uart_puts_p(PSTR("CompleteRcvdCmd\r\n"));
						}
						else if(header[1] == 'B'){
							flash_wakeup();
							spi_highspeed();
							while(ExtEM_startaddrR < ExtEM_endaddrR)
							{
								uart_putc(read_flash_byte(ExtEM_startaddrR));
								ExtEM_startaddrR++;
							}
							spi_regularspeed();
						}
						else{
							sprintf_P(scr_buf, PSTR("Malformed Command"));
							drawline_str(80, scr_buf);
							_delay_ms(2000);
						}
					}
					//drawline_str(10, maintxtbuf);
					msginbuf--;
					snprintf(scr_buf,26, "MsgBufCt:%d, %d ", msginbuf, BufBytesRxd); //was screen_buf
					drawline_str(80, scr_buf);
					
					ExtEM_startaddrR=0;
					ExtEM_endaddrR=0;
					//header = NULL;
					flash_deep_power_down();
					ejectfromwhile=1;
				}
				if(PIND & 0x20){ //left button will allow for time reset of current time count, non-interrupt.
					TIMSK2  &= ~(1<<TOIE2);//disable RTC
					clearlcd();
					
					sprintf_P(scr_buf, PSTR("                 "));
					drawline_str(20, maintxtbuf);
					sprintf_P(scr_buf, PSTR("                 "));
					drawline_str(30, maintxtbuf);
					sprintf_P(scr_buf, PSTR("                 "));
					drawline_str(40, maintxtbuf);
					globalsecondcount = initialsettime(maintxtbuf, &workingtime);
					globalsecondcount = globalsecondcount - startTimeinSec; // just use the globalsec
					ejectfromwhile=1;
					TIMSK2  |= (1<<TOIE2); //reenable RTC
				}
				
			}// End while(!PINC...
			clearlcd();
			PCICR |= (1<<PCIE1) | (1<<PCIE2);//Reenable PC1INT_vect & PC2INT_vect
			disable_UART0(); //turn off UART module
			showscreen =0;
			ejectfromwhile=0;
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
	
	if(showscreen < 7)
	{
		if(showscreen ==0)
		{
			sprintf_P(scr_buf, PSTR("MaxTemp:             "));
			drawline_str(1, scr_buf);
			snprintf(scr_buf,26, " %3d*c ",maxtemp13); //Max Temp observed since time 0
			largechardraw_str(11, scr_buf);
			snprintf(scr_buf,26, "Hours Ago:%ld          ",((extmembyteindex-maxtemploc13)/16)/12); //last maxtemp measure --16 bytes per frame, 1 frame per 5 min, 12 frames per hr.
			drawline_str(32, scr_buf);

			sprintf_P(scr_buf, PSTR("MinTemp:             "));
			drawline_str(50, scr_buf);
			snprintf(scr_buf,26, " %3d*c ",mintemp13); //Min temp observed since time 0
			largechardraw_str(60, scr_buf);
			snprintf(scr_buf,26, "Hours Ago:%ld          ",((extmembyteindex-mintemploc13)/16)/12); //last mintemp measure
			drawline_str(82, scr_buf);
			
			gmtime(startTimeinSec+globalsecondcount, &workingtime);
			snprintf(scr_buf,26, "%4d/%2d/%2d-%2d:%2d:%2d  ",workingtime.year + 1900,workingtime.month + 1,workingtime.mday,workingtime.hour,workingtime.minute,workingtime.second); //current time
			drawline_str(100, scr_buf);
			elapsed_time(globalsecondcount, &workingtime);
			snprintf(scr_buf,26, "%4d/mm/%2d-%2d:%2d:%2d  ",workingtime.year,workingtime.mday,workingtime.hour,workingtime.minute,workingtime.second); //current time
			drawline_str(110, scr_buf);
		}
		
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
			//if(showscreen==7){
			//	ejectfromwhile=0;
			//}
			showscreen=0;
		}
		
	}
}
ISR(PCINT1_vect) //sw2-right
{
	if(valid_sw2){
		if(PINC & 0x02){
			valid_sw2=0;

			if(showscreen>=7)
			showscreen=1;
			else
			showscreen++;
		}
		}else{
		if(!(PINC & 0x02))
		valid_sw2=1;
	}
	_delay_ms(10);
	
	if(showscreen>0)
		savesensordata=1; //Save the data in the data_cache so the memory space can be used for the screens.

	screentimeout= globalsecondcount+32;
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
