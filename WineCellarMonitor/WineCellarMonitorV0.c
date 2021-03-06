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

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <compat/ina90.h>
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
time t; //time structure used in SRAM
time starttime;
volatile uint32_t globalsecondcount=0; //yay! 136 years worth of seconds...
volatile uint32_t screentimeout=0;
uint32_t eeprombyteindex=0;

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

char not_leap(void)      //check for leap year
{
	if (!(t.year%100))
	return (char)(t.year%400);
	else
	return (char)(t.year%4);
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

void RTCInit(void)
{
	//Disable timer2 interrupts
	TIMSK2  = 0;
	//Enable asynchronous mode
	ASSR  |= (1<<AS2);
	//set initial counter value
	TCNT2=0;
	//set prescaler 64 (0.5 seconds)
	TCCR2B |= (1<<CS22)|(0<<CS21)|(0<<CS20);
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
/*
uint8_t eeprom_test(void)
{
	uint32_t i=0;
	uint8_t j=0;
	uint8_t testarray[16];
	uint8_t eeprom_read_var=0;
	
	for(i=0; i<15; i++, j++) //write numbers 0-9 to EEPROM
	{
		write_eeprom(i,j);
		_delay_ms(5);
		testarray[i]= (uint8_t)i;
	}
	
	for(i=0; i<15; i++)
	{
		eeprom_read_var=read_eeprom(i);		
		if(testarray[i]-eeprom_read_var) //check for diff between read value and written val
		return 0; //test failed, probably not connected...
	}
	return 1; //Fortune smiles, EEPROM test passed!!
}
*/
void calc_RP(sensordata *recent_measure)
{
	float tempX = (float)recent_measure->Xaxis12/1024;
	float tempY = (float)recent_measure->Yaxis12/1024;
	float tempZ = (float)recent_measure->Zaxis12/1024;
	
	recent_measure->pitch = atan2(tempX, sqrt(pow(tempY,2)+pow(tempZ,2)))*180/3.1415926;
	recent_measure->roll = atan2(tempY,tempZ)*180/3.1415926;
	//recent_measure->pitch = atan(tempX/sqrt(pow(tempY,2))+pow(tempZ,2))*(180.0/3.14159);
	//recent_measure->roll = atan2(tempY/sqrt(pow(tempX,2))+pow(tempZ,2))*(180.0/3.14159);
}

void stats_screen(char *mainbuf,  sensordata *recent_measure, time *t ) 
{
	//drawline_str(1,  "Primo Victoria!       ");
	drawline_str(10, "Elapsed Time          ");
	drawline_str(30, "Accel Measure(X,Y,Z)  ");
	drawline_str(50, "Press/Temp(Pa,C)      ");
	drawline_str(70, "Humid/Temp(RH,C)      ");
	drawline_str(90, "Light/Batt(lux,V)     ");
	drawline_str(110,"EEprombyteindex       ");
	sprintf(mainbuf, "R:%4d,P:%4d ",(int)recent_measure->roll,(int)recent_measure->pitch);
	drawline_str(1, mainbuf);
	sprintf(mainbuf, "%4d,%2d,%2d,%2d:%2d:%2d   ",t->year,t->month,t->day,t->hour,t->minute,t->second);
	drawline_str(20, mainbuf);
	sprintf(mainbuf, "%5d,%5d,%5d     ",recent_measure->Xaxis12, recent_measure->Yaxis12, recent_measure->Zaxis12);
	drawline_str(40, mainbuf);
	sprintf(mainbuf, "%10u,%10d ",(uint16_t)recent_measure->Pressure12BMP,(int)recent_measure->temperature12BMP);
	drawline_str(60, mainbuf);
	sprintf(mainbuf, "%10d,%10d ",recent_measure->humidity12SHT, recent_measure->temperature12SHT);
	drawline_str(80, mainbuf);
	sprintf(mainbuf, "%10u,%10d ",recent_measure->lightLevel16,recent_measure->BattLevel10);
	drawline_str(100, mainbuf);
	//sprintf(mainbuf, "%4u,%4u,%4u,%4u   ",((PIND & 0x20)>>5),((PINC & 0x02)>>1), ((PORTB & 0x02)>>1),((PORTB & 0x04)>>2));
	sprintf(mainbuf, "eByteIdx:%6lu       ",eeprombyteindex);
	drawline_str(120, mainbuf);
		
}

void synthetic_data(uint8_t *data_cache1, uint32_t *dci, uint16_t buff_len) //Fill the buffer with synthetic test data, intended to save 1 sample at a time.
{
	char maintxtbuf[64];
	static unsigned int colwidth=0, column=0, datacacheindex=0;
	unsigned char i=0;
	
	datacacheindex= *dci;
	
	for(i=0; i<16; i++)
	{
		//data_cache1[datacacheindex] = column;
		data_cache1[datacacheindex] = (datacacheindex/2) & 0xFF;
		
		if(datacacheindex<buff_len){
			datacacheindex++;
			*dci=datacacheindex;
		}else{
			savesensordata = 1;
			return;
		}
	}
	
	if(colwidth <3){
		colwidth++;
		}else{
		colwidth=0;
		column = (column+1) & 0x0F;
	}// end if(colwidth

	sprintf(maintxtbuf, "dci:%6lu, eeprmIDX:%6lu, col:%2d, colwidth:%2d \r\n",*dci,eeprombyteindex, column, colwidth);
		uart_puts(maintxtbuf);

}

void data_buf_dump(uint8_t *data_cache1)  // used immediately after data has been pulled from the EEPROM
{
	char sub_buf[8];
	uint16_t i=0;
		for(i=0; i<512; i++)
		{
			if((i%16)==0)
				uart_puts("\r\n");

			sprintf(sub_buf,"%3d,", data_cache1[i]);
			uart_puts(sub_buf);
		}

		uart_puts("\r\n");	
	
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
	char sub_buf[8];
	uint32_t i=0;
	uint8_t tempval;
eeprom_wakeup();
	for(i=0; i<512; i++)
	{
		if((i%16)==0)
		uart_puts("\r\n");
		tempval=read_eeprom(i);
		sprintf(sub_buf,"%3d,", tempval);
		//sprintf(sub_buf,"%3d,", read_eeprom((eeprombyteindex-i) -1));
		uart_puts(sub_buf);
	}

eeprom_deep_power_down();
	uart_puts("\r\n");
}

int main(void)
{
	char maintxtbuf[70]; // Screen can only handle 21.3 characters(put back to 24 after debug).
	uint8_t data_cache[515];
	uint8_t i2cdeviceaddresses[5]={0,0,0,0,0};
	volatile uint8_t i2cdevicesfound = 0, temp_val=0;
	uint8_t j=0, readerror=0, tempflipflop=0;
	uint32_t index32b=0;
	sensordata recent_measure;
	uint32_t DataCacheIndex=0, temp_dci=0;  //the Eeprom byte index keeps track of position within EEPROM
	uint16_t i=0, dynamic_buflen=512;
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
	
	uart_puts("Startup!\r\n");
	/*
	//Check the Bus for.... stuff
	i2cdevicesfound = scani2c(i2cdeviceaddresses);
	
	sprintf(maintxtbuf, "Devices found: %3d    ", i2cdevicesfound);
	drawline_str(10,maintxtbuf);
	
	drawline_str(20,"I2C Addresses         ");
	
	sprintf(maintxtbuf, "0x%02x 0x%02x 0x%02x 0x%02x", i2cdeviceaddresses[0],i2cdeviceaddresses[1],i2cdeviceaddresses[2],i2cdeviceaddresses[3]);
	drawline_str(30,maintxtbuf);
	*/
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
	
	//EEPROM init
	//drawline_str(50, "Testing EEPROM...     ");
	//eeprom_init();
	
	//Test EEPROM Connection/comms
	/*
	if(eeprom_test()){
		drawline_str(60, "EEPROM Passed! :-)    ");
		}else{
		
		drawline_str(60, "EEPROM Failed! >:-(   ");
	}
	

	for(i=5;i>0; i--)
	{
		sprintf(maintxtbuf, "Closing in %2d seconds",i);
		drawline_str(80,maintxtbuf);
		_delay_ms(1000);
	}
	*/
	//_delay_ms(1000);
	
	//eeprom fill
	/*
	while(index32b<512)
	{
		data_cache[(index32b)] = (index32b)&0xFF;
		
		//if((index32b%EEPROM_PAGE_SIZE)==0){
		//	write_eeprom(index32b-EEPROM_PAGE_SIZE,index32b/8);
		//}
		
		//write_eeprom(index32b,(index32b/256)&0xFF);
		index32b++;
		
	}	

	uart_puts("    0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF");
	for(index32b=0; index32b<512; index32b++)
	{
		if((index32b%EEPROM_PAGE_SIZE)==0){
		sprintf(maintxtbuf,"\r\n%3lu:", index32b/64);//prints line num
		uart_puts(maintxtbuf);
		}
		
		temp_val = data_cache[index32b];
		
		sprintf(maintxtbuf,"%2x", temp_val);
		uart_puts(maintxtbuf);
		
		if(((index32b%64)==63)){
			uart_puts("\r\n");
			uart_puts("    ");
		}

	}
	uart_puts("\r\n\r\n\n\n");
	*/
		
		
		/*
		data_cache[(index32b%EEPROM_PAGE_SIZE)] = index32b/29;
		
		if((index32b%EEPROM_PAGE_SIZE)==0){
			write_page_eeprom(index32b-EEPROM_PAGE_SIZE,data_cache,EEPROM_PAGE_SIZE-1);
		}
		index32b++;
		
	}
	*/
	//Sleep the EEPROM
	//eeprom_deep_power_down();

	//Enable RTC
	//TIMSK2  |= (1<<TOIE2);
	
	//uart_puts("End Inits\r\n");
	clearlcd();
	_delay_ms(50); // allow the UART buffer to flush (debug)
	while(1)
	{
		SMCR|=(1<<SE);
		asm volatile("sleep"::);
		SMCR &= ~(1<<SE);
		
		if(0)//if(sensormeasure==1)
		{
			
			SingleReadKXCJ9(&recent_measure);
			recent_measure.Pressure12BMP = bmp180_getpressure();
			recent_measure.temperature12BMP = bmp180_gettemperature();
			SHT2x_MeasureHM(HUMIDITY,&recent_measure);
			SHT2x_MeasureHM(TEMP,&recent_measure);
			recent_measure.lightLevel16 = bh1750_getlux();
			recent_measure.BattLevel10 = check_batt_level();
			recent_measure.humidity12SHT= (unsigned int)SHT2x_CalcRH(recent_measure.humidity12SHT);
			recent_measure.temperature12SHT = (int)SHT2x_CalcTemperatureC(recent_measure.temperature12SHT);
			/*sprintf(mainbuf, "$,%3d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d\r\n",maincounter++,
			recent_measure.Xaxis12, recent_measure.Yaxis12,	recent_measure.Zaxis12,\
			recent_measure.Pressure12bit,recent_measure.temperature12BMP, \
			recent_measure.humidity12, recent_measure.temperature12SHT,\
			recent_measure.lightLevel16,recent_measure.BattLevel10);
			uart_puts(mainbuf);
			*/
			
			//http://theboredengineers.com/2012/09/the-quadcopter-get-its-orientation-from-sensors/
			calc_RP(&recent_measure);
			
	
			sensormeasure = 0;
			
			data_cache[DataCacheIndex+0] = (recent_measure.Xaxis12 & 0x00FF); data_cache[DataCacheIndex+1] = ((recent_measure.Xaxis12 & 0xFF00)>>8);
			data_cache[DataCacheIndex+2] = (recent_measure.Yaxis12 & 0x00FF); data_cache[DataCacheIndex+3] = ((recent_measure.Yaxis12 & 0xFF00)>>8);
			data_cache[DataCacheIndex+4] = (recent_measure.Zaxis12 & 0x00FF); data_cache[DataCacheIndex+5] = ((recent_measure.Zaxis12 & 0xFF00)>>8);
			data_cache[DataCacheIndex+6] = (recent_measure.Pressure12BMP & 0x00FF); data_cache[DataCacheIndex+7] = ((recent_measure.Pressure12BMP & 0xFF00)>>8);
			data_cache[DataCacheIndex+8] = (recent_measure.humidity12SHT & 0x00FF); data_cache[DataCacheIndex+9] = ((recent_measure.humidity12SHT & 0xFF00)>>8);
			data_cache[DataCacheIndex+10] = (recent_measure.lightLevel16 & 0x00FF); data_cache[DataCacheIndex+11] = ((recent_measure.lightLevel16 & 0xFF00)>>8);
			data_cache[DataCacheIndex+12] = ((recent_measure.temperature12BMP)/10+127); data_cache[DataCacheIndex+13] = ((uint8_t)(recent_measure.temperature12SHT));
			data_cache[DataCacheIndex+14] = (recent_measure.BattLevel10 & 0x00FF); data_cache[DataCacheIndex+15] = ((recent_measure.BattLevel10 & 0xFF00)>>8);
	
			DataCacheIndex = DataCacheIndex + 16; //increment data_cache location
			
			if(DataCacheIndex>=dynamic_buflen)  // Write memory when data_cache full.
				savesensordata=1; //write it all out to sensor data!
		}
		
		if((sensormeasure==1) &&(showscreen<3))
		{
			sensormeasure = 0;
			temp_dci = DataCacheIndex;
			//sprintf(maintxtbuf, "Before SynFill->DataCacheIdx:%6lu, tempDCI:%6lu  EEPROMindex:%6lu\r\n",DataCacheIndex, temp_dci, eeprombyteindex);
			//uart_puts(maintxtbuf);
			synthetic_data(data_cache, &DataCacheIndex,dynamic_buflen);
			//sprintf(maintxtbuf, "After SynFill->DataCacheIdx:%6lu, tempDCI:%6lu  EEPROMindex:%6lu\r\n",DataCacheIndex, temp_dci, eeprombyteindex);
			//uart_puts(maintxtbuf);
			//data_buf_dump(data_cache);
		}
		
		if(savesensordata==1)//Write sensor data to EEprom
		{
			
			//sprintf(maintxtbuf, "Before-> EEprmByte_Idx:%6lu, DataCacheIdx:%6lu  \r\n",eeprombyteindex,DataCacheIndex);
			//uart_puts(maintxtbuf);
			//data_buf_dump(data_cache);
			if(DataCacheIndex>1){
				//uart_puts("PreWriteVerification\r\n");
				sprintf(maintxtbuf, "eeprombyteindex:%ld, DCI:%3ld\r\n", eeprombyteindex, DataCacheIndex);
				uart_puts(maintxtbuf);
				//data_buf_dump(data_cache);  //current buff contents
				uart_puts("Writing EEProm\r\n");
				dynamic_buflen = writeEEstorage(data_cache, DataCacheIndex);
				DataCacheIndex=0;
			//sprintf(maintxtbuf, "After-> EEprmByte_Idx:%6lu, DataCacheIdx:%6lu  \r\n",eeprombyteindex,DataCacheIndex);
			//	uart_puts(maintxtbuf);
			}
			savesensordata=0;
			//eeprom_wakeup();
			
			//sprintf(maintxtbuf, "beforedump_eeprombyteindex:%ld\r\n", eeprombyteindex);
			//uart_puts(maintxtbuf);
			//data_buf_dump(data_cache);  //current buff contents
			//uart_puts("Clearing local buffer\r\n");
			//clear_buffer(data_cache);
			//data_buf_dump(data_cache);  //current buff contents
			//write_page_eeprom(256, data_cache, 256-1);
			//uart_puts("EEProm Dump from buffer\r\n");
			//eeprom_dump();
			//eeprom_deep_power_down();

		}

		if(showscreen==1)  //not a screen, but shows contents of EEPROM
		{
			clearlcd();//shutdown screens
			//showscreen=0;
			sprintf(maintxtbuf, "EEprmByte_Idx:%6lu  ",eeprombyteindex);
			drawline_str(90, maintxtbuf);
			sprintf(maintxtbuf, "DataC_Idx:%6lu     ",DataCacheIndex);
			drawline_str(100, maintxtbuf);
			sprintf(maintxtbuf, "GlobalSecCt:%6lu   ",globalsecondcount);
			drawline_str(110, maintxtbuf);
			sprintf(maintxtbuf, "Showscreen:%4u    ",showscreen);
			drawline_str(120, maintxtbuf);
		}
		
		else if(showscreen==2)
		{
			clearlcd();
			stats_screen(maintxtbuf, &recent_measure,&t); //Current state of sensors, All Text	
		}
		
		else if(showscreen==3) 
		{
			retrieve_data(data_cache,1,10);
			data_buf_dump(data_cache);

			clearlcd();
			graph_screen("   ALS           ", " new     TIME     old", data_cache+256);//show graph screen of humidity
			sprintf(maintxtbuf, "SF:%3u                ",data_cache[127]);
			drawline_str(102, maintxtbuf);
			sprintf(maintxtbuf, "%4d,%4d     ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			sprintf(maintxtbuf, "Idx:%3u,%3u           ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		
		else if(showscreen==4)
		{
			retrieve_data(data_cache,1,8);
			data_buf_dump(data_cache);
		/*			
			for(index32b=0; index32b<512; index32b++)
			{
				if((index32b%EEPROM_PAGE_SIZE)==0){
					sprintf(maintxtbuf,"\r\n%2lu:", index32b/64);//prints line num
					uart_puts(maintxtbuf);
				}
						
				temp_val = data_cache[index32b];
				sprintf(maintxtbuf,"%3ld:%3x,",index32b, temp_val);
				uart_puts(maintxtbuf);
						
				if(((index32b%16)==15)){
					uart_puts("\r\n");
					uart_puts("    ");
				}

			}
			uart_puts("\r\n\r\n\n\n");
		*/
			clearlcd();
			graph_screen("   Humidity     ", " new     TIME     old", data_cache+256);//show graph screen of humidity
			sprintf(maintxtbuf, "SF:%3u                ",data_cache[127]);
			drawline_str(102, maintxtbuf);
			sprintf(maintxtbuf, "%4d,%4d     ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			sprintf(maintxtbuf, "Idx:%3u,%3u           ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
		}
		
		else if(showscreen==5)
		{	 			
			retrieve_data(data_cache,1,13);
			data_buf_dump(data_cache);
			/*	
			//uart_puts("    0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF");
			for(index32b=0; index32b<512; index32b++)
			{
				if((index32b%EEPROM_PAGE_SIZE)==0){
					sprintf(maintxtbuf,"\r\n%2lu: ", index32b/64);//prints line num
					uart_puts(maintxtbuf);
				}
					
				temp_val = data_cache[index32b];
					sprintf(maintxtbuf,"%3ld:%3x,",index32b, temp_val);
					uart_puts(maintxtbuf);
				
				if(((index32b%16)==15)){
					uart_puts("\r\n");
					uart_puts("    ");
				}
			}
			uart_puts("\r\n\r\n\n\n");
			*/
			clearlcd();
			graph_screen("   Temperature  ", " new     TIME     old", data_cache+256);//show graph screen of temperature
			sprintf(maintxtbuf, "SF:%3u                ",data_cache[127]);
			drawline_str(102, maintxtbuf);
			sprintf(maintxtbuf, "%4d,%4d     ",((int)(data_cache[126])<<8 )|(int)(data_cache[125]&0x00FF),((int)(data_cache[124])<<8 )|(int)(data_cache[123]&0x00FF));
			drawline_str(113, maintxtbuf);
			sprintf(maintxtbuf, "Idx:%3u,%3u           ",data_cache[122],data_cache[121]);
			drawline_str(121, maintxtbuf);
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
	globalsecondcount+=1;
	
	gettime(globalsecondcount, &t);
	
	//if((globalsecondcount%8)==0) //measure sensors every 8 seconds
		sensormeasure=1;
	
	//if(globalsecondcount%115200)  //check battery level every 36hours.
	//	checkbatt=1;

//if(screentimeout<=globalsecondcount)
//	showscreen=1;

}

ISR(PCINT1_vect) //sw2-right
{
	static int valid_sw2=1;

	if(valid_sw2){
		if(PINC & 0x02){
			showscreen = showscreen+1;
			if(showscreen>5)
				showscreen=1;
			valid_sw2=0;
		}
	}else{
		if(!(PINC & 0x02))
			valid_sw2=1;
	}
	_delay_ms(10);
	
	if(showscreen>2)
		savesensordata=1; //Save the data in the data_cache so the memory space can be used for the screens.

		//screentimeout= globalsecondcount+8;
	
}

ISR(PCINT2_vect) //sw1-left
{
	static int valid_sw1=1;

	if(valid_sw1){
		if((PIND & 0x20) &&(showscreen <3)){
			TIMSK2  ^= (1<<TOIE2); // Enable or disable the RTC
			valid_sw1=0;
		}
	}else{
		if(!(PIND & 0x20))
			valid_sw1=1;
	}
	_delay_ms(10);
	
}