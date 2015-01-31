/*
bmp180 lib 0x01
built by Ben Kokes 2014,  based on Davide Gironi's(2012) BMP085 driver code

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>

#include "bmp180.h"
#include "twimaster.h"
#include "i2c_core.h"

/*
 * i2c write
 */
/*
void bmp180_writemem(uint8_t reg, uint8_t value) {
	twi_start_wait(BMP180_ADDR | TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}
*/
void bmp180_writemem(uint8_t reg, uint8_t valbuf)
{
uint8_t sensorBuf[1];
sensorBuf[0] = valbuf;
i2c_write(BMP180_ADDR, reg, sensorBuf, 1);	
}

/*
 * i2c read
 */
/*
void bmp180_readmem(uint8_t reg, uint8_t buff[], uint8_t bytes) {
	uint8_t i =0;
	twi_start_wait(BMP180_ADDR | TWI_WRITE);
	twi_write(reg);
	twi_rep_start(BMP180_ADDR | TWI_READ);
	for(i=0; i<bytes; i++) {
		if(i==bytes-1)
			buff[i] = twi_readNak();
		else
			buff[i] = twi_readAck();
	}
	twi_stop();
}
*/
void bmp180_readmem(uint8_t reg, uint8_t *buff, uint8_t bytenum) 
{
	i2c_read(BMP180_ADDR, reg, buff, bytenum);
}

#if BMP180_FILTERPRESSURE == 1
#define BMP180_AVARAGECOEF 21
static long k[BMP180_AVARAGECOEF];
long bmp180_avaragefilter(long input) {
	uint8_t i=0;
	long sum=0;
	for (i=0; i<BMP180_AVARAGECOEF; i++) {
		k[i] = k[i+1];
	}
	k[BMP180_AVARAGECOEF-1] = input;
	for (i=0; i<BMP180_AVARAGECOEF; i++) {
		sum += k[i];
	}
	return (sum /BMP180_AVARAGECOEF) ;
}
#endif

/*
 * read calibration registers
 */
void bmp180_getcalibration(void) {
	uint8_t buff[2] = {0,0};
	//memset(buff, 0, sizeof(buff));

	bmp180_readmem(BMP180_REGAC1, buff, 2);
	bmp180_regac1 = ((int)buff[0] <<8 | ((int)buff[1]));
	bmp180_readmem(BMP180_REGAC2, buff, 2);
	bmp180_regac2 = ((int)buff[0] <<8 | ((int)buff[1]));
	bmp180_readmem(BMP180_REGAC3, buff, 2);
	bmp180_regac3 = ((int)buff[0] <<8 | ((int)buff[1]));
	bmp180_readmem(BMP180_REGAC4, buff, 2);
	bmp180_regac4 = ((unsigned int)buff[0] <<8 | ((unsigned int)buff[1]));
	bmp180_readmem(BMP180_REGAC5, buff, 2);
	bmp180_regac5 = ((unsigned int)buff[0] <<8 | ((unsigned int)buff[1]));
	bmp180_readmem(BMP180_REGAC6, buff, 2);
	bmp180_regac6 = ((unsigned int)buff[0] <<8 | ((unsigned int)buff[1]));
	bmp180_readmem(BMP180_REGB1, buff, 2);
	bmp180_regb1 = ((int)buff[0] <<8 | ((int)buff[1]));
	bmp180_readmem(BMP180_REGB2, buff, 2);
	bmp180_regb2 = ((int)buff[0] <<8 | ((int)buff[1]));
	bmp180_readmem(BMP180_REGMB, buff, 2);
	bmp180_regmb = ((int)buff[0] <<8 | ((int)buff[1]));
	bmp180_readmem(BMP180_REGMC, buff, 2);
	bmp180_regmc = ((int)buff[0] <<8 | ((int)buff[1]));
	bmp180_readmem(BMP180_REGMD, buff, 2);
	bmp180_regmd = ((int)buff[0] <<8 | ((int)buff[1]));
}

/*
 * get raw temperature as read by registers, and do some calculation to convert it
 */
void bmp180_getrawtemperature() {
	uint8_t buff[2] = {0,0};
	//memset(buff, 0, sizeof(buff));
	long ut,x1,x2;

	//read raw temperature
	bmp180_writemem(BMP180_REGCONTROL, BMP180_REGREADTEMPERATURE);
	_delay_ms(5); // min. 4.5ms read Temp delay
	bmp180_readmem(BMP180_REGCONTROLOUTPUT, buff, 2);
	ut = ((long)buff[0] << 8 | ((long)buff[1])); //uncompensated temperature value

	//calculate raw temperature
	x1 = ((long)ut - bmp180_regac6) * bmp180_regac5 >> 15;
	x2 = ((long)bmp180_regmc << 11) / (x1 + bmp180_regmd);
	bmp180_rawtemperature = x1 + x2;
}

/*
 * get raw pressure as read by registers, and do some calculation to convert it
 */
void bmp180_getrawpressure() {
	uint8_t buff[3];
	memset(buff, 0, sizeof(buff));
	long up,x1,x2,x3,b3,b6,p;
	unsigned long b4,b7;

	#if BMP180_AUTOUPDATETEMP == 1
	bmp180_getrawtemperature();
	#endif

	//read raw pressure
	bmp180_writemem(BMP180_REGCONTROL, BMP180_REGREADPRESSURE+(BMP180_MODE << 6));
	_delay_ms(2 + (3<<BMP180_MODE));
	bmp180_readmem(BMP180_REGCONTROLOUTPUT, buff, 3);
	up = ((((long)buff[0] <<16) | ((long)buff[1] <<8) | ((long)buff[2])) >> (8-BMP180_MODE)); // uncompensated pressure value

	//calculate raw pressure
	b6 = bmp180_rawtemperature - 4000;
	x1 = (bmp180_regb2* (b6 * b6) >> 12) >> 11;
	x2 = (bmp180_regac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((long)bmp180_regac1) * 4 + x3) << BMP180_MODE) + 2) >> 2;
	x1 = (bmp180_regac3 * b6) >> 13;
	x2 = (bmp180_regb1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (bmp180_regac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = ((uint32_t)up - b3) * (50000 >> BMP180_MODE);
	p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	bmp180_rawpressure = p + ((x1 + x2 + 3791) >> 4);

	#if BMP180_FILTERPRESSURE == 1
	bmp180_rawpressure = bmp180_avaragefilter(bmp180_rawpressure);
	#endif
}

/*
 * get celsius temperature
 */
int bmp180_gettemperature() { //double bmp180_gettemperature() {
	bmp180_getrawtemperature();
	int temperature = ((bmp180_rawtemperature + 8)>>4);//double temperature = ((bmp180_rawtemperature + 8)>>4);
	//temperature = temperature /10;
	return temperature;
}

/*
 * get pressure
 */
int32_t bmp180_getpressure() {
	bmp180_getrawpressure();
	return bmp180_rawpressure + BMP180_UNITPAOFFSET;
}

/*
 * get altitude
 */
double bmp180_getaltitude() {
	bmp180_getrawpressure();
	return ((1 - pow(bmp180_rawpressure/(double)101325, 0.1903 )) / 0.0000225577) + BMP180_UNITMOFFSET;
}

/*
 * init bmp180
 */
void bmp180_init() {
/*
	#if BMP180_I2CINIT == 1
	//init i2c
	i2c_init();
	_delay_us(10);
	#endif
*/
	bmp180_getcalibration(); //get calibration data
	bmp180_getrawtemperature(); //update raw temperature, at least the first time

	#if BMP180_FILTERPRESSURE == 1
	//initialize the avarage filter
	uint8_t i=0;
	for (i=0; i<BMP180_AVARAGECOEF; i++) {
		bmp180_getrawpressure();
	}
	#endif
}
