/*
 * KXCJ9.c
 *
 * Created: 1/21/2014 10:40:56 PM
 *  Author: bkokes
 */ 
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include "twimaster.h"
#include "i2c_core.h"


#include "KXCJ9.h"


void SensorInitKXCJ9_int(void)
{
	unsigned char tempBuf[2];
	uint8_t error;
	//PORTB|=(1<<PORTB2);
	tempBuf[0] = 0x00; //set PC1 to 0 to enable changes
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = (0x40 | 0x00); //12bit mode,DRDY int, ORd with accel gain of 2G's
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = 0x30; //Enable External int, Active High
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1E, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = 0x03;		//Output Data Rate; 100Hz, LPF Roll-Off: 50Hz
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x21, tempBuf, 1))!=0)
	error=1;

	//enter operating mode
	tempBuf[0] = (0x80 | 0x40 | 0x20 | 0x00); //Enter Operating mode, 12bit mode,DRDY int, ORd with accel gain of 2G's
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;

}

void SingleReadKXCJ9(sensordata *sensor_struct)
{
	uint8_t sensorBuf[8];
	uint8_t error;
	sensorBuf[0] = 0x00; //set PC1 to 0 to enable changes
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, sensorBuf, 1))!=0)
	error=1;
	
	//save the ODR config
	if((i2c_read(KXCJ9_I2C_SLAVE_ADDRESS, 0x21, sensorBuf, 6)) != 0) // Data_CTRL_reg
	error=1;
	sensorBuf[7] = sensorBuf[0];  //(move to the last array element)
	
	//Speed up to 50Hz
	sensorBuf[0] = (0x02); 
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x21, sensorBuf, 1))!=0)// Data_CTRL_reg
	error=1;
	
	//enter operating mode
	sensorBuf[0] = (0x80 | 0x40); //Enter Operating mode, 12bit mode
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, sensorBuf, 1))!=0)
	error=1;
	
	_delay_ms(20);
	
	//Grab CJ9 data
	if((i2c_read(KXCJ9_I2C_SLAVE_ADDRESS, 0x06, sensorBuf, 6)) != 0) // fetch accel data
	error=1;

	sensor_struct->Xaxis12= (((int)sensorBuf[1]<<8) | (int)sensorBuf[0])>>4;	//assemble accel words
	sensor_struct->Yaxis12= (((int)sensorBuf[3]<<8) | (int)sensorBuf[2])>>4;
	sensor_struct->Zaxis12= (((int)sensorBuf[5]<<8) | (int)sensorBuf[4])>>4;
	
	sensorBuf[0] = 0x00; //set PC1 to 0 to enable changes
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, sensorBuf, 1))!=0)
	error=1;

	//Slow back down to 6.25Hz
	sensorBuf[0] = 0x0B;//sensorBuf[7];//restore
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x21, sensorBuf, 1))!=0)// Data_CTRL_reg
	error=1;		

	//enter operating mode
	sensorBuf[0] = (0x80 | 0x20 | 0x00); //Enter Operating mode, 8bit mode,WUFE int, ORd with accel gain of 2G's
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, sensorBuf, 1))!=0)
	error=1;
}

uint8_t KXCJ9_wake_init(uint8_t threshold, uint8_t sampwidth)
{
	
	unsigned char tempBuf[4];
	uint8_t error;
	//PORTB|=(1<<PORTB2);
	tempBuf[0] = 0x00; //set PC1 to 0 to enable changes
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = (0x40 | 0x20); //12bit mode,WUFE int with accel gain of 2G's
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;
	
	tempBuf[0] = 0x03; //WUF ODR 6.25Hz
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1D, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = 0x30; //Enable External int, Active High
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1E, tempBuf, 1))!=0)
	error=1;

	tempBuf[0] = 0x0B;		//Output Data Rate; 6.25Hz
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x21, tempBuf, 1))!=0)
	error=1;
	
	tempBuf[0] = sampwidth;		//Fed from passed parameters
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x29, tempBuf, 1))!=0)// Wakeup timer
	error=1;

	tempBuf[0] = threshold;		//Fed from passed parameters
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x6A, tempBuf, 1))!=0) //Wakeup Threshold
	error=1;

	//enter operating mode
	tempBuf[0] = (0x80 | 0x20 | 0x00); //Enter Operating mode, 8bit mode,WUFE int, ORd with accel gain of 2G's
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, tempBuf, 1))!=0)
	error=1;
	
	return 0;
}

void sleep_KXCJ9(void)
{
	uint8_t sensorBuf[2];
	uint8_t error;
	
	if((i2c_read(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, sensorBuf, 1)) != 0) // Read CTRL REG bit to preserve state.
		error=1;
	
	sensorBuf[0] &= ~(0x80); //flip PC1 bit to disable to goto standby mode
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, sensorBuf, 1))!=0)
		error=1;
}

void resume_KXCJ9(void)
{
	uint8_t sensorBuf[2];
	uint8_t error;
	
	//Incomplete function!!!! State should be saved before sleeping!!
		
	sensorBuf[0] = (0x80);
	if((i2c_write(KXCJ9_I2C_SLAVE_ADDRESS, 0x1B, sensorBuf, 1))!=0)
		error=1;
}