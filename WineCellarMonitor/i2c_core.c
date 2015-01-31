/* 
	Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.    
*/
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c_core.h"
#include "twimaster.h"
//#include "hw_helper.h"



#include <stdio.h>
//extern char str_buf[];
uint8_t debug_enabled = 0;
//void enable_i2c_debugging(void) { debug_enabled = 1; }
//void disable_i2c_debugging(void) { debug_enabled = 0; }

// define local globals

unsigned char volatile i2c_state;
unsigned char volatile i2c_error;
unsigned char volatile i2c_addr;
unsigned char volatile i2c_wrlen;
unsigned char volatile i2c_rdlen;
unsigned char volatile i2c_wr_count;
unsigned char volatile i2c_rd_count;
unsigned char volatile i2c_cmd;

unsigned char volatile i2c_buf[8];
volatile unsigned char i2c_sts;

// identify the current I2C transaction
#define I2C_NULL_CMD	0
#define I2C_WRITE_CMD	1			
#define I2C_READ_CMD	2
/*
void i2c_catch(unsigned char i2c_returncode, unsigned char *errflag)
{
    if (i2c_returncode > I2C_NULL_ERROR)
        *errflag |= ML_I2C_ERROR;
}
*/
unsigned char i2c_write(unsigned char dev_addr, unsigned char reg_addr,
                        unsigned char *pbuf, unsigned char wlen) // was: const unsigned char *pbuf, unsigned char wlen)
{
 unsigned char i=0, retval;



	twi_start_wait(dev_addr+TWI_WRITE);     // set device address and write mode
	retval = twi_write(reg_addr);                        // register address

	// copy/send write buffer 
	for(i = 0;i < wlen;i++)
	{	
		 i2c_buf[i] = (*(pbuf + i));
		twi_write(i2c_buf[i]);
	}

	twi_stop();
	
return(retval);


}

unsigned char i2c_read(unsigned char dev_addr, unsigned char reg_addr,
                       unsigned char *pbuf,unsigned int rd_len)
{
	unsigned char i=0, retval;

  
 	twi_start_wait(dev_addr+TWI_WRITE);     // set device address and write mode
	retval = twi_write(reg_addr);                        // register address
	retval = twi_rep_start(dev_addr+TWI_READ);       	// set device address and read mode
	
	if(rd_len <2)
	{	
		i2c_buf[0] = twi_readNak();                   	//single byte read
		twi_stop();
	}
	else
	{
		for(i=0; i<(rd_len-1); i++)
		{
			i2c_buf[i] = twi_readAck();				//Burst read
		}
		i2c_buf[i] = twi_readNak();					//Read final byte
		twi_stop();
	}
	// copy/clear read buffer 
	for(i = 0;i < rd_len;i++)
	{	
		*(pbuf + i) = i2c_buf[i];
		i2c_buf[i] = 0;
	}

return(retval);

	return(i2c_error);
}




