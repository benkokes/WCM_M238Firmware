/*******************************************************************************/
/*  Copyright (c) 2010 InvenSense Corporation, All Rights Reserved.            */   
/*******************************************************************************/

#ifndef __I2C_CORE_H__
#define __I2C_CORE_H__

#include <stdint.h>

#define TRUE	1
#define	FALSE	0

// CPU speed, BAUDRATE 400kHz and Baudrate Register Settings
//#define CPU_SPEED    1000000UL
#define CPU_SPEED    8000000UL
#define BAUDRATE     400000				//	I2C Baudrate
#define TWI_BAUD	(CPU_SPEED / (2 * BAUDRATE) - 5 )

// Transaction status defines.
#define I2C_RDY					0
#define I2C_BUSY				1
#define I2C_DONE				2

enum _I2C_ERROR_CODES
{
    I2C_NULL_ERROR = 0,
    I2C_UNKNOWN,I2C_ARB_LOST,I2C_BUS_ERROR,I2C_NACK,I2C_BUF_OVR
};

//void i2c_core_init(void);
//void i2c_master_init(void);
//void i2c_core_isr(void);

unsigned char i2c_write(unsigned char dev_addr, unsigned char reg_addr,
                        unsigned char *pbuf, unsigned char wlen);  //was: const unsigned char *pbuf, unsigned char wlen);                               


unsigned char i2c_read(unsigned char dev_addr, unsigned char reg_addr,
                       unsigned char *pbuf,unsigned int rd_len);

//#define ML_I2C_ERROR 1
//#define ML_FIFO_ERROR 2
//void i2c_catch(unsigned char i2c_returncode, uint8_t *errflag);

//#ifdef DEBUG_I2C_TRANSACTIONS
//void enable_i2c_debugging(void);
//void disable_i2c_debugging(void);
//#endif

#endif // __I2C_CORE_H__
