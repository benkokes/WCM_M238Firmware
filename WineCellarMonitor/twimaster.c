/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI 
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
#include <stdio.h>
#include <inttypes.h>
#include <compat/twi.h>
#include <util/delay.h>

#include "twimaster.h"
#include "uart.h"


/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

/* I2C clock in Hz */
#define SCL_CLOCK  400000UL


/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void twi_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char twi_start(unsigned char address)
{
    uint8_t   twst;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

	return 0;

}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void twi_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char twi_rep_start(unsigned char address)
{
    return twi_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void twi_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char twi_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char twi_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char twi_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;

}/* i2c_readNak */



//The below 3 functions are a horrible hack for an I2C scanner -- don't judge me!!!
int8_t scani2c_start(uint8_t expected_status)
{
	uint8_t status;
	int ms_count = 0;

	/* send start condition to take control of the bus */
	TWCR = (_BV(TWINT)|_BV(TWSTA)|_BV(TWEN));//I2C_START;
	while (!(TWCR & _BV(TWINT)) && (ms_count < 1000))
	;

	if (ms_count >= 1000) {
		/*if (verbose) {
			i2c_error(s_i2c_start_error, TWCR, TWSR);
			i2c_error(s_i2c_timeout, TWCR, TWSR);
		}*/
		return -1;
	}

	/* verify start condition */
	status = TWSR;
	if (status != expected_status) {
		/*if (verbose) {
			i2c_error(s_i2c_start_error, TWCR, status);
		}*/
		return -1;
	}

	return 0;
}

int8_t scani2c_sla_rw(uint8_t device, uint8_t op, uint8_t expected_status)
{
	uint8_t sla_w;
	uint8_t status;
	int ms_count = 0;

	/* slave address + read/write operation */
	sla_w = (device << 1) | op;
	TWDR = sla_w;
	TWCR = (_BV(TWINT)|_BV(TWEN));//I2C_MASTER_TX;
	while (!(TWCR & _BV(TWINT)) && (ms_count < 1000))
	;

	/*if (ms_count >= I2C_TIMEOUT) {
		if (verbose) {
			i2c_error(s_i2c_sla_w_error, TWCR, TWSR);
			i2c_error(s_i2c_timeout, TWCR, TWSR);
		}
		return -1;
	}*/

	status = TWSR;
	if ((status & 0xf8) != expected_status) {
		/*if (verbose) {
			i2c_error(s_i2c_sla_w_error, TWCR, status);
		}*/
		return -1;
	}

	return 0;
}

int8_t scani2c_stop(void)
{
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWSTO);
	while (TWCR & _BV(TWSTO))
	;
	return 0;
}



uint8_t scani2c(uint8_t *dev_addys)
{
	uint8_t i;
	uint8_t n;
//	char scani2cbuf[46]; 
	n = 0;
	
	for (i=1; i<127; i++) {

		_delay_ms(5);
		/* start condition */
		if (scani2c_start(0x08)) {
			break;
		}
		
		/* address slave device, write */
		if (!scani2c_sla_rw(i, 0, 0x18)) {
			//sprintf(scani2cbuf,"found device at address 0x%02x,(%3d)\r\n", i,i);
			//uart_puts(scani2cbuf);
			dev_addys[n] = i;
			n++;
		}
		
		scani2c_stop();
		
		_delay_ms(5);
	}

	return n;
}