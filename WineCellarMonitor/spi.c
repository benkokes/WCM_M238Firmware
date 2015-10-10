/*
 * spi.c
 *
 * Created: 1/30/2014 4:31:38 PM
 *  Author: bkokes
 */ 
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include "spi.h"

uint8_t spi_transfer(volatile uint8_t c) {
	SPDR = c;

	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)))
	{};

	c = SPDR;
	return c;
}

void init_spi(void)
{
	SPCR |= (1<<MSTR);           //Master SPI mode
	
	SPCR |= (1<<SPR0); //Set Fosc/8 clock rate
	SPCR &= ~(1<<SPR1);
	
	SPSR |= (1<<SPI2X);
	
	SPCR &= ~(1<<CPOL); //leading edge is rising
	SPCR &= ~(1<<CPHA); //leading edge sample.
	
	SPCR |= (1<<SPE);            //SPI enable

}

void spi_highspeed(void)
{
	SPCR &= ~(1<<SPE); 
	
	SPSR &= ~(1<<SPI2X);//Set Fosc/4 clock rate
	SPCR &= ~(1<<SPR0);
	SPCR &= ~(1<<SPR1);
	
	SPCR |= (1<<SPE); 
}


void spi_regularspeed(void)
{
	SPCR &= ~(1<<SPE); 
	
	SPSR |= (1<<SPI2X);//Set Fosc/8 clock rate
	SPCR |= (1<<SPR0); 
	SPCR &= ~(1<<SPR1);
	
	SPCR |= (1<<SPE); 
}