/*
 * spi.h
 *
 * Created: 1/30/2014 4:32:04 PM
 *  Author: bkokes
 */ 


#ifndef SPI_H_
#define SPI_H_

uint8_t spi_transfer(volatile uint8_t c);
void init_spi(void);
void spi_highspeed(void);
void spi_regularspeed(void);

#endif /* SPI_H_ */