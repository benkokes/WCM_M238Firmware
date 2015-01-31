/*
 * time.h
 *
 * Created: 2/10/2014 11:35:17 AM
 *  Author: bkokes
 */ 


#ifndef TIME_H_
#define TIME_H_

#include <avr/common.h>
#include "DataStructures.h"
void gmtime( uint32_t timer, time *timep );
uint32_t getSecsSinceEpoch(uint16_t epoch, uint8_t month, uint8_t day, uint8_t years, uint8_t hour, uint8_t minute, uint8_t second);
void elapsed_time(uint32_t seconds, time *tmstruct);

#endif /* TIME_H_ */