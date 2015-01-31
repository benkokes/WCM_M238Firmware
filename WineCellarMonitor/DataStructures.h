/*
 * DATASTRUCTURES.h
 *
 * Created: 1/21/2014 10:58:25 PM
 *  Author: bkokes
 */ 


#ifndef DATASTRUCTURES_H_
#define DATASTRUCTURES_H_
#include <avr/common.h>
typedef struct{
	uint8_t second;   //enter the current time, date, month, and year
	uint8_t minute;
	uint8_t hour;
	uint8_t mday;  //day of the month (1-31)
	uint8_t wday;  //day of the week (0-6, 0=Sunday)
	uint16_t yday;   //day of the year (0-365)
	uint8_t month;
	uint16_t year;
	uint8_t isdst;
}time;


typedef struct {//numbers attached to variables are bit designators
	int16_t Xaxis12;
	int16_t Yaxis12;
	int16_t Zaxis12;
	uint32_t Pressure12BMP;
	int8_t temperature12BMP;
	uint16_t humidity12SHT;
	int16_t temperature12SHT;
	uint16_t lightLevel16;
	uint16_t BattLevel10;
	int16_t roll;
	int16_t pitch;
}sensordata;




#endif /* DATASTRUCTURES_H_ */