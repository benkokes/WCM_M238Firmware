/*
 * KXCJ9.h
 *
 * Created: 1/21/2014 10:41:20 PM
 *  Author: bkokes
 */ 


#ifndef KXCJ9_H_
#define KXCJ9_H_
//8bit Address for CJ9 0x1C when ADDR=0, 0x1E for ADDR=1;
#define KXCJ9_I2C_SLAVE_ADDRESS   	0x1E //0x0E in 7-bit, 0x1C W in 8-bit mode - 60d
#include "DataStructures.h"

void SensorInitKXCJ9_int(void);
void SingleReadKXCJ9(sensordata *sensor_struct);
uint8_t KXCJ9_wake_init(uint8_t threshold, uint8_t sampwidth);
void sleep_KXCJ9(void);
void resume_KXCJ9(void);

#endif /* KXCJ9_H_ */