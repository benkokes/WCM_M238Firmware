/*
bmp180 lib 0x01

built by Ben Kokes 2014,  based on Davide Gironi's(2012) BMP085 driver code

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  - this library is a porting of the bmp085driver 0.4 ardunio library
    http://code.google.com/p/bmp085driver/

Notes:
  - to compile with avrgcc you may define -lm flag for some math functions
*/


#ifndef BMP180_H_
#define BMP180_H_

#include <stdio.h>
#include <avr/io.h>

#define BMP180_ADDR (0x77<<1) //0x77 default I2C address

//registers
#define BMP180_REGAC1 0xAA
#define BMP180_REGAC2 0xAC
#define BMP180_REGAC3 0xAE
#define BMP180_REGAC4 0xB0
#define BMP180_REGAC5 0xB2
#define BMP180_REGAC6 0xB4
#define BMP180_REGB1 0xB6
#define BMP180_REGB2 0xB8
#define BMP180_REGMB 0xBA
#define BMP180_REGMC 0xBC
#define BMP180_REGMD 0xBE
#define BMP180_REGCONTROL 0xF4 //control
#define BMP180_REGCONTROLOUTPUT 0xF6 //output 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB
#define BMP180_REGREADTEMPERATURE 0x2E //read temperature
#define BMP180_REGREADPRESSURE 0x34 //read pressure

//modes
#define BMP180_MODEULTRALOWPOWER 0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define BMP180_MODESTANDARD 1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define BMP180_MODEHIGHRES 2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define BMP180_MODEULTRAHIGHRES 3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25

//autoupdate temperature enabled
#define BMP180_AUTOUPDATETEMP 1 //autoupdate temperature every read

//setup parameters
#define BMP180_MODE BMP180_MODEHIGHRES //define a mode
#define BMP180_UNITPAOFFSET 0 //define a unit offset (pa)
#define BMP180_UNITMOFFSET 0 //define a unit offset (m)

//avarage filter
#define BMP180_FILTERPRESSURE 1 //avarage filter for pressure

//variables
int bmp180_regac1, bmp180_regac2, bmp180_regac3, bmp180_regb1, bmp180_regb2, bmp180_regmb, bmp180_regmc, bmp180_regmd;
unsigned int bmp180_regac4, bmp180_regac5, bmp180_regac6;
long bmp180_rawtemperature, bmp180_rawpressure;

//functions (removed 'extern' from all 4)
extern void bmp180_init();
int32_t bmp180_getpressure();
extern double bmp180_getaltitude();
extern int bmp180_gettemperature(); //extern double bmp180_gettemperature();

#endif
