/*
bh1750 lib 0x01

copyright (c) Davide Gironi, 2013
Website: http://code.google.com/p/davidegironi/downloads/detail?name=avr_lib_bh1750_01.zip
Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/


#ifndef BH1750_H_
#define BH1750_H_


#define BH1750_ADDR (0x5C<<1) //device address

//i2c settings
#define BH1750_I2CINIT 1 //init i2c

//resolution modes
#define BH1750_MODEH 0x10 //continuously h-resolution mode, 1lx resolution, 120ms
#define BH1750_MODEH2 0x11 //continuously h-resolution mode, 0.5lx resolution, 120ms
#define BH1750_MODEL 0x13 //continuously l-resolution mode, 4x resolution, 16ms
#define BH1750_MODEH_OT 0x20 //OneTime h-resolution mode, 1lx resolution, 120ms
#define BH1750_MODEH2_OT 0x21 //OntTime h-resolution mode, 0.5lx resolution, 120ms
#define BH1750_MODEL_OT 0x23 //OneTime l-resolution mode, 4x resolution, 16ms
//define active resolution mode
#define BH1750_MODE BH1750_MODEH_OT

//Modes
#define BH1750_POWERUP 0x01
#define BH1750_POWERDOWN 0x00

//Reset sequence
#define BH1750_RESET 0x07

//functions
extern void bh1750_init();
extern unsigned int bh1750_getlux();


#endif
