/*
 * sharplcd.h
 *
 * Created: 1/30/2014 4:29:17 PM
 *  Author: bkokes
 */ 


#ifndef SHARPLCD_H_
#define SHARPLCD_H_

void drawline_str(uint8_t rowaddr, char *textbuf);
void clearlcd(void);
void largechardraw_str(uint8_t rowaddr, char *textbuf);
void graph_screen(char* scr_title, char* units, uint8_t* databuf);

#endif /* SHARPLCD_H_ */