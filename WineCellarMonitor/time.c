/*
 * time.c
 *
 * Created: 2/10/2014 11:35:04 AM
 *  Author: bkokes
 */ 

#include "time.h"

//#define FIRSTYEAR 0
//const uint8_t DayOfMonth[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

#define YEAR0                   1900
#define EPOCH_YR                1970
#define SECS_DAY                (24L * 60L * 60L)
#define LEAPYEAR(year)          (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)          (LEAPYEAR(year) ? 366 : 365)
#define FIRSTSUNDAY(timp)       (((timp)->yday - (timp)->wday + 420) % 7)
#define FIRSTDAYOF(timp)        (((timp)->wday - (timp)->yday + 420) % 7)

const int16_t _ytab[2][12] = {
	{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
	{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

void gmtime(uint32_t timer, time *tmbuf) {  // convert seconds count to calendar format.
	//time_t time = *timer;
	uint32_t dayclock, dayno;
	int year = EPOCH_YR;

	dayclock = (uint32_t)timer % SECS_DAY;
	dayno = (uint32_t)timer / SECS_DAY;

	tmbuf->second = dayclock % 60;
	tmbuf->minute = (dayclock % 3600) / 60;
	tmbuf->hour = dayclock / 3600;
	tmbuf->wday = (dayno + 4) % 7; // Day 0 was a Thursday
	while (dayno >= (unsigned long) YEARSIZE(year)) {
		dayno -= YEARSIZE(year);
		year++;
	}
	tmbuf->year = year - YEAR0;  // calculates offset from year 1900.  This is the value stored in the Struct
	tmbuf->yday = dayno;
	tmbuf->month = 0;			//January is month 0.
	while (dayno >= (unsigned long) _ytab[LEAPYEAR(year)][tmbuf->month]) {
		dayno -= _ytab[LEAPYEAR(year)][tmbuf->month];
		tmbuf->month++;
	}
	tmbuf->mday = dayno + 1;
	tmbuf->isdst = 0;
		
}

/*	
void gettime( uint32_t sec, time *t )
{
	uint16_t day;
	uint8_t year;
	uint16_t dayofyear;
	uint8_t leap400;
	uint8_t month;

	t->second = sec % 60;
	sec /= 60;
	t->minute = sec % 60;
	sec /= 60;
	t->hour = sec % 24;
	day = sec / 24;

	// t->wday = (day + FIRSTDAY) % 7;		// weekday

	year = FIRSTYEAR % 100;			// 0..99
	leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3);	// 4, 3, 2, 1

	for(;;){
		dayofyear = 365;
		if( (year & 3) == 0 ){
			dayofyear = 366;					// leap year
			if( year == 0 || year == 100 || year == 200 )	// 100 year exception
			if( --leap400 )					// 400 year exception
			dayofyear = 365;
		}
		if( day < dayofyear )
		break;
		day -= dayofyear;
		year++;					// 00..136 / 99..235
	}
	t->year = year + FIRSTYEAR / 100 * 100;	// + century

	if( dayofyear & 1 && day > 58 )		// no leap year and after 28.2.
	day++;					// skip 29.2.

	for( month = 1; day >= DayOfMonth[month-1]; month++ )
	day -= DayOfMonth[month-1];

	t->month = month;				// 1..12
	t->day = day + 1;				// 1..31
}

*/
uint32_t getSecsSinceEpoch(uint16_t epoch, uint8_t month, uint8_t day, uint8_t years, uint8_t hour, uint8_t minute, uint8_t second)
{
	
	unsigned long secs = 0;
	int countleap = 0;
	int i;
	int dayspermonth;

	secs = years * (86400UL * 365UL);
	for (i = 0; i < (years - 1); i++)
	{
		if (LEAPYEAR((epoch + i)))
		countleap++;
	}
	secs += (countleap * 86400UL);

	secs += second;
	secs += (hour * 3600UL);
	secs += (minute * 60UL);
	secs += ((day - 1) * 86400UL);

	if (month > 1)
	{
		dayspermonth = 0;

		if (LEAPYEAR((epoch + years))) // Only counts when we're on leap day or past it
		{
			if (month > 2)
			{
				dayspermonth = 1;
				} else if (month == 2 && day >= 29) {
				dayspermonth = 1;
			}
		}

		for (i = 0; i < (month - 1); i++)
		{
			secs += (_ytab[dayspermonth][i] * 86400UL);
		}
	}

	return secs;
}
/*
char not_leap(void)      //check for leap year
{
	if (!(t.year%100))
	return (char)(t.year%400);
	else
	return (char)(t.year%4);
}
*/

void elapsed_time(uint32_t seconds, time *tmstruct)
{
	tmstruct->year = seconds/(SECS_DAY*365);
	//tmstruct->month  not populated because of variable day-widths of each month.
	tmstruct->mday = seconds/SECS_DAY; 
	tmstruct->hour = (seconds/3600) - ((tmstruct->mday) * 24);
	tmstruct->minute = seconds/60 - (tmstruct->mday * 24) - (tmstruct->hour * 24);
	tmstruct->second = seconds%60;
	
	
}