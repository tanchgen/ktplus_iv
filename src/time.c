/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: time.c 4 2007-08-27 13:11:03Z xtimor $
 *
 * unix_time.c
 *      Author: Jet <g.tanchin@yandex.ru>
 *  Created on: 08 апр. 2016 г.
 */

#include <main.h>
#include "stm32f10x.h"
#include "my_time.h"
#include "stm32f10x_it.h"
#include "drive.h"
//#include "logger.h"
//#include "thermo.h"

volatile timer_ticks_t timer_delayCount;
volatile time_t uxTime;
volatile uint32_t myTick;
volatile uint32_t usDelFlag;

tDate sysDate;
tTime sysTime;

extern RCC_ClocksTypeDef RCC_Clocks;

uint32_t toReadCount;
uint8_t  secondFlag = FALSE;

/*
// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void timeInit( void ) {
	RTC_InitTypeDef rtcInitStruct;
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;

  RTC_StructInit( &rtcInitStruct );
  RTC_Init( &rtcInitStruct );
  //##-1- Configure the Date #################################################
  // Set Date: Wednesday June 1st 2016
  sdatestructure.RTC_Year = 16;
  sdatestructure.RTC_Month = RTC_Month_June;
  sdatestructure.RTC_Date = 1;
  sdatestructure.RTC_WeekDay = RTC_Weekday_Wednesday;

  if(RTC_SetDate( RTC_Format_BIN ,&sdatestructure ) != SUCCESS)
  {
    // Initialization Error
    genericError( GEN_ERR_HW );
  }

  stimestructure.RTC_Hours = 0;
  stimestructure.RTC_Minutes = 0;
  stimestructure.RTC_Seconds = 0;

  if(RTC_SetTime( rtcInitStruct.RTC_HourFormat ,&stimestructure ) != SUCCESS)
  {
    // Initialization Error
    genericError( GEN_ERR_HW );
  }

}
*/

// Получение системного мремени
uint32_t getTick( void ) {
	// Возвращает количество тиков
	return myTick;
}

uint32_t sys_now( void ){
	return myTick;
}
#define _TBIAS_DAYS		((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS		(_TBIAS_DAYS * (uint32_t)86400)
#define	_TBIAS_YEAR		0
#define MONTAB(year)		((((year) & 03) || ((year) == 0)) ? mos : lmos)

const int16_t	lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t	mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define	Daysto32(year, mon)	(((year - 1) / 4) + MONTAB(year)[mon])

/////////////////////////////////////////////////////////////////////

time_t xTm2Utime( tDate *mdate, tTime *mtime ){
	/* convert time structure to scalar time */
int32_t		days;
int32_t		secs;
int32_t		mon, year;

	/* Calculate number of days. */
	mon = mdate->Month - 1;
	year = mdate->Year - _TBIAS_YEAR;
	days  = Daysto32(year, mon) - 1;
	days += 365 * year;
	days += mdate->Date;
	days -= _TBIAS_DAYS;

	/* Calculate number of seconds. */
	secs  = 3600 * mtime->Hours;
	secs += 60 * mtime->Minutes;
	secs += mtime->Seconds;

	secs += (days * (time_t)86400);

	return (secs);
}

/////////////////////////////////////////////////////////////////////

void xUtime2Tm( tDate * mdate, tTime *mtime, time_t secsarg){
	uint32_t		secs;
	int32_t		days;
	int32_t		mon;
	int32_t		year;
	int32_t		i;
	const int16_t *	pm;

	#ifdef	_XT_SIGNED
	if (secsarg >= 0) {
			secs = (uint32_t)secsarg;
			days = _TBIAS_DAYS;
		} else {
			secs = (uint32_t)secsarg + _TBIAS_SECS;
			days = 0;
		}
	#else
		secs = secsarg;
		days = _TBIAS_DAYS;
	#endif

		/* days, hour, min, sec */
	days += secs / 86400;
	secs = secs % 86400;
	mtime->Hours = secs / 3600;
	secs %= 3600;
	mtime->Minutes = secs / 60;
	mtime->Seconds = secs % 60;

	mdate->WeekDay = (days + 1) % 7;

	/* determine year */
	for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
	days -= i;
	mdate->Year = year + _TBIAS_YEAR;

		/* determine month */
	pm = MONTAB(year);
	for (mon = 12; days < pm[--mon]; );
	mdate->Month = mon + 1;
	mdate->Date = days - pm[mon] + 1;
}

void timersHandler( void ) {

	// Decrement to zero the counter used by the delay routine.
  if (timer_delayCount != 0u) {
    --timer_delayCount;
  }

/*	// Таймаут для логгирования температуры
	if ( toLogCount > 1) {
		toLogCount--;
	}
*/
	// Таймаут для считывания температуры
	if ( toReadCount > 1) {
		toReadCount--;
	}

	// Секундный таймер
	if ( !(myTick % 1000) ) {
		secondFlag = TRUE;
		uxTime++;
		xUtime2Tm( &sysDate, &sysTime, uxTime );
	}


}

void timersProcess( void ) {
/*
	// Таймаут для логгирования температуры
	if ( toLogCount == 1 ) {
		toLogCount = toLogTout+1;
		toLogWrite();
	}
*/
	// Флаг "Прошла еще  одна секунда"
	if ( secondFlag ) {
		secondFlag = FALSE;
		if ( sysTime.Seconds == 0 ){
			// Каждую минуту отправляем текущее  положение задвижки
			canSendMsg( VALVE_DEG, valve.curDeg );
		}
	}
}

// Инициализация таймера микросекундных задержек
void debounceInit( void ) {

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	TIM_DeInit(DEBOUNCE_TIM);
	DEBOUNCE_TIM->CR1 &= ~TIM_CR1_CEN;
	// Выставляем счетчик на 0,5 мкс
	DEBOUNCE_TIM->PSC = (RCC_Clocks.PCLK2_Frequency/1000) - 1;			// Считаем миллисекунды
	DEBOUNCE_TIM->ARR = 49;								// До 50-и миллисекунд
	DEBOUNCE_TIM->CR1 &= ~TIM_CR1_CKD;
	DEBOUNCE_TIM->CR1 &= ~TIM_CR1_DIR;		// Считаем на возрастание
	DEBOUNCE_TIM->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ( DEBOUNCE_NVIC_IRQCHANNEL );
	NVIC_SetPriority( DEBOUNCE_NVIC_IRQCHANNEL, 0xF );
}

// Задержка в мс
void myDelay( uint32_t del ){
	uint32_t finish = myTick + del;
	while ( myTick < finish)
	{}
}

