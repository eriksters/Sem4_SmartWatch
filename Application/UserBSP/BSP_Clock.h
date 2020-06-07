#ifndef BSP_CLOCK_H
#define BSP_CLOCK_H

#include "stm32f4xx_hal.h"
#include "_ApplicationDateTime.h"

extern RTC_HandleTypeDef      RtcHandle;

void clock_init( void );


void clock_getDateTime( ApplicationDateTime );

void clock_setDateTime( ApplicationDateTime );


char clock_getMonth( void );

char clock_getWeekDay( void );

char clock_getDate( void );

char clock_getHour( void );

char clock_getMinute( void );

char clock_getSecond( void );


void clock_setMonth( char );

void clock_setWeekDay( char );

void clock_setDate( char );

void clock_setHour( char );

void clock_setMinute( char );

void clock_setSecond( char );

#endif