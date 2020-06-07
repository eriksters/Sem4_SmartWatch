#ifndef __BSP_HEARTRATE_H
#define __BSP_HEARTRATE_H

#include <stdint.h>
#include "MAX30102.h"

void heartRate_init( void );

void i2c_init( void );

int32_t heartRate_getHeartRate( void );

#endif