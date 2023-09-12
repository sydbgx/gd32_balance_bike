#ifndef __BSP_BATTERY_H
#define __BSP_BATTERY_H

#include "gd32f4xx.h"
#include <stdio.h>
#include "systick.h"

void bsp_battery_init(void );

float bsp_battery_get_voltage(void );

void bsp_battery_test(void );

#endif //__BSP_BATTERY_H
