#ifndef BSP_ULTRASONIC_H
#define BSP_ULTRASONIC_H
#include <stdint.h>
#include <stdio.h>
#include <systick.h>
#include "gd32f4xx.h"

void bsp_ultrasonic_init();
float bsp_ultrasonic_get();
float bsp_ultrasonic_getn(int count);
void bsp_ultrasonic_test();

#endif //BSP_ULTRASONIC_H
