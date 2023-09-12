#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "gd32f4xx.h"
#include <stdio.h>
#include "systick.h"

void bsp_motor_init(void );

void bsp_motor_set(int left, int right);

void bsp_motor_test(void );

#endif //__BSP_MOTOR_H
