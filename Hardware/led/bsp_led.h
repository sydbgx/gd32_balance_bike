#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdio.h>
#include "gd32f4xx.h"
#include "systick.h"

void bsp_led_init();

void bsp_led_set(uint8_t led, uint8_t value);

void bsp_led_test();

#endif //BSP_LED_H
