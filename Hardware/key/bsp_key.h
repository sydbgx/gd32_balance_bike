#ifndef __BSP_KEY_H
#define __BSP_KEY_H

//#include <stdint.h>
#include <stdio.h>
#include "gd32f4xx.h"
#include "systick.h"

void bsp_key_init();

void bsp_key_callback(uint8_t key, uint8_t value);

void bsp_key_test();

#endif //__BSP_KEY_H
