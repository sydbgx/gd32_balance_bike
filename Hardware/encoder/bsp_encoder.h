#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "gd32f4xx.h"
#include <stdio.h>
#include "systick.h"
/* 初始化 */
void bsp_encoder_init(void );
/* 获取左边电机的转速 */
short bsp_encoder_get_left(void );
/* 获取右边电机的转速 */
short bsp_encoder_get_right(void );
void bsp_encoder_test(void );

#endif //__BSP_ENCODER_H
