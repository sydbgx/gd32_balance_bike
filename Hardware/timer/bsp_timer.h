#ifndef BSP_TIMER_H
#define BSP_TIMER_H
#include "gd32f4xx_timer.h"
#include <stdio.h>
#include "systick.h"

/* 初始化 */
void bsp_timer_init();
/* 打开定时器 */
void bsp_timer_open();
/* 关闭定时器 */
void bsp_timer_close();
/* 获取当前时间 */
int bsp_timer_get();
void bsp_timer_test();

#endif //BSP_TIMER_H
