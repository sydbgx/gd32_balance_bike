#include "bsp_ultrasonic.h"
#include "bsp_timer.h"

// PB13 ECHO  读
#define RCU_ECHO        RCU_GPIOB
#define PORT_ECHO       GPIOB
#define PIN_ECHO        GPIO_PIN_13

// PB14 TRIG  发
#define RCU_TRIG        RCU_GPIOB
#define PORT_TRIG       GPIOB
#define PIN_TRIG        GPIO_PIN_14

void bsp_ultrasonic_gpio_init()
{
	// 初始化gpio ECHO
	rcu_periph_clock_enable(RCU_ECHO);
	gpio_mode_set(PORT_ECHO, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_ECHO);
	gpio_bit_write(PORT_ECHO, PIN_ECHO, RESET);

	/* 发 */
	rcu_periph_clock_enable(RCU_TRIG);
	gpio_mode_set(PORT_TRIG, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PIN_TRIG);
	gpio_output_options_set(PORT_TRIG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PIN_TRIG);
	gpio_bit_write(PORT_TRIG, PIN_TRIG, RESET);
}

void bsp_ultrasonic_init()
{
	bsp_ultrasonic_gpio_init();
}

/* 获取距离 */
float bsp_ultrasonic_get()
{
	// 1. 等设置TRIG引脚为高电平
	gpio_bit_write(PORT_TRIG, PIN_TRIG, SET);
	// 2. 至少等待20us
	delay_1us(20);
	// 3. 设置TRIG引脚为低电平
	gpio_bit_write(PORT_TRIG, PIN_TRIG, RESET);
	// 4. 等待ECHO信号由低电平变为高电平
	while (gpio_input_bit_get(PORT_ECHO, PIN_ECHO) == RESET);
	// 5. 开始计时
	bsp_timer_open();
	// 6. 等待ECHO信号由高电平变为低电平
	while (gpio_input_bit_get(PORT_ECHO, PIN_ECHO) == SET);
	// 7.关闭计时
	bsp_timer_close();

	// 8. 计算时间差
	uint32_t time = bsp_timer_get();

	/* 9. 计算距离 340m/s
	 * 340m/s = 34000cm/1000ms = 34000cm/1000000us = 0.034cm/us
	 * */
	float distance = 0.034 * time / 2;
	return distance;
}

/* 测试次数，取测量平均值，相当于数字滤波器（0~15） */
float bsp_ultrasonic_getn(int count)
{
	// TODO
	float sum = 0;
	int i = count;
	while (i > 0) {
		sum += bsp_ultrasonic_get();
		delay_1ms(10);
		i--;
	}
	return sum / count;
}

void bsp_ultrasonic_test()
{
	bsp_timer_init();
	bsp_ultrasonic_init();

	while (1) {
		float distance = bsp_ultrasonic_getn(5);
		printf("distance: %f \r\n", distance);
		delay_1ms(500);
	}
}