#include "bsp_key.h"

#define RCU_KEY1 RCU_GPIOE
#define PORT_KEY1 GPIOE
#define PIN_KEY1 GPIO_PIN_5

#define RCU_KEY2 RCU_GPIOE
#define PORT_KEY2 GPIOE
#define PIN_KEY2 GPIO_PIN_6

#define KEY1_EXTI EXTI_5
#define PIN_KEY1_EXTI EXTI_SOURCE_PIN5

#define KEY2_EXTI EXTI_6
#define PIN_KEY2_EXTI EXTI_SOURCE_PIN6

void bsp_key_gpio_init()
{
	/* KEY1 */
	rcu_periph_clock_enable(RCU_KEY1);
	gpio_mode_set(PORT_KEY1, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_KEY1);
	// 时钟配置
	rcu_periph_clock_enable(RCU_SYSCFG);
	// 配置中断源
	syscfg_exti_line_config(EXTI_SOURCE_GPIOE, PIN_KEY1_EXTI);
	// 中断初始化
	exti_init(KEY1_EXTI, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	// 配置中断优先级
	nvic_irq_enable(EXTI5_9_IRQn, 1, 1);
	// 使能中断
	exti_interrupt_enable(KEY1_EXTI);
	// 清除中断标志位
	exti_interrupt_flag_clear(KEY1_EXTI);

	/* KEY2 */
	rcu_periph_clock_enable(RCU_KEY2);
	gpio_mode_set(PORT_KEY2, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_KEY2);
	// 时钟配置
	rcu_periph_clock_enable(RCU_SYSCFG);
	// 配置中断源
	syscfg_exti_line_config(EXTI_SOURCE_GPIOE, PIN_KEY2_EXTI);
	// 中断初始化
	exti_init(KEY2_EXTI, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	// 配置中断优先级
	nvic_irq_enable(EXTI5_9_IRQn, 1, 1);
	// 使能中断
	exti_interrupt_enable(KEY2_EXTI);
	// 清除中断标志位
	exti_interrupt_flag_clear(KEY2_EXTI);
}

void bsp_key_init()
{
	bsp_key_gpio_init();
}

/* 中断函数 */
void EXTI5_9_IRQHandler(void)
{
	// 判断中断中断触发
	if(SET == exti_interrupt_flag_get(KEY1_EXTI)) {
		if(gpio_input_bit_get(PORT_KEY1, PIN_KEY1) == RESET) {
			bsp_key_callback(1, SET);
		} else {
			bsp_key_callback(1, RESET);
		}
	}
	// 清除中断标志位
	exti_interrupt_flag_clear(KEY1_EXTI);

	if(SET == exti_interrupt_flag_get(KEY2_EXTI)) {
		if(gpio_input_bit_get(PORT_KEY2, PIN_KEY2) == RESET) {
			bsp_key_callback(2, SET);
		} else {
			bsp_key_callback(2, RESET);
		}
	}
	exti_interrupt_flag_clear(KEY2_EXTI);
}

/* 按下按键触发任务 */
void bsp_key_callback(uint8_t key, uint8_t value)
{
        // 按下按键1清空编码器
        if (key == 1) {
                timer_counter_value_config(TIMER1, 0);
                timer_counter_value_config(TIMER2, 0);
        }
}

void bsp_key_test()
{
	bsp_key_init();
	while (1) {}
}
