#include "bsp_led.h"

#define RCU_LED1        RCU_GPIOE
#define PORT_LED1        GPIOE
#define PIN_LED1        GPIO_PIN_3

// PD7
#define RCU_LED2        RCU_GPIOD
#define PORT_LED2        GPIOD
#define PIN_LED2        GPIO_PIN_7

/*
 * LED GPIO 初始化
 * */
void bsp_led_gpio_init()
{
        /* PE3 */
        rcu_periph_clock_enable(RCU_LED1);
        // 配置GPIO 输入输出模式
        gpio_mode_set(PORT_LED1, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED1);
        gpio_output_options_set(PORT_LED1, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PIN_LED1);

        /* PD7 */
        rcu_periph_clock_enable(RCU_LED2);
        gpio_mode_set(PORT_LED2, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED2);
        gpio_output_options_set(PORT_LED2, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PIN_LED2);
}

/**
 * LED 初始化
 */
void bsp_led_init()
{
        bsp_led_gpio_init();
}

void bsp_led_set(uint8_t led, uint8_t value)
{
        switch (led) {
        case 1:
                gpio_bit_write(PORT_LED1, PIN_LED1, value);
                break;
        case 2:
                gpio_bit_write(PORT_LED2, PIN_LED2, value);
                break;
        default:
                printf("led error:%d\r\n", led);
        }
}

void bsp_led_test()
{
        bsp_led_init();

	while (1)  {
		bsp_led_set(2, SET);        // LED1 ON
		delay_1ms(100);
		bsp_led_set(2, RESET);      // LED1 OFF
		delay_1ms(100);

		bsp_led_set(1, SET);        // LED1 ON
		delay_1ms(100);
		bsp_led_set(1, RESET);      // LED1 OFF
		delay_1ms(100);
	}
}
