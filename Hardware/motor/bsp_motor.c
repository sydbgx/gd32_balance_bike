#include "bsp_motor.h"

#define RCU_MOTOR_L1	RCU_GPIOA
#define PORT_MOTOR_L1 	GPIOA
#define PIN_MOTOR_L1	GPIO_PIN_0
#define AF_MOTOR_L1	GPIO_AF_2

#define RCU_MOTOR_L2	RCU_GPIOA
#define PORT_MOTOR_L2 	GPIOA
#define PIN_MOTOR_L2	GPIO_PIN_1
#define AF_MOTOR_L2	GPIO_AF_2

#define RCU_MOTOR_R1	RCU_GPIOA
#define PORT_MOTOR_R1  	GPIOA
#define PIN_MOTOR_R1   	GPIO_PIN_2
#define AF_MOTOR_R1   	GPIO_AF_2

#define RCU_MOTOR_R2	RCU_GPIOB
#define PORT_MOTOR_R2 	GPIOA
#define PIN_MOTOR_R2   	GPIO_PIN_3
#define AF_MOTOR_R2	GPIO_AF_2

#define RCU_MOTOR_TIMER  RCU_TIMER4
#define PORT_MOTOR_TIMER TIMER4

void bsp_motor_gpio_init(void)
{
	rcu_periph_clock_enable(RCU_MOTOR_L1);
	gpio_mode_set(PORT_MOTOR_L1, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PIN_MOTOR_L1);
	gpio_af_set(PORT_MOTOR_L1, AF_MOTOR_L1, PIN_MOTOR_L1);
	gpio_output_options_set(PORT_MOTOR_L1, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PIN_MOTOR_L1);
	gpio_bit_write(PORT_MOTOR_L1, PIN_MOTOR_L1, RESET);

	rcu_periph_clock_enable(RCU_MOTOR_L2);
	gpio_mode_set(PORT_MOTOR_L2, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PIN_MOTOR_L2);
	gpio_af_set(PORT_MOTOR_L2, AF_MOTOR_L2, PIN_MOTOR_L2);
	gpio_output_options_set(PORT_MOTOR_L2, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PIN_MOTOR_L2);
	gpio_bit_write(PORT_MOTOR_L2, PIN_MOTOR_L2, RESET);

	rcu_periph_clock_enable(RCU_MOTOR_R1);
	gpio_mode_set(PORT_MOTOR_R1, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PIN_MOTOR_R1);
	gpio_af_set(PORT_MOTOR_R1, AF_MOTOR_R1, PIN_MOTOR_R1);
	gpio_output_options_set(PORT_MOTOR_R1, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PIN_MOTOR_R1);
	gpio_bit_write(PORT_MOTOR_R1, PIN_MOTOR_R1, RESET);

	rcu_periph_clock_enable(RCU_MOTOR_R2);
	gpio_mode_set(PORT_MOTOR_R2, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PIN_MOTOR_R2);
	gpio_af_set(PORT_MOTOR_R2, AF_MOTOR_R2, PIN_MOTOR_R2);
	gpio_output_options_set(PORT_MOTOR_R2, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, PIN_MOTOR_R2);
	gpio_bit_write(PORT_MOTOR_R2, PIN_MOTOR_R2, RESET);
}

void bsp_motor_timer_init(void)
{
	rcu_periph_clock_enable(RCU_MOTOR_TIMER);
	// 开启倍频
	rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
	// 复位定时器
	timer_deinit(PORT_MOTOR_TIMER);
	// 设置定时器
	timer_parameter_struct timer_initpara;
	// 对参数初始化
	timer_struct_para_init(&timer_initpara);
	timer_initpara.prescaler         = 240 - 1;
	timer_initpara.period            = 10000 - 1;

	timer_init(PORT_MOTOR_TIMER, &timer_initpara);

	// 定时器输出参数配置
	timer_oc_parameter_struct timer_oc_initpara;
	timer_channel_output_struct_para_init(&timer_oc_initpara);
	timer_oc_initpara.outputstate = TIMER_CCX_ENABLE;
	// 将参数配置给通道
	timer_channel_output_config(PORT_MOTOR_TIMER, TIMER_CH_0,&timer_oc_initpara);
	timer_channel_output_config(PORT_MOTOR_TIMER, TIMER_CH_1,&timer_oc_initpara);
	timer_channel_output_config(PORT_MOTOR_TIMER, TIMER_CH_2,&timer_oc_initpara);
	timer_channel_output_config(PORT_MOTOR_TIMER, TIMER_CH_3,&timer_oc_initpara);

	// 设置初始占空比
	timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_0, 0);
	timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_1, 0);
	timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_2, 0);
	timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_3, 0);

	// 设置PWM模式
	timer_channel_output_mode_config(PORT_MOTOR_TIMER, TIMER_CH_0, TIMER_OC_MODE_PWM0);
	timer_channel_output_mode_config(PORT_MOTOR_TIMER, TIMER_CH_1, TIMER_OC_MODE_PWM0);
	timer_channel_output_mode_config(PORT_MOTOR_TIMER, TIMER_CH_2, TIMER_OC_MODE_PWM0);
	timer_channel_output_mode_config(PORT_MOTOR_TIMER, TIMER_CH_3, TIMER_OC_MODE_PWM0);

	// 打开影子寄存器
	timer_channel_output_shadow_config(PORT_MOTOR_TIMER, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
	timer_channel_output_shadow_config(PORT_MOTOR_TIMER, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
	timer_channel_output_shadow_config(PORT_MOTOR_TIMER, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
	timer_channel_output_shadow_config(PORT_MOTOR_TIMER, TIMER_CH_3, TIMER_OC_SHADOW_ENABLE);

	// 使能定时器，开启
	timer_primary_output_config(PORT_MOTOR_TIMER, ENABLE);
	timer_enable(PORT_MOTOR_TIMER);
}

void bsp_motor_init(void)
{
	bsp_motor_gpio_init();
	bsp_motor_timer_init();
}

void bsp_motor_set(int left, int right)
{
        left = left > 9999 ? 9999 : (left < -9999 ? -9999 : left);
        right = right > 9999 ? 9999 : (right < -9999 ? -9999 : right);

	if (left > 0) {
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_0, 0);
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_1, left);
	} else {
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_0, -left);
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_1, 0);
	}

	if (left > 0) {
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_2, right);
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_3, 0);
	} else {
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_2, 0);
		timer_channel_output_pulse_value_config(PORT_MOTOR_TIMER, TIMER_CH_3, -right);
	}
}

void bsp_motor_test(void)
{
	bsp_motor_init();

	int duty = -5000;

	while (1) {
		duty += 500;
		bsp_motor_set(duty, 0);
		delay_1ms(500);
		if (duty > 5000)
			duty = -5000;
	}
}