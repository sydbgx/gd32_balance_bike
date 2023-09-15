#include "bsp_encoder.h"

/**
 * 编码器模式，只能使用TIMERX中的CH0和CH1，
 * 使用了CH0和CH1的编码器模式，后面的CH2和CH3就不支持使用。
 */
#define RCU_ENCODER_L1 	RCU_GPIOA
#define PORT_ENCODER_L1 GPIOA
#define PIN_ENCODER_L1 	GPIO_PIN_15
#define AF_ENCODER_L1 	GPIO_AF_1

#define RCU_ENCODER_L2 	RCU_GPIOB
#define PORT_ENCODER_L2 GPIOB
#define PIN_ENCODER_L2 	GPIO_PIN_3
#define AF_ENCODER_L2 	GPIO_AF_1

#define RCU_TIMER_L 	RCU_TIMER1
#define PORT_TIMER_L 	TIMER1

#define RCU_ENCODER_R1 	RCU_GPIOB
#define PORT_ENCODER_R1 GPIOB
#define PIN_ENCODER_R1 	GPIO_PIN_4
#define AF_ENCODER_R1 	GPIO_AF_2

#define RCU_ENCODER_R2 	RCU_GPIOB
#define PORT_ENCODER_R2 GPIOB
#define PIN_ENCODER_R2 	GPIO_PIN_5
#define AF_ENCODER_R2 	GPIO_AF_2

#define RCU_TIMER_R 	RCU_TIMER2
#define PORT_TIMER_R 	TIMER2

void bsp_encoder_gpio_init(void)
{
	// 初始化左边编码器
	rcu_periph_clock_enable(RCU_ENCODER_L1);
	gpio_mode_set(PORT_ENCODER_L1, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_ENCODER_L1);
	gpio_af_set(PORT_ENCODER_L1, AF_ENCODER_L1, PIN_ENCODER_L1);

	rcu_periph_clock_enable(RCU_ENCODER_L2);
	gpio_mode_set(PORT_ENCODER_L2, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_ENCODER_L2);
	gpio_af_set(PORT_ENCODER_L2, AF_ENCODER_L2, PIN_ENCODER_L2);

	// 初始化右边编码器
	rcu_periph_clock_enable(RCU_ENCODER_R1);
	gpio_mode_set(PORT_ENCODER_R1, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_ENCODER_R1);
	gpio_af_set(PORT_ENCODER_R1, AF_ENCODER_R1, PIN_ENCODER_R1);

	rcu_periph_clock_enable(RCU_ENCODER_R2);
	gpio_mode_set(PORT_ENCODER_R2, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_ENCODER_R2);
	gpio_af_set(PORT_ENCODER_R2, AF_ENCODER_R2, PIN_ENCODER_R2);
}

/* 配置左边编码器 */
void bsp_encoder_left_init()
{
	rcu_periph_clock_enable(RCU_TIMER_L);
	// 复位定时器
	timer_deinit(PORT_TIMER_L);

	// 配置定时器的基本参数
	timer_parameter_struct timer_initpara;
	timer_struct_para_init(&timer_initpara);
	timer_initpara.period = 65535U;
	timer_init(PORT_TIMER_L, &timer_initpara);

	// 配置定时器的输入参数
	timer_ic_parameter_struct ic_initpara;
	timer_channel_input_struct_para_init(&ic_initpara);
	// 电路过滤器，取值范围为 0~15
	ic_initpara.icfilter = 10;
	// 设置参数
	timer_input_capture_config(PORT_TIMER_L, TIMER_CH_0,&ic_initpara);
	timer_input_capture_config(PORT_TIMER_L, TIMER_CH_1,&ic_initpara);

	// 启动定时器编码格式
	timer_quadrature_decoder_mode_config(PORT_TIMER_L, TIMER_QUAD_DECODER_MODE2,
					     TIMER_IC_POLARITY_FALLING,TIMER_IC_POLARITY_FALLING);

	// 启用定时器
	timer_counter_value_config(PORT_TIMER_L, 0);
	// 设置影子寄存器：增加缓冲区，是数据更加平缓
	timer_auto_reload_shadow_enable(PORT_TIMER_L);
	timer_enable(PORT_TIMER_L);
}

/* 配置右边编码器 */
void bsp_encoder_right_init()
{
	rcu_periph_clock_enable(RCU_TIMER_R);
	// 复位定时器
	timer_deinit(RCU_TIMER_R);

	// 配置定时器的基本参数
	timer_parameter_struct timer_initpara;
	timer_struct_para_init(&timer_initpara);
	timer_initpara.period = 65535U;
	timer_init(PORT_TIMER_R, &timer_initpara);

	// 配置定时器的输入参数
	timer_ic_parameter_struct ic_initpara;
	timer_channel_input_struct_para_init(&ic_initpara);
	// 电路过滤器
	ic_initpara.icfilter = 10;
	// 设置参数
	timer_input_capture_config(PORT_TIMER_R, TIMER_CH_2,&ic_initpara);
	timer_input_capture_config(PORT_TIMER_R, TIMER_CH_3,&ic_initpara);

	// 启动定时器编码格式
	timer_quadrature_decoder_mode_config(PORT_TIMER_R, TIMER_QUAD_DECODER_MODE2,
					     TIMER_IC_POLARITY_FALLING,TIMER_IC_POLARITY_FALLING);

	// 启用定时器
	timer_counter_value_config(PORT_TIMER_R, 0);
	// 设置影子寄存器：增加缓冲区，是数据更加平缓
	timer_auto_reload_shadow_enable(PORT_TIMER_R);
	timer_enable(PORT_TIMER_R);
}

/* 配置右边编码器 */
void bsp_encoder_timer_init(void)
{
	rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);

	bsp_encoder_left_init();

	bsp_encoder_right_init();
}

/* 初始化 */
void bsp_encoder_init()
{
	bsp_encoder_gpio_init();
	bsp_encoder_timer_init();
}

/* 获取左边电机的转速 */
short bsp_encoder_get_left(void)
{
	// 使用short接受：电机正反转有正负，short范围是-32768~32767，可以接受负数，period是65535
	short value = (short)timer_counter_read(PORT_TIMER_L);
	timer_counter_value_config(PORT_TIMER_L, 0);
	return -value;
}

/* 获取右边电机的转速 */
short bsp_encoder_get_right(void)
{
	// 使用short接受：电机正反转有正负，short范围是-32768~32767，可以接受负数，period是65535
	short value = (short)timer_counter_read(PORT_TIMER_R);
	timer_counter_value_config(PORT_TIMER_R, 0);
	return value;
}

void bsp_encoder_test(void)
{
	bsp_encoder_init();

	while (1) {
		short left = bsp_encoder_get_left();
		short right = bsp_encoder_get_right();
		printf("left: %d right: %d\r\n", left, right);
		delay_1ms(200);
	}
}