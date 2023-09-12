#include "bsp_timer.h"

uint32_t timer_ms = 0;

void bsp_timer_init()
{
        // rcu定时器时钟预分频器配置
        rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
        // 开启定时器
        rcu_periph_clock_enable(RCU_TIMER5);
        // 初始化定时器（复位）
        timer_deinit(TIMER5);

        // 设置定时器参数
        timer_parameter_struct timer_initpara;
        timer_struct_para_init(&timer_initpara);
        timer_initpara.period = 1000 - 1;
	// 定时 1ms
        timer_initpara.prescaler = 240 - 1;

        timer_init(TIMER5, &timer_initpara);

        // 当计数值到达指定值时，更新的时候需要统计ms
        nvic_irq_enable(TIMER5_DAC_IRQn, 1, 1);
        timer_interrupt_enable(TIMER5, TIMER_INT_UP);
        timer_enable(TIMER5);
}

// TIMER5 中断函数
void TIMER5_DAC_IRQHandler(void)
{
        if (timer_interrupt_flag_get(TIMER5, TIMER_INT_UP) != RESET) {
                timer_interrupt_flag_clear(TIMER5, TIMER_INT_UP);
                timer_ms++;
        }
}

void bsp_timer_open()
{
	// 清空计数
        timer_counter_value_config(TIMER5, 0);
        timer_ms = 0;
        timer_enable(TIMER5);
}

void bsp_timer_close()
{
        timer_disable(TIMER5);
}

int bsp_timer_get()
{
        uint32_t t = timer_ms * 1000;
	// 当前计数值
        t += timer_counter_read(TIMER5);
        return t;
}

void bsp_timer_test()
{
        bsp_timer_init();

        while (1) {
                bsp_timer_open();
                delay_1ms(500);
                bsp_timer_close();
		// 获取时间差
                int time = bsp_timer_get();
                printf("time: %d \r\n", time);
        }
}
