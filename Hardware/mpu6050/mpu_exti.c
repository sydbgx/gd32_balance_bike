#include "mpu_exti.h"
#include "gd32f4xx.h"
/*******************************************************************************
* Function Name  : MPU6050_EXTI_Init
* Description    : MPU6050中断初始化 PB5
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MPU6050_EXTI_Init(void)
{

        // mpu6050 中断引脚初始化 PC1
        rcu_periph_clock_enable(RCU_GPIOC);
        rcu_periph_clock_enable(RCU_SYSCFG);

        gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
        gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

        syscfg_exti_line_config(EXTI_SOURCE_GPIOC, EXTI_SOURCE_PIN1);
        /*5.初始化中断线  第三个参数指定上升沿触发*/
        exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_RISING);
        /*6.使能中断和清除中断标志位*/
        //清除中断标志位
        exti_interrupt_flag_clear(EXTI_1);
        //使能中断
        exti_interrupt_enable(EXTI_1);

        nvic_irq_enable(EXTI1_IRQn, 2U, 0U);

}

