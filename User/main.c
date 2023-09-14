#include "systick.h"
#include <stdio.h>

#include "bsp_dma_recv.h"
#include "bsp_led.h"
#include "usart.h"
#include "bsp_timer.h"
#include "bsp_ultrasonic.h"
#include "bsp_battery.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_key.h"

#include "data_show.h"

extern float g_balanceKP, g_balanceKI, g_balanceKD, g_velocityKP, g_velocityKI, g_velocityKD;

void test(int left, int right)
{
        while (1) {
                if (left > 9999)
                        left = 9999;
                if (left < -9999)
                        left = -9999;
                if (right > 9999)
                        right = 9999;
                if (right < -9999)
                        right = -9999;

                printf("left:%d\n", left);
                printf("right:%d\n", right);
        }
}

void p_control_test()
{
        // 90°转动
        int target = 350;
        int current = 0;

        while (1) {
                float kp = g_balanceKP;
                current = bsp_encoder_get_left();

                // 误差计算
                float error = target - current;
                int pwm = kp * error;
                bsp_motor_set(pwm, 0);

                // 显示当前编码器的读数
                float cure_enc = current;
                data_show_push(&cure_enc, 1);
                delay_1ms(5);
        }
}

void pi_control_test()
{
        // 90°转动
        int target = 350;
        // 误差的积分
        float error_integral = 0;

        while (1) {
                float kp = g_balanceKP;
                float ki = g_balanceKI;
                int current = bsp_encoder_get_left();

                // 误差计算
                float error = target - current;
                /* PID 公式 */
                int pwm = kp * error + ki * error_integral;

                error_integral += error;
                bsp_motor_set(pwm, 0);

                // 显示当前编码器的读数
                float cure_enc = current;
                data_show_push(&cure_enc, 1);
                delay_1ms(5);
        }
}

void pd_control_test()
{
        // 90°转动
        int target = 350;
        // 微分：误差变化的速率
        float error_last = 0;
        while (1) {
                float kp = g_balanceKP;
                float ki = g_balanceKI;
                float kd = g_balanceKD;
                int current = bsp_encoder_get_left();

                // 误差计算
                float error = target - current;
                /* PID 公式 */
                int pwm = kp * error + kd * (error - error_last);
                // 记录上一次的误差值
                error_last = error;

                bsp_motor_set(pwm, 0);

                // 显示当前编码器的读数
                float cure_enc = current;
                data_show_push(&cure_enc, 1);
                delay_1ms(5);
        }
}

void pid_control_test()
{
        // 90°转动
        int target = 350;
        // 累计误差
        float error_sum = 0;
        // 微分：误差变化的速率， 记录上一次的误差值
        float error_last = 0;
        while (1) {
                float kp = g_balanceKP;
                float ki = g_balanceKI;
                float kd = g_balanceKD;
                // 编码器当前值
                int current = bsp_encoder_get_left();
                // 误差计算
                float error = target - current;
                /* PID 公式 */
                int pwm = kp * error + ki * error_sum + kd * (error - error_last);
                // pwm 控制电机
                bsp_motor_set(pwm, 0);
                error_sum += error;
                error_last = error;
                // 显示当前编码器的读数
                float cure_enc = current;
                data_show_push(&cure_enc, 1);
                delay_1ms(5);
        }
}

// 增量式PID
void incremental_pid_control_test()
{
        // 目标：90°转动
        int target = 350;
        int pwm = 0;
        // 累计误差
        float error_sum = 0;
        // 微分：误差变化的速率， 记录上一次的误差值
        float error_last = 0;
        float error_last_last = 0;
        while (1) {
                float kp = g_balanceKP;
                float ki = g_balanceKI;
                float kd = g_balanceKD;
                // 编码器当前值
                int current = bsp_encoder_get_left();
                // 误差计算
                float error = target - current;

                /**增量式PID 公式
                 * pwm += kp * (本次误差 - 上次误差) + ki * 本次误差 + kd * ((本次误差 - 上次误差) - (上次误差 - 上上次误差))
                 */
                pwm += kp * (error - error_last) + ki * error + kd * ((error - error_last) - (error_last - error_last_last));
                // 更新参数
                error_last_last = error_last;
                error_last = error;

                bsp_motor_set(pwm, 0);

                // 显示当前编码器的读数
                float curr_encoder = current;
                data_show_push(&curr_encoder, 1);
                delay_1ms(5);
        }
}

int main(void)
{
        nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
        systick_config();

        bsp_usart_dma_init(115200);

        //printf("led test\n");
        //bsp_led_test();

        //printf("timer test\n");
        //bsp_timer_test();

        /*printf("ultrasonic test\n");
        bsp_ultrasonic_test();*/

        //printf("battery 1\n");
        //bsp_battery_test();

        /*printf("motor test\n");
        bsp_motor_test();*/

        /*printf("encoder test\n");
        bsp_encoder_test();*/

        /*printf("key test\n");
        bsp_key_test();*/

        //test(10000, 10000);

        bsp_encoder_init();
        bsp_motor_init();
        bsp_key_init();

        //kp_control_test();
        //pi_control_test();
        //pd_control_test();
        //pid_control_test();

        incremental_pid_control_test();
}
