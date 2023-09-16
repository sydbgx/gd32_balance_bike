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
#include "i2c.h"
#include "inv_mpu.h"

#include "data_show.h"

extern float g_balanceKP, g_balanceKI, g_balanceKD, g_velocityKP, g_velocityKI, g_velocityKD;

void balance_bike_test(int left, int right)
{
        while (1) {
                /*left = left > 9999 ? 9999 : (left < -9999 ? -9999 : left);
                right = right > 9999 ? 9999 : (right < -9999 ? -9999 : right);
                printf("left:%d\n", left);
                printf("right:%d\n", right);*/

		// 测试电机方向是否统一，如果不统一则需要调整电机方向并修改值为正数
		bsp_motor_set(3000, 3000);
		left = bsp_encoder_get_left();
		right = bsp_encoder_get_right();
		printf("left: %d right: %d\r\n", left, right);
		delay_1ms(100);

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

// 串级PID
void tandem_pid_control_test()
{

        int target = 350;
        float error_sum = 0;
        float error_last = 0;
        float encoder_last = 0;

        float error_speed_sum = 0;
        float error_speed_last = 0;
        while (1) {
                float kp1 = g_balanceKP;
                float ki1 = g_balanceKI;
                float kd1 = g_balanceKD;

                float kp2 = g_velocityKP;
                float ki2 = g_velocityKI;
                float kd2 = g_velocityKD;

                // 获取当前编码值
                int current = bsp_encoder_get_left();
                // 位置环PID: kp * 本次误差 + ki * 累计误差 + kd * 误差变化
                float error = target - current;
                float location_pid = kp1 * error + ki1 * error_sum + kd1 * (error - error_last);
                error_last = error;
                error_sum += error;

                /** 限定输出
                 * 300转/分钟，每一转有1400个信号
                 *  300 * 1400 / (60 * 1000) 转/毫秒
                 *  42 / 6 = 7 转/毫秒
                 *  延时 5毫秒， pwm = 7 * 5 = 35
                 */
                location_pid = location_pid > 50 ? 50 : (location_pid < -50 ? -50 : location_pid);

                float speed = current - encoder_last;
                encoder_last = current;
                // 计算误差，位置环的输出是速度环的输入
                float error_speed = location_pid - speed;
                // 速度环PID: kp * 本次误差 + ki * 累计误差 + kd * 误差变化
                float speed_pid = kp2 * error_speed + ki2 * error_speed_sum + kd2 * (error_speed - error_speed_last);
                error_speed_last = error_speed;
                error_speed_sum += error_speed;

                bsp_motor_set(speed_pid, 0);

                // 显示当前编码器的读数
                float curr_encoder = current;
                data_show_push(&curr_encoder, 1);
                delay_1ms(5);
        }
}

void mpu6050_test(void)
{
	uint8_t mpu_result = mpu_dmp_init();
	float pitch;
	float roll;
	float yaw;
	printf("mpu_result:%d\r\n", mpu_result);
	while (1) {
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		printf("pitch:%.2f roll:%.2f yaw:%.2f\r\n", pitch, roll, yaw);
		delay_1ms(100);
	}
}

void balance_test()
{
	uint8_t mpu_result = 0;
	/*while ((mpu_result = mpu_dmp_init()) != 0) {
		printf("\n");
		printf("mpu6050 init error: %d\r\n", mpu_result);
	}*/
	mpu_result = mpu_dmp_init();
	printf("mpu_result:%d\r\n", mpu_result);
	float pitch;
	float roll;
	float yaw;
	short gx, gy, gz;
	// 小车平衡目标值
	float median = 3;
	float error_angle_integral = 0;
	float error_angle_last = 0;
	float error_speed_integral = 0;
	float error_speed_last = 0;
	while (1) {
		float kp = 0;
		float ki = 0;
		float kd = 10;

		float sp = 0;
		float si = 0;
		float sd = 0;

		while (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0) ;
		MPU_Get_Gyroscope(&gx, &gy, &gz);
		printf("roll=%.2f pitch=%.2f yaw=%.2f gx=%d gy=%d gz=%d\r\n", roll, pitch, yaw, gx, gy, gz);
		/* 直立环：控制角度 roll = 3 */
		float error_angle = median - roll;
		float balance_result = kp * error_angle + ki * error_angle_integral + kd * gx;
		error_angle_integral += error_angle;

		int left = bsp_encoder_get_left();
		int right = bsp_encoder_get_right();
		/* 速度环：控制速度，target = 0 */
		float error_speed = 0 - (left + right);
		int speed_result = sp * error_speed + si * error_speed_integral + sd * (error_speed - error_speed_last);
		error_speed_integral += error_speed;
		error_speed_last = error_speed;

		/* 积分限制幅度 */
		error_speed_integral > 10000 ? error_speed_integral = 10000 : (error_speed_integral
			< -10000 ? error_speed_integral = -10000 : error_speed_integral);

		/* 测试 */
		int pwm = balance_result + speed_result;
		printf("pwm: %d  left: %d  right: %d\r\n", pwm, left, right);
		//printf("pwm: %d  \r\n", pwm);
		bsp_motor_set(pwm, pwm);
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

        //incremental_pid_control_test();
        //tandem_pid_control_test();
	//mpu6050_test();

	balance_test();
	while(1) {
	}
}
