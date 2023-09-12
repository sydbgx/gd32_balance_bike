#include "systick.h"
#include <stdio.h>

#include "bsp_led.h"
#include "usart.h"
#include "bsp_timer.h"
#include "bsp_ultrasonic.h"
#include "bsp_battery.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_key.h"

int main(void)
{
        systick_config();
        usart_init(115200);

        //printf("led test\n");
	//bsp_led_test();

	//printf("timer test\n");
	//bsp_timer_test();

	/*printf("ultrasonic test\n");
	bsp_ultrasonic_test();*/

	/*printf("battery test\n");
	bsp_battery_test();*/

	/*printf("motor test\n");
	bsp_motor_test();*/

	/*printf("encoder test\n");
	bsp_encoder_test();*/

	printf("key test\n");
	bsp_key_test();

        while (1) {
		printf("未执行的循环\n");
        }
}