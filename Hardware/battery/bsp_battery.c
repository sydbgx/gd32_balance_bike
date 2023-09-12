#include "bsp_battery.h"
// PB1 ADC01_IN9
#define RCU_BATTERY        RCU_GPIOB
#define PORT_BATTERY       GPIOB
#define PIN_BATTERY        GPIO_PIN_1

#define ADC_RCU_BATTERY  RCU_ADC0
#define ADC_PORT_BATTERY ADC0
#define CHANNEL_BATTERY        ADC_CHANNEL_9

void bsp_battery_gpio_init(void)
{
	rcu_periph_clock_enable(RCU_BATTERY);
	gpio_mode_set(PORT_BATTERY, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, PIN_BATTERY);
}

void bsp_battery_adc_init(void)
{
	/* 开启时钟 */
	rcu_periph_clock_enable(ADC_RCU_BATTERY);
	/* 打开倍频，主频 */
	adc_clock_config(ADC_ADCCK_PCLK2_DIV4);

	/* ADC 规则配置 */
	/* ADC channel length config */
	adc_channel_length_config(ADC_PORT_BATTERY, ADC_INSERTED_CHANNEL, 1);
	/* ADC temperature sensor channel config */
	adc_inserted_channel_config(ADC_PORT_BATTERY, 0, CHANNEL_BATTERY, ADC_SAMPLETIME_480);

	/* ADC external trigger enable */
	adc_external_trigger_config(ADC_PORT_BATTERY, ADC_INSERTED_CHANNEL, DISABLE);
	/* ADC data alignment config */
	adc_data_alignment_config(ADC_PORT_BATTERY, ADC_DATAALIGN_RIGHT);
	/* ADC SCAN function enable */
	adc_special_function_config(ADC_PORT_BATTERY, ADC_SCAN_MODE, ENABLE);

	/* enable ADC interface */
	adc_enable(ADC_PORT_BATTERY);
	/* ADC calibration and reset calibration */
	adc_calibration_enable(ADC_PORT_BATTERY);
}

void bsp_battery_init(void)
{
	bsp_battery_gpio_init();
	bsp_battery_adc_init();
}

float bsp_battery_get_voltage(void)
{
	adc_software_trigger_enable(ADC_PORT_BATTERY, ADC_INSERTED_CHANNEL);
	uint32_t value = ADC_IDATA0(ADC_PORT_BATTERY);
	float vol = value / 4095.0 * 3.3 * 4.0;
	printf("value: %lu, vol: %f\r\n", value, vol);
	return vol;
}

void bsp_battery_test(void)
{
	bsp_battery_init();

	while (1) {
		bsp_battery_get_voltage();
		delay_1ms(500);
	}
}
