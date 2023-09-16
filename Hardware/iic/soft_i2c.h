#ifndef __SOFT_I2C_H__
#define __SOFT_I2C_H__

#include "systick.h"
#include "gd32f4xx.h"

#define SCL_RCU		RCU_GPIOB
#define SCL_PORT	GPIOB
#define SCL_PIN		GPIO_PIN_8
#define SCL_AF		GPIO_AF_4

#define SDA_RCU		RCU_GPIOB
#define SDA_PORT	GPIOB
#define SDA_PIN		GPIO_PIN_9
#define SDA_AF		GPIO_AF_4

#define SCL(BIT) 	gpio_bit_write(SCL_PORT, SCL_PIN, BIT?SET:RESET)
#define SDA(BIT) 	gpio_bit_write(SDA_PORT, SDA_PIN, BIT?SET:RESET)
#define SDA_STATE() 	gpio_input_bit_get(SDA_PORT, SDA_PIN)

#define SDA_IN()  	gpio_mode_set(SDA_PORT, GPIO_MODE_INPUT,  GPIO_PUPD_NONE, SDA_PIN)
#define SDA_OUT() 	gpio_mode_set(SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SDA_PIN)


void soft_i2c_init();

uint8_t soft_i2c_read(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t len);

uint8_t soft_i2c_write(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t len);

void soft_i2c_deinit();

uint8_t soft_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t soft_i2c_read_byte(uint8_t addr, uint8_t reg);

#endif