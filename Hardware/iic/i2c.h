#ifndef __I2C_H__
#define __I2C_H__

#include "systick.h"
#include "gd32f4xx.h"

void I2C_init();

uint8_t I2C_read(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t len);

uint8_t I2C_write(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t len);

uint8_t I2C_write_byte(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C_read_byte(uint8_t addr, uint8_t reg);
void I2C_deinit();

#endif