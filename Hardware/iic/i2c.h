
#ifndef I2C_H
#define I2C_H
#include <stdint.h>

#include "gd32f4xx.h"

#define I2C0_SPEED              400000
#define I2C0_SLAVE_ADDRESS7     0xA0
#define I2C_PAGE_SIZE           8


/* configure the GPIO ports */
void gpio_config(void);
/* configure the I2C0 interfaces */
void i2c_config(void);

// PB6 -- I2C0_SCL   PB7 -- I2C0_SDA
//#define IIC_SCL    GPIO_PIN_6 //SCL
//#define IIC_SDA    GPIO_PIN_7 //SDA	 


//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
int IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
uint8_t IIC_Send_Byte(uint8_t dev_addr, uint8_t reg,uint8_t txd);			//IIC发送一个字节

uint8_t IIC_Read_Byte(uint8_t dev_addr, uint8_t reg);//IIC读取一个字节

uint8_t IIC_ReadData(uint8_t dev_addr, uint8_t reg,uint8_t* buffer, uint8_t len);
uint8_t IIC_WriteData(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t IIC_WriteCmd(uint8_t addr,uint8_t cmd);

uint8_t IIC_WhoAmI(uint8_t slave_addr);
void IIC_SearchAddr(void);

#endif  /* I2C_H */
