#include "gd32f4xx.h"
#include "i2c.h"
#include <stdio.h>

#define USE_I2C0
#ifdef USE_I2C0
#define I2C_RCU_CLOCK RCU_GPIOB
#define I2C_CLOCK RCU_I2C0
#define I2C_INDEX        I2C0
#define I2C_USE_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_8
#define I2C_SDA_PIN GPIO_PIN_9
#endif


//#define USE_I2C1
#ifdef USE_I2C1
#define I2C_RCU_CLOCK RCU_GPIOB
#define I2C_CLOCK RCU_I2C1
#define I2C_INDEX	I2C1
#define I2C_USE_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_10
#define I2C_SDA_PIN GPIO_PIN_11
#endif

//初始化IIC的IO口
void IIC_Init(void)
{
	// PB10 --I2C1_SCL  PB11 -- I2C1_SDA
	// PB6 -- I2C0_SCL   PB7 -- I2C0_SDA
	rcu_periph_clock_enable(I2C_RCU_CLOCK);
	// 配置I2C时钟
	rcu_periph_clock_enable(I2C_CLOCK);

	// 连接i2c

	/* 配置为输出模式 复用模式 */
	gpio_af_set(I2C_USE_PORT, GPIO_AF_4, I2C_SCL_PIN);
	gpio_mode_set(I2C_USE_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_PIN);
	gpio_output_options_set(I2C_USE_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);


	gpio_af_set(I2C_USE_PORT, GPIO_AF_4, I2C_SDA_PIN);
	gpio_mode_set(I2C_USE_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SDA_PIN);
	gpio_output_options_set(I2C_USE_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_PIN);


	rcu_periph_clock_enable(I2C_CLOCK);

	i2c_clock_config(I2C_INDEX, 400000, I2C_DTCY_2);

	i2c_mode_addr_config(I2C_INDEX, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x68);

	i2c_enable(I2C_INDEX);

	i2c_ack_config(I2C_INDEX, I2C_ACK_ENABLE);

	printf("I2C INIT COMPLETE\r\n");
}

uint8_t IIC_ReadData(uint8_t dev_addr, uint8_t reg, uint8_t *buffer, uint8_t len)
{
	dev_addr = dev_addr << 1;
	/* wait until I2C bus is idle */
	while (i2c_flag_get(I2C_INDEX, I2C_FLAG_I2CBSY));

	if (2 == len) {
		i2c_ackpos_config(I2C_INDEX, I2C_ACKPOS_NEXT);
	}

	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C_INDEX);

	/* wait until SBSEND bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_SBSEND));

	/* send slave address to I2C bus */
	i2c_master_addressing(I2C_INDEX, dev_addr, I2C_TRANSMITTER);

	/* wait until ADDSEND bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_ADDSEND));

	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C_INDEX, I2C_FLAG_ADDSEND);

	/* wait until the transmit data buffer is empty */
	while (SET != i2c_flag_get(I2C_INDEX, I2C_FLAG_TBE));

	/* enable I2C_INDEX*/
	i2c_enable(I2C_INDEX);

	/* send the EEPROM's internal address to write to */
	i2c_data_transmit(I2C_INDEX, reg);

	/* wait until BTC bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));

	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C_INDEX);

	/* wait until SBSEND bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_SBSEND));

	/* send slave address to I2C bus */
	i2c_master_addressing(I2C_INDEX, dev_addr, I2C_RECEIVER);

	if (len < 3) {
		/* disable acknowledge */
		i2c_ack_config(I2C_INDEX, I2C_ACK_DISABLE);
	}

	/* wait until ADDSEND bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_ADDSEND));

	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C_INDEX, I2C_FLAG_ADDSEND);

	if (1 == len) {
		i2c_ackpos_config(I2C_INDEX, I2C_ACKPOS_NEXT);
		/* send a stop condition to I2C bus */
		i2c_stop_on_bus(I2C_INDEX);

	}


	/* while there is data to be read */
	while (len) {
		if (3 == len) {
			/* wait until BTC bit is set */
			while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));

			/* disable acknowledge */
			i2c_ack_config(I2C_INDEX, I2C_ACK_DISABLE);
		}
		if (2 == len) {
			/* wait until BTC bit is set */
			while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));

			/* send a stop condition to I2C bus */
			i2c_stop_on_bus(I2C_INDEX);
		}

		if (1 == len) {
			i2c_ackpos_config(I2C_INDEX, I2C_ACKPOS_CURRENT);        // 这一步是关键
			i2c_ack_config(I2C_INDEX, I2C_ACK_DISABLE);
		}

		/* wait until the RBNE bit is set and clear it */
		if (i2c_flag_get(I2C_INDEX, I2C_FLAG_RBNE)) {
			/* read a byte from the EEPROM */
			*buffer = i2c_data_receive(I2C_INDEX);

			/* point to the next location where the byte read will be saved */
			buffer++;

			/* decrement the read bytes counter */
			len--;
		}
	}

	/* wait until the stop condition is finished */
	while (I2C_CTL0(I2C_INDEX) & 0x0200);

	/* enable acknowledge */
	i2c_ack_config(I2C_INDEX, I2C_ACK_ENABLE);

	i2c_ackpos_config(I2C_INDEX, I2C_ACKPOS_CURRENT);

	return 0;
}

uint8_t IIC_WriteData(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	dev_addr = dev_addr << 1;
	while (i2c_flag_get(I2C_INDEX, I2C_FLAG_I2CBSY));
	i2c_start_on_bus(I2C_INDEX);
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_SBSEND));
	i2c_master_addressing(I2C_INDEX, dev_addr, I2C_TRANSMITTER);
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_ADDSEND));
	i2c_flag_clear(I2C_INDEX, I2C_FLAG_ADDSEND);
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_TBE));
	i2c_data_transmit(I2C_INDEX, reg);
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));
	while (len) {
		i2c_data_transmit(I2C_INDEX, *buf);
		buf++;
		len--;
		while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));
	}
	i2c_stop_on_bus(I2C_INDEX);
	while (I2C_CTL0(I2C_INDEX) & 0x0200);
	return 0;
}

uint8_t IIC_WriteCmd(uint8_t dev_addr, uint8_t cmd)
{
	dev_addr = dev_addr << 1;
	/* wait until I2C bus is idle */
	while (i2c_flag_get(I2C_INDEX, I2C_FLAG_I2CBSY));

	/* send a start condition to I2C bus */
	i2c_start_on_bus(I2C_INDEX);

	/* wait until SBSEND bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_SBSEND));

	/* send slave address to I2C bus */
	i2c_master_addressing(I2C_INDEX, dev_addr, I2C_TRANSMITTER);

	/* wait until ADDSEND bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_ADDSEND));

	/* clear the ADDSEND bit */
	i2c_flag_clear(I2C_INDEX, I2C_FLAG_ADDSEND);

	/* wait until the transmit data buffer is empty */
	while (SET != i2c_flag_get(I2C_INDEX, I2C_FLAG_TBE));

	/* send the EEPROM's internal address to write to : only one byte address */
	i2c_data_transmit(I2C_INDEX, cmd);

	/* wait until BTC bit is set */
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));

//    /* send the byte to be written */
//    i2c_data_transmit(I2C_INDEX, *p_buffer); 
//    
//    /* wait until BTC bit is set */
//    while(!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));

	/* send a stop condition to I2C bus */
	i2c_stop_on_bus(I2C_INDEX);

	/* wait until the stop condition is finished */
	while (I2C_CTL0(I2C_INDEX) & 0x0200);

	return 0;
}

/*!
    \brief 	轮询询问I2C从设备地址
    \param[in] 	发送7位设备地址
*/
uint8_t IIC_WhoAmI(uint8_t dev_addr)
{
	uint16_t i;
	dev_addr = dev_addr << 1;
	while (i2c_flag_get(I2C_INDEX, I2C_FLAG_I2CBSY));

	i2c_start_on_bus(I2C_INDEX);
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_SBSEND));

	i2c_master_addressing(I2C_INDEX, dev_addr, I2C_TRANSMITTER);
	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_ADDSEND)) {
		if (i < 0xFFFF) i++;
		else {
			i2c_stop_on_bus(I2C_INDEX);
			while (I2C_CTL0(I2C_INDEX) & 0x0200);
			return 2;
		}
	}
	i2c_stop_on_bus(I2C_INDEX);
	while (I2C_CTL0(I2C_INDEX) & 0x0200);

	return 0;
}
void IIC_SearchAddr(void)
{
	printf("i2c search address:\r\n");
	uint8_t address;
	for (address = 1; address < 127; address++) {
		uint8_t ret = IIC_WhoAmI(address);
		if (ret == 0)
			printf("ret=%d ,address:%X\n", ret, address);
	}
}

int IIC_Start(void)                                //发送IIC开始信号
{
	while (i2c_flag_get(I2C_INDEX, I2C_FLAG_I2CBSY));
	i2c_start_on_bus(I2C_INDEX);

	return 0;
}

void IIC_Stop(void)                                //发送IIC停止信号
{

}

//IIC发送一个字节
uint8_t IIC_Send_Byte(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
	dev_addr = dev_addr << 1;
	while (i2c_flag_get(I2C_INDEX, I2C_FLAG_I2CBSY));

	i2c_start_on_bus(I2C_INDEX);

	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_SBSEND));

	i2c_master_addressing(I2C_INDEX, dev_addr, I2C_TRANSMITTER);

	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_ADDSEND));

	i2c_flag_clear(I2C_INDEX, I2C_FLAG_ADDSEND);

	while (SET != i2c_flag_get(I2C_INDEX, I2C_FLAG_TBE));

	i2c_data_transmit(I2C_INDEX, reg);

	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));

	i2c_data_transmit(I2C_INDEX, data);

	while (!i2c_flag_get(I2C_INDEX, I2C_FLAG_BTC));

	i2c_stop_on_bus(I2C_INDEX);

	while (I2C_CTL0(I2C_INDEX) & 0x0200);
	return 0;
}

uint8_t IIC_Read_Byte(uint8_t dev_addr, uint8_t reg)//IIC读取一个字节
{
	uint8_t buffer = 0;
	IIC_ReadData(dev_addr, reg, &buffer, 1);
	return buffer;
}

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
/*void gpio_config(void)
{
	*//* enable GPIOB clock *//*
	rcu_periph_clock_enable(RCU_GPIOB);
	*//* enable I2C0 clock *//*
	rcu_periph_clock_enable(RCU_I2C0);

	*//* connect PB6 to I2C0_SCL *//*
	*//* connect PB7 to I2C0_SDA *//*
	gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
}*/

/*!
    \brief      configure the I2C0 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
/*void i2c_config(void)
{
	*//* enable I2C clock *//*
	rcu_periph_clock_enable(RCU_I2C0);
	*//* configure I2C clock *//*
	i2c_clock_config(I2C0, I2C0_SPEED, I2C_DTCY_2);
	*//* configure I2C address *//*
	i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_SLAVE_ADDRESS7);
	*//* enable I2C0 *//*
	i2c_enable(I2C0);
	*//* enable acknowledge *//*
	i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}*/
