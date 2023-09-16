#include "i2c.h"

void I2C_init()
{
	uint32_t i2cx_scl_port_rcu = RCU_GPIOB;
	uint32_t i2cx_scl_port = GPIOB;
	uint32_t i2cx_scl_pin = GPIO_PIN_8;
	uint32_t i2cx_scl_af = GPIO_AF_4;
	uint32_t i2cx_sda_port_rcu = RCU_GPIOB;
	uint32_t i2cx_sda_port = GPIOB;
	uint32_t i2cx_sda_pin = GPIO_PIN_9;
	uint32_t i2cx_sda_af = GPIO_AF_4;
	uint32_t i2cx = I2C0;
	uint32_t i2cx_rcu = RCU_I2C0;
	uint32_t i2cx_speed = 400000;

	/****************** GPIO config **********************/
	// 时钟配置
	rcu_periph_clock_enable(i2cx_scl_port_rcu);
	// 设置复用功能
	gpio_af_set(i2cx_scl_port, i2cx_scl_af, i2cx_scl_pin);
	// 设置输出模式
	gpio_mode_set(i2cx_scl_port, GPIO_MODE_AF, GPIO_PUPD_NONE, i2cx_scl_pin);
	gpio_output_options_set(i2cx_scl_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, i2cx_scl_pin);
	// 时钟配置
	rcu_periph_clock_enable(i2cx_sda_port_rcu);
	// 设置复用功能
	gpio_af_set(i2cx_sda_port, i2cx_sda_af, i2cx_sda_pin);
	// 设置输出模式
	gpio_mode_set(i2cx_sda_port, GPIO_MODE_AF, GPIO_PUPD_NONE, i2cx_sda_pin);
	gpio_output_options_set(i2cx_sda_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, i2cx_sda_pin);

	/****************** I2C config  **********************/
	i2c_deinit(i2cx);
	// 时钟配置
	rcu_periph_clock_enable(i2cx_rcu);
	// I2C速率配置
	i2c_clock_config(i2cx, i2cx_speed, I2C_DTCY_2);
	// 使能i2c
	i2c_mode_addr_config(i2cx, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x00);
	i2c_enable(i2cx);
	// i2c ack enable
	i2c_ack_config(i2cx, I2C_ACK_ENABLE);
}

static uint8_t I2C_wait(uint32_t i2cx, uint32_t flag)
{
	uint16_t TIMEOUT = 50000;
	uint16_t cnt = 0;

	while (!i2c_flag_get(i2cx, flag)) {
		cnt++;

		if (cnt > TIMEOUT)
			return 1;
	}

	return 0;
}

static uint8_t I2C_waitn(uint32_t i2cx, uint32_t flag)
{
	uint16_t TIMEOUT = 50000;
	uint16_t cnt = 0;

	while (i2c_flag_get(i2cx, flag)) {
		cnt++;

		if (cnt > TIMEOUT)
			return 1;
	}

	return 0;
}


uint8_t I2C_write(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t data_len)
{
	uint32_t i2cx = I2C0;
	uint8_t address = addr << 1;

	/************* start ***********************/
	// 等待I2C闲置
	if (I2C_waitn(i2cx, I2C_FLAG_I2CBSY))
		return 1;

	// start
	i2c_start_on_bus(i2cx);

	// 等待I2C主设备成功发送起始信号
	if (I2C_wait(i2cx, I2C_FLAG_SBSEND))
		return 2;

	/************* device address **************/
	// 发送设备地址
	i2c_master_addressing(i2cx, address, I2C_TRANSMITTER);

	// 等待地址发送完成
	if (I2C_wait(i2cx, I2C_FLAG_ADDSEND))
		return 3;

	i2c_flag_clear(i2cx, I2C_FLAG_ADDSEND);

	/************ register address ************/
	// 寄存器地址
	// 等待发送数据缓冲区为空
	if (I2C_wait(i2cx, I2C_FLAG_TBE))
		return 4;
	// 发送数据
	i2c_data_transmit(i2cx, reg);

	// 等待数据发送完成
	if (I2C_wait(i2cx, I2C_FLAG_BTC))
		return 5;

	/***************** data ******************/
	// 发送数据
	uint32_t i;

	for (i = 0; i < data_len; i++) {
		uint32_t d = data[i];

		// 等待发送数据缓冲区为空
		if (I2C_wait(i2cx, I2C_FLAG_TBE))
			return 6;

		// 发送数据
		i2c_data_transmit(i2cx, d);

		// 等待数据发送完成
		if (I2C_wait(i2cx, I2C_FLAG_BTC))
			return 7;
	}

	/***************** stop ********************/
	// stop
	i2c_stop_on_bus(i2cx);

	if (I2C_waitn(i2cx, I2C_CTL0(I2C0)&I2C_CTL0_STOP))
		return 8;

	return 0;
}

uint8_t I2C_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	return I2C_write(addr, reg, &data, 1);
}

void I2C_deinit()
{
}

uint8_t I2C_read(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t len)
{
	uint32_t i2cx = I2C0;
	uint8_t address = addr << 1;

	/************* start ***********************/
	// 等待I2C空闲
	if (I2C_waitn(i2cx, I2C_FLAG_I2CBSY))
		return 1;

	// 发送启动信号
	i2c_start_on_bus(i2cx);

	if (I2C_wait(i2cx, I2C_FLAG_SBSEND))
		return 2;

	/************* device address **************/
	// 发送从设备地址
	i2c_master_addressing(i2cx, address, I2C_TRANSMITTER);

	if (I2C_wait(i2cx, I2C_FLAG_ADDSEND))
		return 3;

	i2c_flag_clear(i2cx, I2C_FLAG_ADDSEND);

	/********** register address **************/
	// 等待发送缓冲区
	if (I2C_wait(i2cx, I2C_FLAG_TBE))
		return 4;

	// 发送寄存器地址
	i2c_data_transmit(i2cx, reg);

	// 等待发送数据完成
	if (I2C_wait(i2cx, I2C_FLAG_BTC))
		return 5;

	if (I2C_wait(i2cx, I2C_FLAG_TBE))
		return 6;

	// 发送再启动信号
	i2c_start_on_bus(i2cx);

	if (I2C_wait(i2cx, I2C_FLAG_SBSEND))
		return 7;

	// 发送从设备地址
	i2c_master_addressing(i2cx, address, I2C_RECEIVER);

	if (I2C_wait(i2cx, I2C_FLAG_ADDSEND))
		return 8;

	i2c_flag_clear(i2cx, I2C_FLAG_ADDSEND);
	//ack
	i2c_ack_config(i2cx, I2C_ACK_ENABLE);
	i2c_ackpos_config(i2cx, I2C_ACKPOS_CURRENT);

	if (I2C_wait(i2cx, (I2C_CTL0(i2cx) & I2C_CTL0_ACKEN)))
		return 23;

	// 读取数据
	uint8_t i;

	for (i = 0; i < len; i++) {
		if (i != len - 1) {
			// 等待接收缓冲区
			if (I2C_wait(i2cx, I2C_FLAG_BTC))
				return 9;
		}

		// 等待ACK数据发送完成
		// 等待接收缓冲区
		if (I2C_wait(i2cx, I2C_FLAG_RBNE))
			return 10;

		data[i] = i2c_data_receive(i2cx);

		if (i == len - 1)
			// 在读取最后一个字节之前，禁用ACK，并发送停止信号
			i2c_ack_config(i2cx, I2C_ACK_DISABLE);
	}

	i2c_stop_on_bus(i2cx);

	if (I2C_waitn(i2cx, I2C_CTL0(I2C0) & I2C_CTL0_STOP))
		return 11;

	return 0;
}

uint8_t I2C_read_byte(uint8_t addr, uint8_t reg)
{
	uint8_t data;
	I2C_read(addr, reg, &data, 1);
	return data;
}