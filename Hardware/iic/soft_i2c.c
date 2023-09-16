#include "soft_i2c.h"
#include "systick.h"

static void start();
static void stop();
static uint8_t wait_ack();
static void send_ack();
static void send_nack();
static void send(uint8_t data);
static uint8_t recv();


void soft_i2c_init()
{
        // 时钟配置
        rcu_periph_clock_enable(SCL_RCU);
        // 设置输出模式
        gpio_mode_set(SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SCL_PIN);
        gpio_output_options_set(SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, SCL_PIN);
        // 设置复用功能
        //gpio_af_set(SCL_PORT, SCL_AF, SCL_PIN);
        // 时钟配置
        rcu_periph_clock_enable(SDA_RCU);
        // 设置输出模式
        gpio_mode_set(SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SDA_PIN);
        gpio_output_options_set(SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, SDA_PIN);
        // 设置复用功能
        //gpio_af_set(SDA_PORT, SDA_AF, SDA_PIN);
}

static void send(uint8_t data)
{
        uint8_t i;
        SDA_OUT();

        for (i = 0; i < 8; i++) {
                if (data & 0x80) {
                        SDA(1);

                } else {
                        SDA(0);
                }

                SCL(1);
                delay_1us(5);
                SCL(0);
                delay_1us(5);
                data <<= 1;
        }
}

static uint8_t recv()
{
        uint8_t i, data;
        SDA_IN();
        data = 0;

        for (i = 0; i < 8; i++) {
                SCL(0);
                delay_1us(5);
                SCL(1);
                delay_1us(5);
                data <<= 1;
                data |= SDA_STATE();
                delay_1us(5);
        }

        SCL(0);
        return data;
}

uint8_t soft_i2c_read(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t len)
{
        start();
        send(addr << 1);								//发送设备写地址

        if (wait_ack()) {
                return 1;        //等待响应
        }

        send(reg);											//发送寄存器地址

        if (wait_ack()) {
                return 2;        //等待响应
        }

        start();
        send((addr << 1) | 0x01);				//发送设备读地址

        if (wait_ack()) {
                return 3;        //等待响应
        }

        do {
                *data = recv();
                data++;

                if (len != 1) {
                        send_ack();        // 发送 NACK
                }
        } while (--len);

        send_nack();										// 发送 NACK
        stop();
        return 0;
}

uint8_t soft_i2c_write(uint8_t addr, uint8_t reg, uint8_t* data, uint32_t len)
{
        start();
        send(addr << 1);										//发送设备写地址

        if (wait_ack()) {
                return 1;        //等待响应
        }

        send(reg);													//发送寄存器地址

        if (wait_ack()) {
                return 2;        //等待响应
        }

        do {
                send(*data++);

                if (wait_ack()) {
                        return 3;
                }
        } while (--len);

        stop();
        return 0;
}

uint8_t soft_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
        return soft_i2c_write(addr, reg, &data, 1);
}

uint8_t soft_i2c_read_byte(uint8_t addr, uint8_t reg)
{
        uint8_t data;
	soft_i2c_read(addr, reg, &data, 1);
        return data;
}




void soft_i2c_deinit()
{
}


static void start()
{
        SDA_OUT();
        SDA(1);
        delay_1us(5);
        SCL(1);
        delay_1us(5);
        SDA(0);
        delay_1us(5);
        SCL(0);
        delay_1us(5);
}

static void stop()
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        SCL(1);
        delay_1us(5);
        SDA(1);
        delay_1us(5);
}

static uint8_t wait_ack()
{
        int8_t retry = 10;
        SCL(0);
        SDA(1);
        SDA_IN();
        delay_1us(5);
        SCL(1);
        delay_1us(5);

        while (SDA_STATE() == 1 && retry > 0) {
                retry --;
                delay_1us(5);
        }

        if (retry <= 0) {
                stop();
                // 重启I2C
		soft_i2c_init();
                return 1;

        } else {
                SCL(0);
                SDA_OUT();
        }

        return 0;
}

static void send_ack()
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        delay_1us(5);
        SDA(0);
        SCL(1);
        delay_1us(5);
        SCL(0);
        SDA(1);
}
static void send_nack()
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        delay_1us(5);
        SDA(1);
        SCL(1);
        delay_1us(5);
        SCL(0);
        SDA(1);
}
