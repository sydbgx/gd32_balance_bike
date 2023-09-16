#include "mpu6050.h"
#include "i2c.h"
#include "systick.h"
#include <stdio.h>


#define USE_SOFT_I2C
#ifdef USE_SOFT_I2C
#include "soft_i2c.h"
#define MPU6050_I2C_INIT 	soft_i2c_init
#define MPU6050_WRITE 		soft_i2c_write
#define MPU6050_WRITE_BYTE	soft_i2c_write_byte
#define MPU6050_READ 		soft_i2c_read
#define MPU6050_READ_BYTE 	soft_i2c_read_byte
#else
#include "i2c.h"
#define MPU6050_I2C_INIT 	i2c_init
#define MPU6050_WRITE 		i2c_write
#define MPU6050_WRITE_BYTE	i2c_write_byte
#define MPU6050_READ 		i2c_read
#define MPU6050_READ_BYTE 	i2c_read_byte
#endif

/**
 * 初始化MPU6050
 * @return 返回值:0,成功，其他,错误代码
 */
uint8_t MPU_Init(void)
{
        uint8_t res;
	uint8_t ret = 0;
	MPU6050_I2C_INIT();

        delay_1ms(100);
        printf("MPU_Init start...\r\n");

        //复位MPU6050
	uint8_t data = 0x80;
	MPU6050_WRITE(MPU_ADDR, MPU_PWR_MGMT1_REG, &data, 1);

        delay_1ms(50);
        //MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050
	MPU6050_WRITE_BYTE(MPU_ADDR, MPU_PWR_MGMT1_REG, 0x00);

        MPU_Set_Gyro_Fsr(3);            //陀螺仪传感器,±2000dps
        MPU_Set_Accel_Fsr(0);           //加速度传感器,±2g
        MPU_Set_Rate(200);             //设置采样率50Hz
	MPU6050_WRITE_BYTE(MPU_ADDR, MPU_INT_EN_REG, 0X00);		//关闭所有中断
	MPU6050_WRITE_BYTE(MPU_ADDR, MPU_USER_CTRL_REG, 0X00);     	//I2C主模式关闭
	MPU6050_WRITE_BYTE(MPU_ADDR, MPU_FIFO_EN_REG, 0X00);       	//开启FIFO
	MPU6050_WRITE_BYTE(MPU_ADDR, MPU_INTBP_CFG_REG, 0X80);     	//INT引脚低电平有效
        res = MPU6050_READ_BYTE(MPU_ADDR, MPU_DEVICE_ID_REG);

	//器件ID正确
        if (res == MPU_ADDR)
        {
		//设置 CLKSEL,PLL X轴为参考
		MPU6050_WRITE_BYTE(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X01);
		//加速度与陀螺仪都工作
		MPU6050_WRITE_BYTE(MPU_ADDR, MPU_PWR_MGMT2_REG, 0X00);
		//设置采样率为50Hz
		MPU_Set_Rate(200);

        } else {
		return 1;
	}

        printf("mpu6050 init complete\r\n");
        return 0;
}

/**
 * @brief	设置MPU6050陀螺仪传感器满量程范围
 * @param fsr	0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * @return	返回值:0,设置成功，其他,设置失败
 */
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
        //return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
        return MPU6050_WRITE_BYTE(MPU_ADDR, MPU_GYRO_CFG_REG, fsr << 3);
}

/**
 * @brief	设置MPU6050加速度传感器满量程范围
 * @param fsr 	0,±2g;1,±4g;2,±8g;3,±16g
 * @return 	返回值:0,设置成功，其他,设置失败
 */
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
        //return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
        return MPU6050_WRITE_BYTE(MPU_ADDR, MPU_ACCEL_CFG_REG, fsr << 3);
}

/** 	
 * @brief	设置MPU6050的数字低通滤波器
 * @param lpf 	数字低通滤波频率(Hz)
 * @return 	返回值:0,设置成功，其他,设置失败
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
        uint8_t data = 0;
        if (lpf >= 188)data = 1;
        else if (lpf >= 98)data = 2;
        else if (lpf >= 42)data = 3;
        else if (lpf >= 20)data = 4;
        else if (lpf >= 10)data = 5;
        else data = 6;
        //return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
        return MPU6050_WRITE_BYTE(MPU_ADDR, MPU_CFG_REG, data);
}

/**
 * @brief	设置MPU6050的采样率(假定Fs=1KHz)
 * @param rate 	采样率，4~1000(Hz)
 * @return 	返回值:0,设置成功，其他,设置失败
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
        uint8_t data;
        if (rate > 1000)rate = 1000;
        if (rate < 4)rate = 4;
        data = 1000 / rate - 1;
        //data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
        data = MPU6050_WRITE_BYTE(MPU_ADDR, MPU_SAMPLE_RATE_REG, data);
        return MPU_Set_LPF(rate / 2);        //自动设置LPF为采样率的一半
}

/**
 * 得到温度值
 * @return 温度值(扩大了100倍)
 */
short MPU_Get_Temperature(void)
{
        uint8_t buf[2];
        short raw;
        float temp;
        //MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);

	MPU6050_READ(MPU_ADDR, MPU_TEMP_OUTH_REG, buf, 2);
        raw = ((uint16_t) buf[0] << 8) | buf[1];
        temp = 36.53 + ((double) raw) / 340;
        return temp * 100;;
}

/**
 * @brief 得到陀螺仪值(原始值)
 * @param gx x轴的原始读数(带符号)
 * @param gy y轴的原始读数(带符号)
 * @param gz z轴的原始读数(带符号)
 * @return
 */
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
        uint8_t buf[6], res;
        //res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
        res = MPU6050_READ(MPU_ADDR, MPU_GYRO_XOUTH_REG, buf, 6);
        if (res == 0) {
                *gx = ((uint16_t) buf[0] << 8) | buf[1];
                *gy = ((uint16_t) buf[2] << 8) | buf[3];
                *gz = ((uint16_t) buf[4] << 8) | buf[5];
        }
        return res;;
}

/**
 * @brief 得到陀螺仪值(原始值)
 * @param ax x轴的原始读数(带符号)
 * @param ay y轴的原始读数(带符号)
 * @param az z轴的原始读数(带符号)
 * @return
 */
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
        uint8_t buf[6], res;
        //res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
        res = MPU6050_READ(MPU_ADDR, MPU_ACCEL_XOUTH_REG, buf, 6);
        if (res == 0) {
                *ax = ((uint16_t) buf[0] << 8) | buf[1];
                *ay = ((uint16_t) buf[2] << 8) | buf[3];
                *az = ((uint16_t) buf[4] << 8) | buf[5];
        }
        return res;;
}

/**
 * @brief	IIC连续写
 * @param addr 	器件地址
 * @param reg 	寄存器地址
 * @param len 	写入长度
 * @param buf 	数据区
 * @return 	返回值:0,正常，其他,错误代码
 */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
        return MPU6050_WRITE(addr, reg, buf, len);
}

/**
 * @brief IIC连续读
 * @param addr:器件地址
 * @param reg:要读取的寄存器地址
 * @param len:要读取的长度
 * @param buf:读取到的数据存储区
 * @return 返回值:0,正常，其他,错误代码
 */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
        return MPU6050_READ(addr, reg, buf, len);
}

