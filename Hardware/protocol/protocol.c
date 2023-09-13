#include "protocol.h"
#include <stdio.h>


float g_balanceKP, g_balanceKI, g_balanceKD, g_velocityKP, g_velocityKI, g_velocityKD;

Protocol_RX g_rx_data;

/**
帧头(0x7d) | 标志位(0x00) | 长度(11) | X速度(2) | Y速度(2) | Z速度(1) | 校验位(0x7b) | 帧尾(11位)
*/
void protocol_parse_vel(uint8_t *buff, int16_t datalength)
{
        if (datalength >= 11) {
                if (buff[0] != RX_FRAME_HEADER) return;
                if (buff[0] != 0x00) return;
                if (buff[10] != RX_FRAME_TAIL) return;

                short x = buff[3] << 8 | buff[4];
                short y = buff[5] << 8 | buff[6];
                short z = buff[7] << 8 | buff[8];

                float realX = x * 1.0 / 100;
                float realY = y * 1.0 / 100;
                float realZ = z * 1.0 / 100;
        }
}

typedef union {
        float f;
        unsigned char b[4];
} Float2Bytes;

float bytes2float(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
        Float2Bytes fb;
        fb.b[0] = a;
        fb.b[1] = b;
        fb.b[2] = c;
        fb.b[3] = d;
        return fb.f;
}

/**
        接收的数据：
	帧头     标志位 	长度     X速度	Y速度  	Z速度	校验位 	帧尾
	0x7a    0x00  	11      2     	2       2       1    	0x7b   11位

        帧头     标志位	长度	kp	ki      kd	校验位1 	帧尾    直立环
  	0x7a	0x01  	17     	4   	4 	4	1	0x7b

        帧头     标志位 	长度	kp  	ki  	kd   	校验位1 	帧尾    速度环
        0x7a    0x02  	17     	4   	4  	4       1     	0x7b

	帧头     标志位   长度     kp      ki    kd    校验位1    帧尾     转向换
  	0x7a    0x03     17     4       4     4     1         0x7b

	浮点数据,放大100倍,转为整数发送

	0x7a 0x00 0x00 0x11 0x11 0x11 0x11 0x12 0x13 0x12 0x7b

	发出去的数据：
	帧头    停止位  X速度   Y速度   Z速度  X加速度   Y加速度  Z加速度  X角速度  Y角速度  Z角速度  电压  校验位  帧尾
	0x7b   1      2      2      2     2        2       2       2      2       2       2    1      0x7a

	short 是2个字节，范围-32768~32767
*/

void protocol_parse_rx_pid(uint8_t *rxbuf, int16_t datalength, uint8_t *pidtype, float *kp, float *ki, float *kd)
{

        uint16_t protocol_length = 18;

        if (datalength < protocol_length) {
                return;
        }
        printf("count=%d %x %x\n", datalength, rxbuf[0], rxbuf[16]);

        if (rxbuf[0] != RX_FRAME_HEADER)
                return;

        if (rxbuf[1] != 0x01)
                return;

        if (rxbuf[17] != RX_FRAME_TAIL)
                return;

        // pid的类型
        *pidtype = rxbuf[3];

        *kp = bytes2float(rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7]);
        *ki = bytes2float(rxbuf[8], rxbuf[9], rxbuf[10], rxbuf[11]);
        *kd = bytes2float(rxbuf[12], rxbuf[13], rxbuf[14], rxbuf[15]);

        printf("type=%d kp=%2f ki=%2f kd=%2f \r\n", *pidtype, *kp, *ki, *kd);

}
/**
        帧头	标志位 长度   X速度  Y速度   Z速度  校验位  帧尾
        0x7a    0x00  11    2     2      2      1     0x7b   11位
*/
void protocol_parse_xyz(uint8_t *rxbuf, int16_t datalength, float *x, float *y, float *z)
{
        uint16_t protocol_length = 11;

        if (datalength < protocol_length) {
                return;
        }
        printf("count=%d %x %x\n", datalength, rxbuf[0], rxbuf[protocol_length - 1]);

        if (rxbuf[0] != RX_FRAME_HEADER) return;

        if (rxbuf[1] != 0x00) return;

        if (rxbuf[protocol_length - 1] != RX_FRAME_TAIL) return;

        *x = ((float) ((rxbuf[3] << 8) | (rxbuf[4]))) / 100.0;
        *y = ((float) ((rxbuf[5] << 8) | (rxbuf[6]))) / 100.0;
        *z = ((float) ((rxbuf[7] << 8) | (rxbuf[8]))) / 100.0;


        printf("x=%2f y=%2f z=%2f \r\n", *x, *y, *z);
}

void protocol_parse_rx(uint8_t *buff, int16_t datalength)
{
/*
    接收的数据：
    帧头	标志位 长度  X速度	Y速度  Z速度  校验位  帧尾
    0x7d   0x00  11      2     2      2      1    0x7b   11位

    0x7d 0x00 11 0x11 0x11 0x11 0x11 0x12 0x13 0x12 0x7b

    发出去的数据：
    帧头 停止位  X速度 Y速度 Z速度 X加速度 Y加速度 Z加速度 X角速度 Y角速度 Z角速度 电压 校验位 帧尾
    0x7b    1     2     2    2     2         2     2      2       2       2      2    1   0x7d

    short 是2个字节，范围-32768~32767

    接收pid数据
    帧头  标志位 长度 直立环kp  ki  kd  速度环kp  ki	kd 校验位1 帧尾
    0x7d   0x01  29     4     4   4       4     4   4   1     0x7b
*/
        static uint8_t rxbuf[11] = {0};
        static uint16_t count = 0;
        printf("count=%d \n", datalength);
        if (datalength < 11) {
                return;
        }


        for (uint16_t i = 0; i < datalength; i++) {
                uint8_t data = buff[i];
                rxbuf[count] = data;


                if (data == RX_FRAME_HEADER || count > 0) {
                        count++;
                } else {
                        count = 0;
                }

                // 判断数据是否已经全部接收完成
                if (count == 11) {
                        // 判断帧尾
                        if (rxbuf[10] == RX_FRAME_TAIL) {
                                // 验证校验位
                                uint8_t check = protocol_check_sum(rxbuf, 9, 0);


                                // 判断接收到的是否和第9位相等
                                uint8_t error = 0;
                                if (rxbuf[9] != check) {
                                        error = 0;
                                }

                                if (error == 0) { // 若没有错误
                                        // 获取里面的数据
                                        //从串口数据求三轴目标速度，分高8位和低8位 单位mm/s
                                        short x = (short) ((rxbuf[3] << 8) + (rxbuf[4]));
                                        short y = (short) ((rxbuf[5] << 8) + (rxbuf[6]));
                                        short z = (short) ((rxbuf[7] << 8) + (rxbuf[8]));

                                        // 数据校验成功，将数据封装到结构体中，便于其它地方去使用
                                        g_rx_data.header = RX_FRAME_HEADER;
                                        g_rx_data.code1 = rxbuf[1];
                                        g_rx_data.code2 = rxbuf[2];
                                        g_rx_data.x = x;
                                        g_rx_data.y = y;
                                        g_rx_data.z = z;
                                        g_rx_data.check = rxbuf[9];
                                        g_rx_data.tail = RX_FRAME_TAIL;
                                }
                        }
                }
        }
}

/**
对校验位之前的所有数据进行校验
*/
uint8_t protocol_check_sum(uint8_t *buff, uint8_t length, uint8_t mode)
{
        uint8_t check_sum = 0;

//Validate the data to be sent
//对要发送的数据进行校验
        if (mode == 1)
                for (uint8_t k = 0; k < length; k++) {
                        check_sum = check_sum ^ buff[k];
                }

//Verify the data received
//对接收到的数据进行校验
        if (mode == 0)
                for (uint8_t k = 0; k < length; k++) {
                        check_sum = check_sum ^ buff[k];
                }
        return check_sum;
}

void protocol_rx_clean()
{
        g_rx_data.header = 0;
        g_rx_data.code1 = 0;
        g_rx_data.code2 = 0;
        g_rx_data.x = 0;
        g_rx_data.y = 0;
        g_rx_data.z = 0;
        g_rx_data.check = 0;
        g_rx_data.tail = 0;
}

#include <stdio.h>
void protocol_print(Protocol_RX *rx)
{
        printf("protocol_rx: header=%x, code1=%x, code2=%x, x=%d, y=%d, z=%d, check=%d,tail=%x\n",
               rx->header,
               rx->code1,
               rx->code2,
               rx->x,
               rx->y,
               rx->z,
               rx->check,
               rx->tail
        );
}