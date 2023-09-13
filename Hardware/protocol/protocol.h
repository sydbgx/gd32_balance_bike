#ifndef __PROTOCOL__H__
#define __PROTOCOL__H__
#include <stdint.h>

#define RX_FRAME_HEADER 0x7b
#define RX_FRAME_TAIL         0x7a


#define TX_FRMAE_HEADER 0x7a
#define TX_FRAME_TAIL         0x7b

/*
uint8_t rxbuf[12] = {0};
uint8_t count = 0;
short x,y,z;
uint8_t FRMAE_HEADER = 0x7d;
uint8_t FRAME_TAIL = 0x7b;
*/
/*
        接收的数据：
        帧头	预留位 预留位  X速度	Y速度  Z速度  校验位  帧尾
        0x7a    1      1      2     2      2      1    0x7b   11位

        0x7a 0x00 0x00 0x11 0x11 0x11 0x11 0x12 0x13 0x12 0x7b

        发出去的数据：
        帧头 停止位  X速度 Y速度 Z速度 X加速度 Y加速度 Z加速度 X角速度 Y角速度 Z角速度 电压 校验位 帧尾
        0x7b    1     2     2    2     2         2     2      2       2       2      2    1   0x7a  24位

        short 是2个字节，范围-32768~32767
*/
typedef struct {
	uint8_t header;
	uint8_t code1;
	uint8_t code2;
	short x;
	short y;
	short z;
	uint8_t check;
	uint8_t tail;
} Protocol_RX;

typedef struct {
	uint8_t header;         /* 帧头 */
	uint8_t code;           /* 预留位 */
	short x;                /* X速度 */
	short y;                /* Y速度 */
	short z;                /* Z速度 */
	short ax;               /* X加速度 */
	short ay;               /* Y加速度 */
	short az;               /* Z加速度 */
	short gx;
	short gy;
	short gz;
	short voltage;          /* 电池电压 */
	uint8_t check;
	uint8_t tail;
} Protocol_TX;

/**
    解析接收到的数据
*/
void protocol_parse_xyz(uint8_t *recv_buff, int16_t datalength, float *x, float *y, float *z);
void protocol_parse_rx_pid(uint8_t *rxbuf, int16_t datalength, uint8_t *pidtype, float *kp, float *ki, float *kd);
void protocol_parse_rx(uint8_t *recv_buff, int16_t datalength);
uint8_t protocol_check_sum(uint8_t *buff, uint8_t length, uint8_t mode);
void protocol_print(Protocol_RX *rx);


#endif