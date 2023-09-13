#ifndef _BSP_DMA_RECV_H
#define _BSP_DMA_RECV_H

#include "gd32f4xx.h"
#include "systick.h"
#include "stdio.h"
#include "string.h"

//usart时钟
#define BSP_USART_RCU             RCU_USART0
//PA9<=>TX时钟
#define BSP_USART_TX_RCU          RCU_GPIOA
//PA10<=>RX时钟
#define BSP_USART_RX_RCU          RCU_GPIOA

//TX端口和引脚
#define BSP_USART_TX_PORT      GPIOA
#define BSP_USART_TX_PIN        GPIO_PIN_9
//RX端口和引脚
#define BSP_USART_RX_PORT       GPIOA
#define BSP_USART_RX_PIN        GPIO_PIN_10
//引脚复用的功能号
#define BSP_USART_AF        GPIO_AF_7 
// 串口0
#define BSP_USART 				USART0 
// 外部中断0
#define BSP_USART_IRQ		USART0_IRQn 
// 串口0中断服务函数
#define BSP_USART_DMA_IRQHandler  USART0_IRQHandler		
// fputc
#define DAM_FPUTC	fputc

//DMA时钟
#define BSP_DMA_RCU	RCU_DMA1
//DMA
#define BSP_DMA DMA1
//DMA channel
#define BSP_DMA_CHANNEL DMA_CH2
//DMA中断
#define BSP_DMA_CH_IRQ	DMA1_Channel2_IRQn
// DMA中断服务函数名
#define BSP_DMA_CH_IRQ_HANDLER DMA1_Channel2_IRQHandler  


/* 串口缓冲区的数据长度 */
#define USART_RECEIVE_LENGTH  4096
extern uint8_t  g1_recv_buff[USART_RECEIVE_LENGTH]; // 接收缓冲区
extern uint16_t g1_recv_length;										 // 接收数据长度
extern uint8_t  g1_recv_complete_flag; 						 // 接收完成标志位

// 配置DMA
void bsp_dma_config(void);
//配置usart
void bsp_usart_dma_init(uint32_t band_rate);
//发送字符
void bsp_usart_dma_send_data(uint8_t ucch);
//发送字符串
void bsp_usart_dma_send_string(uint8_t *ucstr);
//程序运行
void bsp_dma_recv_run(void);

#endif

