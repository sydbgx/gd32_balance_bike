#ifndef __USART_H
#define __USART_H
#include "gd32f4xx.h"
#include <stdio.h>


#define BSP_USART_CLOCK         RCU_USART0
#define BSP_USART_TX_CLOCK      RCU_GPIOA
#define BSP_USART_RX_CLOCK      RCU_GPIOA

#define BSP_USART_TX_GPIO_PORT  GPIOA
#define BSP_USART_RX_GPIO_PORT  GPIOA
#define BSP_USART_TX_GPIO_PIN   GPIO_PIN_9
#define BSP_USART_RX_GPIO_PIN   GPIO_PIN_10

#define BSP_USART_AF            GPIO_AF_7
#define BSP_USART_NUM           USART0

#define USART_RX_ENABLE 1

// value:波特率
void usart_init(uint32_t baudval);

//发送数据  value:发送的数据
void usart_send_byte(uint8_t Byte);
//发送字符串 value：发送的数据
void usart_send_string(uint8_t *String);

#if USART_RX_ENABLE
extern void usart0_receive(uint8_t *data, uint32_t length);
#endif

#endif
