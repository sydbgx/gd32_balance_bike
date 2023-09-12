#include "usart.h"

// value:波特率
void usart_init(uint32_t baudval)
{
        /**
         * 缓冲区为 0，一有数据就发送
         */
        setvbuf(stdout, NULL, _IONBF, 0);

        //开启时钟
        rcu_periph_clock_enable(BSP_USART_CLOCK);
        rcu_periph_clock_enable(BSP_USART_TX_CLOCK);
        rcu_periph_clock_enable(BSP_USART_RX_CLOCK);
        //配置复用端口
        gpio_af_set(BSP_USART_RX_GPIO_PORT, BSP_USART_AF, BSP_USART_RX_GPIO_PIN);
        gpio_af_set(BSP_USART_TX_GPIO_PORT, BSP_USART_AF, BSP_USART_TX_GPIO_PIN);
        //配置gpio模式
        gpio_mode_set(BSP_USART_RX_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BSP_USART_RX_GPIO_PIN);
        gpio_mode_set(BSP_USART_TX_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BSP_USART_TX_GPIO_PIN);
        //配置gpio输出  推挽，50Mhz
        gpio_output_options_set(BSP_USART_RX_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, BSP_USART_RX_GPIO_PIN);
        gpio_output_options_set(BSP_USART_TX_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, BSP_USART_TX_GPIO_PIN);

        //使用一个外设前，最好复位一下
        usart_deinit(BSP_USART_NUM);
        //设置波特率
        usart_baudrate_set(BSP_USART_NUM, baudval);
        //配置校验
        usart_parity_config(BSP_USART_NUM, USART_PM_NONE);
        //配置数据长度
        usart_word_length_set(BSP_USART_NUM, USART_WL_8BIT);
        //配置停止位
        usart_stop_bit_set(BSP_USART_NUM, USART_STB_1BIT);

        //发送使能
        usart_transmit_config(BSP_USART_NUM, USART_TRANSMIT_ENABLE);
        //接收使能
        usart_receive_config(BSP_USART_NUM, USART_RECEIVE_ENABLE);
        //使能串口
        usart_enable(BSP_USART_NUM);
}

//发送数据  value:发送的数据
void usart_send_byte(uint8_t Byte)
{
        usart_data_transmit(BSP_USART_NUM, Byte);
        //等待数据发送缓冲区标志位
        while (usart_flag_get(BSP_USART_NUM, USART_FLAG_TBE) == RESET);
}

//发送字符串 value：发送的数据
void usart_send_string(uint8_t *String)
{
        //地址为空或值为空跳出
        while (String && *String) {
                usart_send_byte(*String++);
        }
}

//重写printf
int __io_putchar(int ch)
{
        usart_data_transmit(BSP_USART_NUM, (uint32_t) ch);
        //等待数据发送缓冲区标志位
        while (usart_flag_get(BSP_USART_NUM, USART_FLAG_TBE) == RESET);
        return ch;
}
int _write(int fd, char *pBuffer, int size)
{
        for (int i = 0; i < size; i++) {
                __io_putchar(*pBuffer++);
        }
        return size;
}