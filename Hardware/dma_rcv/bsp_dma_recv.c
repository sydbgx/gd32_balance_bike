#include "bsp_dma_recv.h"
#include <stdio.h>
#include "protocol.h"


extern float g_balanceKP, g_balanceKI, g_balanceKD, g_velocityKP, g_velocityKI, g_velocityKD;

uint8_t g1_recv_buff[USART_RECEIVE_LENGTH];       // 接收缓冲区
uint16_t g1_recv_length = 0;                      // 接收数据长度
uint8_t g1_recv_complete_flag = 0;                // 接收数据完成标志位

//usart配置
void bsp_usart_dma_init(uint32_t band_rate)
{
        //1.开启时钟
        rcu_periph_clock_enable(BSP_USART_TX_RCU); // 端口时钟
        rcu_periph_clock_enable(BSP_USART_RX_RCU); // 端口时钟
        rcu_periph_clock_enable(BSP_USART_RCU);    // 串口时钟
        //2.配置GPIO复用模式
        gpio_af_set(BSP_USART_TX_PORT, BSP_USART_AF, BSP_USART_TX_PIN);
        gpio_af_set(BSP_USART_RX_PORT, BSP_USART_AF, BSP_USART_RX_PIN);
        //3.配置GPIO的模式
        /* 配置TX为复用模式 上拉模式 */
        gpio_mode_set(BSP_USART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BSP_USART_TX_PIN);
        /* 配置RX为复用模式 上拉模式 */
        gpio_mode_set(BSP_USART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BSP_USART_RX_PIN);
        //4.配置GPIO的输出
        /* 配置TX为推挽输出 50MHZ */
        gpio_output_options_set(BSP_USART_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_USART_TX_PIN);
        /* 配置RX为推挽输出 50MHZ */
        gpio_output_options_set(BSP_USART_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_USART_RX_PIN);
        //5.配置串口参数
        usart_deinit(BSP_USART); // 复位串口
        usart_baudrate_set(BSP_USART, band_rate); // 设置波特率
        usart_parity_config(BSP_USART, USART_PM_NONE); // 没有校验位
        usart_word_length_set(BSP_USART, USART_WL_8BIT); // 8位数据位
        usart_stop_bit_set(BSP_USART, USART_STB_1BIT); // 1位停止位
        //6.使能串口
        usart_enable(BSP_USART);
        //串口发送
        usart_transmit_config(BSP_USART, USART_TRANSMIT_ENABLE);
        //串口接收
        usart_receive_config(BSP_USART, USART_RECEIVE_ENABLE);
        //7.中断配置
        //中断优先级
        nvic_irq_enable(BSP_USART_IRQ, 2, 2);
        //读取数据缓冲区非空中断和过载错误中断
        usart_interrupt_enable(BSP_USART, USART_INT_RBNE);
        //空闲检测中断
        usart_interrupt_enable(BSP_USART, USART_INT_IDLE);

}

//DMA配置
void bsp_dma_config(void)
{
        //1.DMA单数据结构体
        dma_single_data_parameter_struct dma_init_struct;
        //2.开启DMA时钟
        rcu_periph_clock_enable(BSP_DMA_RCU);
        //3.初始化DMA通道
        dma_deinit(BSP_DMA, BSP_DMA_CHANNEL);
        //4.配置DMA初始化参数
        // 外设地址
        dma_init_struct.periph_addr = (uint32_t) &USART_DATA(BSP_USART);
        // 不使用增量模式，为固定模式
        dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        // 内存地址(用来接收数据)
        dma_init_struct.memory0_addr = (uint32_t) g1_recv_buff;
        // 增量模式
        dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        // 一次传输长度8bit
        dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
        // 关闭循环模式
        dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
        // 外设到内存
        dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
        // 要传输的数据量
        dma_init_struct.number = USART_RECEIVE_LENGTH;
        // 超高优先级
        dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
        //5.初始化DMA结构体
        dma_single_data_mode_init(BSP_DMA, BSP_DMA_CHANNEL, &dma_init_struct);
        //6.使能通道外设
        dma_channel_subperipheral_select(BSP_DMA, BSP_DMA_CHANNEL, DMA_SUBPERI4);
        //7.使能DMA通道
        dma_channel_enable(BSP_DMA, BSP_DMA_CHANNEL);
        //8.使能DMA通道中断
        dma_interrupt_enable(BSP_DMA, BSP_DMA_CHANNEL, DMA_CHXCTL_FTFIE);
        //9.配置中断优先级
        nvic_irq_enable(BSP_DMA_CH_IRQ, 2, 1);
        //10.使能串口DMA接收
        usart_dma_receive_config(BSP_USART, USART_RECEIVE_DMA_ENABLE);
}

//DMA中断服务函数
void BSP_DMA_CH_IRQ_HANDLER(void)
{
        // 传输完成中断
        if (dma_interrupt_flag_get(BSP_DMA, BSP_DMA_CHANNEL, DMA_INT_FLAG_FTF) == SET) {
                // 清中断标志位
                dma_interrupt_flag_clear(BSP_DMA, BSP_DMA_CHANNEL, DMA_INT_FLAG_FTF);
        }
}

//串口接收中断服务函数
void BSP_USART_DMA_IRQHandler(void)
{
        // 检测到帧中断
        // 检测到帧中断
        if (usart_interrupt_flag_get(BSP_USART,
                                     USART_INT_FLAG_IDLE) == SET)
        {
                // 必须要读，读出来的值不能要
                usart_data_receive(BSP_USART);                                                                                                                                                                                                                                          // 必须要读，读出来的值不能要

                /* 处理DMA接收到的数据 */
                // 计算实际接收的数据长度
                g1_recv_length = USART_RECEIVE_LENGTH -
                        dma_transfer_number_get(BSP_DMA, BSP_DMA_CHANNEL);
                //g1_recv_buff[g1_recv_length] = '\0';																											 // 数据接收完毕，数组结束标志

                uint8_t pid_type = 0;
                float KP, KI, KD = 0;
                float x, y, z;
                printf("protocol start parse: %X %x %d\r\n", g1_recv_buff[0], g1_recv_buff[1], g1_recv_length);
                // 根据标志位去解析
                if (g1_recv_buff[0] == RX_FRAME_HEADER) {
                        switch (g1_recv_buff[1]) {
                        case 0x00:
                                protocol_parse_xyz(g1_recv_buff, g1_recv_length, &x, &y, &z);
                                break;
                        case 0x01:
                                protocol_parse_rx_pid(g1_recv_buff, g1_recv_length, &pid_type, &KP, &KI, &KD);
                                switch (pid_type) {
                                case 0x00:
                                        g_balanceKP = KP;
                                        g_balanceKI = KI;
                                        g_balanceKD = KD;
                                        break;
                                case 0x01:
                                        g_velocityKP = KP;
                                        g_velocityKI = KI;
                                        g_velocityKD = KD;
                                        break;
                                }
                                break;
                        default:
                                printf("protocol not parse: %x\r\n", g1_recv_buff[1]);
                                break;

                        }
                }

                /* 重新设置DMA传输 */
                // 失能DMA通道
                dma_channel_disable(BSP_DMA, BSP_DMA_CHANNEL);
                // 重新配置DMA进行传输
                bsp_dma_config();

        }
        if (usart_interrupt_flag_get(BSP_USART, USART_INT_FLAG_RBNE) == SET) {
                dma_interrupt_flag_clear(BSP_DMA, BSP_DMA_CHANNEL, USART_INT_FLAG_RBNE);
        }
        if (usart_interrupt_flag_get(BSP_USART, USART_INT_FLAG_RBNE_ORERR) == SET) {
                dma_interrupt_flag_clear(BSP_DMA, BSP_DMA_CHANNEL, USART_INT_FLAG_RBNE_ORERR);
        }
}

//发送一个字符
void bsp_usart_dma_send_data(uint8_t ucch)
{
        usart_data_transmit(BSP_USART, (uint8_t) ucch);                                                         // 发送数据
        while (RESET == usart_flag_get(BSP_USART, USART_FLAG_TBE));  // 等待发送数据缓冲区标志置位
}
//发送字符串
void bsp_usart_dma_send_string(uint8_t *ucstr)
{
        while (ucstr && *ucstr) {                        // 地址为空或者值为空跳出
                bsp_usart_dma_send_data(*ucstr++);  // 发送单个字符
        }
}

//重写fputc
int fputc(int ch, FILE *f)
{
        bsp_usart_dma_send_data(ch);
        // 等待发送数据缓冲区标志置位
        return ch;
}