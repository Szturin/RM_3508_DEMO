#include "usart_app.h"

uint16_t uart_rx_index = 0;
uint16_t uart_rx_ticks = 0;
uint8_t uart_rx_buffer[128]={0};
uint8_t uart_rx_dma_buffer[128]={0};

ringbuffer_t usart_rb; //定义ringbuffer_t类型结构体变量
uint8_t usart_read_buffer[128];//定义环形缓存区数组

//串口中断回调函数

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == USART1)
//    {
//        uart_rx_ticks = uwTick;
//        uart_rx_index++;//索引自增
//        //每次触发回调，都要重新初始化接收中断，定义接收的位置
//        HAL_UART_Receive_IT(&huart1,&uart_rx_buffer[uart_rx_index],1);
//		//printf("test");//排错
//    }
//}


//空闲中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//printf("dma data:%s\n",uart_rx_dma_buffer);//发送串口接收内容

    //引入环形缓存区
    if(!ringbuffer_is_full(&usart_rb))//判断环形缓存区是否为空
    {
        ringbuffer_write(&usart_rb,uart_rx_dma_buffer,Size);//将DMA缓冲区中的数据写入环形缓冲区
    }
	memset(uart_rx_dma_buffer,0,sizeof(uart_rx_dma_buffer));//清空
}

void uart_proc(void)
{
//    if(uart_rx_index == 0) return;
//    
//    if(uwTick - uart_rx_ticks > 100)//时间超过100
//    {
//        printf("uart data:%s\n",uart_rx_buffer);//发送串口接收内容
//        
//        memset (uart_rx_buffer,0,uart_rx_index);//清空
//        uart_rx_index = 0;//指针指令
//        huart1.pRxBuffPtr = uart_rx_buffer;//uart1缓存区指针指向buffer
//    }
	// 如果环形缓冲区为空，直接返回 
	if(ringbuffer_is_empty(&usart_rb)) return;
	// 从环形缓冲区读取数据到读取缓冲区 
	ringbuffer_read(&usart_rb, usart_read_buffer, usart_rb.itemCount);
	// 打印读取缓冲区中的数据 
	//printf("ringbuffer data: %s\n", usart_read_buffer);
	// 上位机<test协议>
	printf("{plotter}%s\r\n", usart_read_buffer);
	
	// 清空读取缓冲区
	memset(usart_read_buffer, 0, sizeof(uint8_t) * BUUFER_SIZE);	
}
