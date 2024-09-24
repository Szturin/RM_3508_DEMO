//系统链接文件,管理工程项目各个链接关系
#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "usart.h"

#include "scheduler.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_control.h"
#include "usart_app.h"
#include "ringbuffer.h"
#include "key_app.h"

#define BUUFER_SIZE 64

extern uint16_t uart_rx_index;
extern uint16_t uart_tx_ticks;
extern uint8_t uart_rx_buffer[128];
extern uint8_t uart_rx_dma_buffer[128];
extern uint32_t dma_buff[2][30];
extern float adc_value[2];
extern float set_speed;	 //电机速度全局变量

#endif