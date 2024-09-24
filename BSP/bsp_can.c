#include "bsp_can.h"
#include "main.h"
#include "string.h"
#include "usart.h"

// 外部声明CAN句柄
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 电机数据结构体数组
Motor_measure_t Motor_measure[14];

// CAN发送消息结构体与数据缓存
static CAN_TxHeaderTypeDef CANx_tx_message;
static uint8_t CANx_send_data[8];

/**
 * @brief CAN滤波器初始化
 *        配置两个CAN通道的过滤器
 */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;  // 定义过滤器结构体

    // CAN1过滤器配置
    can_filter_st.FilterActivation = ENABLE;                                // ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;                       // 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;                      // 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;                                    // ID高位
    can_filter_st.FilterIdLow = 0x0000;                                     // ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;                                // 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;                                 // 过滤器掩码低位
    can_filter_st.FilterBank = 0;                                           // 过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                      // 与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);                           // HAL库配置过滤器函数
    HAL_CAN_Start(&hcan1);                                                  // 使能CAN控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);      // 使能CAN的各种中断

    // CAN2过滤器配置
    can_filter_st.SlaveStartFilterBank = 14;                                // 双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;                                          // 过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                           // HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);                                                  // 使能CAN控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);      // 使能CAN的各种中断
}

/**
 * @brief CAN接收回调函数
 *        处理CAN接收到的数据帧
 * @param hcan CAN句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RX_Header;  // 定义数据帧的帧头
    uint8_t RX_BUFFER[8];           // 接收存放数据帧数据的数组

    // 获取CAN接收到的数据帧并存入局部变量
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RX_Header, RX_BUFFER);

    static uint8_t i = 0;

    // 判断接收的是CAN1还是CAN2
    if (hcan == &hcan1) {
        // CAN1：通过反馈数据的ID确定这一组数据存放的地址
        i = RX_Header.StdId - Chassis_3508A;
        Motor_measure_fun(&Motor_measure[i], RX_BUFFER);  // 调用函数把数据存入结构体数组
    } else if (hcan == &hcan2) {
        // CAN2：通过反馈数据的ID确定这一组数据存放的地址
        i = RX_Header.StdId - CAN2_3508_ID1 + 7;
        Motor_measure_fun(&Motor_measure[i], RX_BUFFER);  // 调用函数把数据存入结构体数组
    }
}

/**
 * @brief 解析电机测量数据
 * @param ptr 电机测量数据结构体指针
 * @param RX_buffer 接收到的CAN数据
 */
void Motor_measure_fun(Motor_measure_t *ptr, uint8_t* RX_buffer)
{
    // 记录上一次转子机械角度
    ptr->last_angle = ptr->angle;

    // 解析转子机械角度（高字节与低字节拼接）
    ptr->angle = (uint16_t)((RX_buffer)[0] << 8 | (RX_buffer)[1]);

    // 解析转子转速(rpm)
    ptr->speed = (uint16_t)((RX_buffer)[2] << 8 | (RX_buffer)[3]);

    // 解析转矩电流
    ptr->torque_current = (uint16_t)((RX_buffer)[4] << 8 | (RX_buffer)[5]);

    // 解析温度
    ptr->temp = (RX_buffer)[6];

    // 判断电机转子正向或反向过零，计算转子总圈数
    if (ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt++;

    // 计算转子总角度
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;
}

/**
 * @brief 设置并发送电机控制命令
 * @param hcan CAN句柄
 * @param STDID 标准ID
 * @param motor1 电机1控制值
 * @param motor2 电机2控制值
 * @param motor3 电机3控制值
 * @param motor4 电机4控制值
 */
void Set_motor_cmd(CAN_HandleTypeDef *hcan, uint32_t STDID,
                   int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;  // 定义一个变量用于存储发送邮箱编号

    // 设置发送消息的标识符
    CANx_tx_message.StdId = STDID;
    CANx_tx_message.IDE = CAN_ID_STD;  // 标识符选择位，STD-标准帧
    CANx_tx_message.RTR = CAN_RTR_DATA;  // 定义帧类型
    CANx_tx_message.DLC = 0x08;  // 数据帧长度为8位

    // 填充要发送的数据
    CANx_send_data[0] = motor1 >> 8;
    CANx_send_data[1] = motor1;
    CANx_send_data[2] = motor2 >> 8;
    CANx_send_data[3] = motor2;
    CANx_send_data[4] = motor3 >> 8;
    CANx_send_data[5] = motor3;
    CANx_send_data[6] = motor4 >> 8;
    CANx_send_data[7] = motor4;

    // 发送CAN数据
    HAL_CAN_AddTxMessage(hcan, &CANx_tx_message, CANx_send_data, &send_mail_box);  // hal库CAN发送函数：该函数用于向发送邮箱添加发送报文，并激活发送请求
}