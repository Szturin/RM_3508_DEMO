#include "pid.h"
#include <math.h>
#include <main.h>

// 定义14个电机的PID结构体数组

Motor_PID_typedef Motor_pid_POSITION[4];
Motor_PID_typedef Motor_pid_SPEED[4];
/**
  * @brief          输出值限制宏定义
  * @param[in,out]  input: 要限制的值
  * @param[in]      max: 最大输出值
  */
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          PID结构体初始化
  *                 将计算的相关参数传递到PID结构体中
  * @param[out]     PID: PID结构数据指针
  * @param[in]      Mode: PID模式 
  *                    PID_POSITION_SPEED: 位置式PID，速度
  *                    PID_POSITION_ANGLE: 位置式PID，角度
  *                    PID_DELTA_SPEED: 增量式PID，速度
  * @param[in]      kp: PID参数P
  * @param[in]      ki: PID参数I
  * @param[in]      kd: PID参数D
  * @param[in]      Max_iout: PID最大积分输出
  * @param[in]      Max_out: PID最大输出
  * @param[in]      deadband: PID死区
  * @retval         none
  */
void PID_Init(PID_typedef *PID, PID_mode Mode, float kp, float ki, float kd, float Max_iout, float Max_out, float deadband)
{
    if (PID == NULL) return;

    // 将参数传递到PID结构体中
    PID->mode = Mode;
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->Max_iout = Max_iout;
    PID->Max_out = Max_out;
    PID->DeadBand = deadband;
}

/**
  * @brief          电机PID结构体初始化
  *                 将计算的相关参数传递到PID结构体中
  * @param[out]     PID: PID结构数据指针
  * @param[in]      Mode: PID模式 
  *                    PID_POSITION_SPEED: 位置式PID，速度
  *                    PID_POSITION_ANGLE: 位置式PID，角度
  *                    PID_DELTA_SPEED: 增量式PID，速度
  * @param[in]      kp: PID参数P
  * @param[in]      ki: PID参数I
  * @param[in]      kd: PID参数D
  * @param[in]      Max_iout: PID最大积分输出
  * @param[in]      Max_out: PID最大输出
  * @param[in]      deadband: PID死区
  * @retval         none
  */
void Motor_PID_Init(Motor_PID_typedef *PID, PID_mode Mode, float kp, float ki, float kd, float Max_iout, float Max_out, float deadband)
{
    if (PID == NULL) return;

    // 将参数传递到PID结构体中
    PID->mode = Mode;
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->Max_iout = Max_iout;
    PID->Max_out = Max_out;
    PID->DeadBand = deadband;
}

/**
  * @brief          PID计算
  * @param[out]     PID: PID结构数据指针
  * @param[in]      measure: 反馈测量数据
  * @param[in]      target: 目标值
  * @retval         PID输出值
  */
float PID_calc(PID_typedef *PID, float measure, float target)
{
    if (PID == NULL)
        return 0;
    
    // 更新误差值，2为上上次误差，1为上次误差，0为当前误差
    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->measure = measure;
    PID->target = target;
    PID->error[0] = target - measure;
    
    if (fabsf(PID->error[0]) > PID->DeadBand || PID->DeadBand == 0) 
    {
        // 判断是否在不同模式下进行PID计算
        if (PID->mode == PID_DELTA_SPEED) // 增量式PID计算
        { 
            // PID计算公式
            PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]);  // 增量式P输出 (P项使用误差增量)
            PID->Iout = PID->Ki * PID->error[0];                    // I输出 (不要累加)
            PID->Dout = PID->Kd * (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]);  // D项

            // 计算总输出，增量式PID需要对输出增量进行累加
            PID->OUT += PID->Pout + PID->Iout + PID->Dout;  
            LimitMax(PID->OUT, PID->Max_out);                       // 限制总输出
        }
        else if (PID->mode == PID_POSITION_SPEED || PID->mode == PID_POSITION_ANGLE) // 位置式PID计算
        {
            if (PID->mode == PID_POSITION_ANGLE)
            {
                // 处理角度误差，使其在 -4096 至 +4096 范围内
                if (PID->error[0] > 4096) PID->error[0] -= 8192;
                else if (PID->error[0] < -4096) PID->error[0] += 8192;
            }
            
            // 位置式PID计算
            PID->Pout = PID->Kp * PID->error[0];                     // P输出
            PID->Iout += PID->Ki * PID->error[0];                    // 积分累加
            PID->Dout = PID->Kd * (PID->error[0] - PID->error[1]);   // D项

            // 积分抗饱和处理
            LimitMax(PID->Iout, PID->Max_iout);                      // 限制I输出

            // 计算总输出
            PID->OUT = PID->Pout + PID->Iout + PID->Dout;            // 输出值累加
            LimitMax(PID->OUT, PID->Max_out);                        // 限制总输出
        }
    }
    else
    {
        PID->OUT = 0;  // 误差在死区内，输出为0
    }
    
    return PID->OUT;  // 返回PID输出值
}

/**
  * @brief          PID计算
  * @param[out]     PID: PID结构数据指针
  * @param[in]      measure: 反馈测量数据
  * @param[in]      target: 目标值
  * @retval         PID输出值
  */
float Motor_PID_calc(Motor_PID_typedef *PID, float measure, float target)
{
    if (PID == NULL)
        return 0;
    
    float StaticDif_Err = 1.0f;     //静差(余差)
    float StaticDif_Out = 1.0f;     //静差
	
    // 更新误差值，2为上上次误差，1为上次误差，0为当前误差
    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->measure = measure;
    PID->target = target;
	
	/*误差计算*/
    PID->error[0] = target - measure;
	
    /* 误差较小时取消误差 */
	
    if(fabs(PID->error[0]) < StaticDif_Err) {
        PID->error[0]= 0;
    }

    //if ((fabs(PID->error[0]) > PID->DeadBand) || PID->DeadBand == 0) //错误写法，造成输出一但在误差小于死区，总输出就置零
	if ((fabs(PID->target) > PID->DeadBand) || PID->DeadBand == 0)
    {
        // 判断是否在不同模式下进行PID计算
        if (PID->mode == PID_DELTA_SPEED) // 增量式PID计算
        { 
            // PID计算公式
            PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]);  // 增量式P输出 (P项使用误差增量)
            PID->Iout = PID->Ki * PID->error[0];                    // I输出 (不要累加)
			PID->D_item = (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]); // D项计算
			PID->Dout = PID->Kd * PID->D_item;                    // D输出

            // 计算总输出，增量式PID需要对输出增量进行累加
            PID->OUT += PID->Pout + PID->Iout + PID->Dout;  
            LimitMax(PID->OUT, PID->Max_out);                       // 限制总输出
        }
        else if (PID->mode == PID_POSITION_SPEED || PID->mode == PID_POSITION_ANGLE) // 位置式PID计算
        {
            if (PID->mode == PID_POSITION_ANGLE)
            {
                // 处理角度误差，使其在 -4096 至 +4096 范围内
                if (PID->error[0] > 4096) PID->error[0] -= 8192;
                else if (PID->error[0] < -4096) PID->error[0] += 8192;
            }
            
            // 位置式PID计算
            PID->Pout = PID->Kp * PID->error[0];                     // P输出
            PID->Iout += PID->Ki * PID->error[0];                    // 积分累加
            PID->Dout = PID->Kd * (PID->error[0] - PID->error[1]);   // D项

            // 积分抗饱和处理
            LimitMax(PID->Iout, PID->Max_iout);                      // 限制I输出

            // 计算总输出
            PID->OUT = PID->Pout + PID->Iout + PID->Dout;            // 输出值累加
            LimitMax(PID->OUT, PID->Max_out);                        // 限制总输出
        }
	}
	else
	{
		PID->OUT = 0;  // 误差在死区内，输出为0
	}
    
    /* 消除静差 */
    if(fabs(PID->OUT) < StaticDif_Out) {
        PID->OUT = 0.0f;	
	}
    return PID->OUT;  // 返回PID输出值
}


float PID_calc_lite(Motor_PID_typedef *PID, float measure, float target, int mode)
{
    if (PID == NULL)
        return 0;

    // 更新误差值，2为上上次误差，1为上次误差，0为当前误差
    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->measure = measure;
    PID->target = target;
    PID->error[0] = target - measure;

    // 判断是否在不同模式下进行PID计算
    if (mode == PID_DELTA_SPEED)
    { 
        // 增量式PID计算
        if (mode == PID_POSITION_ANGLE)
        {
            // 处理角度误差，使其在 -4096 至 +4096 范围内
            if (PID->error[0] > 4096) PID->error[0] = PID->error[0] - 8191;
            else if (PID->error[0] < -4096) PID->error[0] = PID->error[0] + 8191;
        }

        // PID计算公式
        PID->Pout = PID->Kp * PID->error[0];             // P输出
        PID->Iout += PID->Ki * PID->error[0];            // I输出累加
        PID->D_item = (PID->error[0] - PID->error[1]);   // 计算D项
        PID->Dout = PID->Kd * PID->D_item;               // D输出

        LimitMax(PID->Iout, PID->Max_iout);              // 限制I输出
        PID->OUT = PID->Pout + PID->Iout + PID->Dout;    // 计算总输出
        LimitMax(PID->OUT, PID->Max_out);                // 限制总输出
    }
    else if (mode == PID_POSITION_SPEED || mode == PID_POSITION_ANGLE)
    {
        // 位置式PID计算
        PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]); // P输出
        PID->Iout = PID->Ki * PID->error[0];                  // I输出
        PID->D_item = (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]); // D项计算
        PID->Dout = PID->Kd * PID->D_item;                    // D输出

        PID->OUT += PID->Pout + PID->Iout + PID->Dout;        // 位置式总输出
        LimitMax(PID->OUT, PID->Max_out);                     // 限制输出
    }
    else
    {
        PID->OUT = 0;  // 误差在死区内，输出为0
    }

    return PID->OUT;  // 返回PID输出值
}

/**
  * @brief          PID参数总初始化
  *                 初始化每个电机的PID参数
  * @retval         none
  */
void PID_Total_Init(void)
{
    // 初始化电机1的PID参数
    //PID_Init(&Motor_pid_POSITION[0], PID_POSITION_SPEED, 0, 0, 0, 8000, 16384, 0);  // M2006 电机 ID:1
	
	//增量式PID参数初始化
	//死区：300 
	Motor_PID_Init(&Motor_pid_SPEED[0], PID_DELTA_SPEED, 9.449, 0.655, 5, 8000, 16384, 100);  // M2006 电机 ID:1
    // 这里可以继续初始化其他电机的PID参数
}

/**
  * @brief          加减速缓冲，防止速度变化过快
  * @param[in]      target_speed: 指向目标速度的指针
  * @param[in]      current_speed: 当前速度
  * @param[in]      max_speed_delta: 最大允许的速度变化量（加速度/减速度限制）
  * @param[out]     buffered_speed: 指向缓冲后速度的指针
  * @retval         none
  */
void LimitAcceleration(float target_speed, int16_t current_speed, int16_t max_speed_delta, float *buffered_speed)
{
    int16_t speed_difference = (int16_t)(target_speed) - current_speed;  // 计算目标速度与当前速度的差值
	
	if ((speed_difference > max_speed_delta) && (*buffered_speed < target_speed))
	{
		*buffered_speed += max_speed_delta; // 限制最大加速度
		
		if(fabs(*buffered_speed) >= fabs(target_speed))
		{
			*buffered_speed = target_speed;  // 如果在允许范围内，则直接使用目标速度		
		}
	}
	else if ((speed_difference < -max_speed_delta) && (*buffered_speed > target_speed))
	{
		*buffered_speed -= max_speed_delta;  // 限制最大减速度	
		if(fabs(*buffered_speed) <= fabs(target_speed))
		{
			*buffered_speed = target_speed;  // 如果在允许范围内，则直接使用目标速度		
		}
	}
	else
	{
		*buffered_speed = target_speed;  // 如果在允许范围内，则直接使用目标速度		
	}
	
	
}