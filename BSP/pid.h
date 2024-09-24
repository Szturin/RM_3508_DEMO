#ifndef __PID_H
#define __PID_H

#include "bsp_system.h"

typedef enum 
{
	PID_POSITION_SPEED = 0,
	PID_POSITION_ANGLE,
	PID_DELTA_SPEED
}PID_mode;

typedef struct
{
	PID_mode mode;			//模式	位置式速度；位置式角度；增量式速度
	float Kp;
	float Ki;
	float Kd;
	
	float Max_iout;		//最大积分输出限制
	float Max_out;		//最大总输出限制
	
	float measure;		//测量值
	float target;		//目标值
	float DeadBand;		//死区
	
	float Pout;
	float Iout;
	float Dout;
	float D_item;			//微分项	
	float error[3];			//偏差		0最新
	float OUT;				//输出
}PID_typedef;

typedef struct
{
	PID_mode mode;			//模式	位置式速度；位置式角度；增量式速度
	float Kp;
	float Ki;
	float Kd;

	float Max_iout;		//最大积分输出限制
	float Max_out;		//最大总输出限制

	float measure;		//测量值
	float target;		//目标值
	float DeadBand;		//死区
	
	float Pout;
	float Iout;
	float Dout;
	float D_item;			//微分项	
	float error[3];			//偏差		0最新
	float error_abs; 		//误差绝对值
	float OUT;				//输出
}Motor_PID_typedef;


/**
  * @brief          pid 结构体数据初始化
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION_SPEED: 位置式PID，速度
  *                 			PID_POSITION_ANGLE: 位置式PID，角度
	*  											PID_DELTA_SPEED		：差分式PID，速度
  * @param[in]      kp:PID参数p
	* @param[in] 			ki:PID参数i
	* @param[in] 			kd:PID参数d
  * @param[in]      Max_iout:pid最大积分输出
  * @param[in]      Max_out:pid最大输出
	* @param[in]			deadband:PID死区
  * @retval         none
  */
void PID_Init(PID_typedef *PID,PID_mode Mode,float kp,float ki,float kd,float Max_iout,float Max_out,float deadband);


float PID_calc(PID_typedef *PID, float measure, float target);
float Motor_PID_calc(Motor_PID_typedef *PID, float measure, float target);
float PID_calc_lite(Motor_PID_typedef *PID, float measure, float target, int mode);
void PID_Total_Init(void);
void LimitAcceleration(float target_speed, int16_t current_speed, int16_t max_speed_delta, float *buffered_speed);
#endif






