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
	PID_mode mode;			//ģʽ	λ��ʽ�ٶȣ�λ��ʽ�Ƕȣ�����ʽ�ٶ�
	float Kp;
	float Ki;
	float Kd;
	
	float Max_iout;		//�������������
	float Max_out;		//������������
	
	float measure;		//����ֵ
	float target;		//Ŀ��ֵ
	float DeadBand;		//����
	
	float Pout;
	float Iout;
	float Dout;
	float D_item;			//΢����	
	float error[3];			//ƫ��		0����
	float OUT;				//���
}PID_typedef;

typedef struct
{
	PID_mode mode;			//ģʽ	λ��ʽ�ٶȣ�λ��ʽ�Ƕȣ�����ʽ�ٶ�
	float Kp;
	float Ki;
	float Kd;

	float Max_iout;		//�������������
	float Max_out;		//������������

	float measure;		//����ֵ
	float target;		//Ŀ��ֵ
	float DeadBand;		//����
	
	float Pout;
	float Iout;
	float Dout;
	float D_item;			//΢����	
	float error[3];			//ƫ��		0����
	float error_abs; 		//������ֵ
	float OUT;				//���
}Motor_PID_typedef;


/**
  * @brief          pid �ṹ�����ݳ�ʼ��
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION_SPEED: λ��ʽPID���ٶ�
  *                 			PID_POSITION_ANGLE: λ��ʽPID���Ƕ�
	*  											PID_DELTA_SPEED		�����ʽPID���ٶ�
  * @param[in]      kp:PID����p
	* @param[in] 			ki:PID����i
	* @param[in] 			kd:PID����d
  * @param[in]      Max_iout:pid���������
  * @param[in]      Max_out:pid������
	* @param[in]			deadband:PID����
  * @retval         none
  */
void PID_Init(PID_typedef *PID,PID_mode Mode,float kp,float ki,float kd,float Max_iout,float Max_out,float deadband);


float PID_calc(PID_typedef *PID, float measure, float target);
float Motor_PID_calc(Motor_PID_typedef *PID, float measure, float target);
float PID_calc_lite(Motor_PID_typedef *PID, float measure, float target, int mode);
void PID_Total_Init(void);
void LimitAcceleration(float target_speed, int16_t current_speed, int16_t max_speed_delta, float *buffered_speed);
#endif






