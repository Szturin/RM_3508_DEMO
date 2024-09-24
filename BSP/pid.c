#include "pid.h"
#include <math.h>
#include <main.h>

// ����14�������PID�ṹ������

Motor_PID_typedef Motor_pid_POSITION[4];
Motor_PID_typedef Motor_pid_SPEED[4];
/**
  * @brief          ���ֵ���ƺ궨��
  * @param[in,out]  input: Ҫ���Ƶ�ֵ
  * @param[in]      max: ������ֵ
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
  * @brief          PID�ṹ���ʼ��
  *                 ���������ز������ݵ�PID�ṹ����
  * @param[out]     PID: PID�ṹ����ָ��
  * @param[in]      Mode: PIDģʽ 
  *                    PID_POSITION_SPEED: λ��ʽPID���ٶ�
  *                    PID_POSITION_ANGLE: λ��ʽPID���Ƕ�
  *                    PID_DELTA_SPEED: ����ʽPID���ٶ�
  * @param[in]      kp: PID����P
  * @param[in]      ki: PID����I
  * @param[in]      kd: PID����D
  * @param[in]      Max_iout: PID���������
  * @param[in]      Max_out: PID������
  * @param[in]      deadband: PID����
  * @retval         none
  */
void PID_Init(PID_typedef *PID, PID_mode Mode, float kp, float ki, float kd, float Max_iout, float Max_out, float deadband)
{
    if (PID == NULL) return;

    // ���������ݵ�PID�ṹ����
    PID->mode = Mode;
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->Max_iout = Max_iout;
    PID->Max_out = Max_out;
    PID->DeadBand = deadband;
}

/**
  * @brief          ���PID�ṹ���ʼ��
  *                 ���������ز������ݵ�PID�ṹ����
  * @param[out]     PID: PID�ṹ����ָ��
  * @param[in]      Mode: PIDģʽ 
  *                    PID_POSITION_SPEED: λ��ʽPID���ٶ�
  *                    PID_POSITION_ANGLE: λ��ʽPID���Ƕ�
  *                    PID_DELTA_SPEED: ����ʽPID���ٶ�
  * @param[in]      kp: PID����P
  * @param[in]      ki: PID����I
  * @param[in]      kd: PID����D
  * @param[in]      Max_iout: PID���������
  * @param[in]      Max_out: PID������
  * @param[in]      deadband: PID����
  * @retval         none
  */
void Motor_PID_Init(Motor_PID_typedef *PID, PID_mode Mode, float kp, float ki, float kd, float Max_iout, float Max_out, float deadband)
{
    if (PID == NULL) return;

    // ���������ݵ�PID�ṹ����
    PID->mode = Mode;
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->Max_iout = Max_iout;
    PID->Max_out = Max_out;
    PID->DeadBand = deadband;
}

/**
  * @brief          PID����
  * @param[out]     PID: PID�ṹ����ָ��
  * @param[in]      measure: ������������
  * @param[in]      target: Ŀ��ֵ
  * @retval         PID���ֵ
  */
float PID_calc(PID_typedef *PID, float measure, float target)
{
    if (PID == NULL)
        return 0;
    
    // �������ֵ��2Ϊ���ϴ���1Ϊ�ϴ���0Ϊ��ǰ���
    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->measure = measure;
    PID->target = target;
    PID->error[0] = target - measure;
    
    if (fabsf(PID->error[0]) > PID->DeadBand || PID->DeadBand == 0) 
    {
        // �ж��Ƿ��ڲ�ͬģʽ�½���PID����
        if (PID->mode == PID_DELTA_SPEED) // ����ʽPID����
        { 
            // PID���㹫ʽ
            PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]);  // ����ʽP��� (P��ʹ���������)
            PID->Iout = PID->Ki * PID->error[0];                    // I��� (��Ҫ�ۼ�)
            PID->Dout = PID->Kd * (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]);  // D��

            // ���������������ʽPID��Ҫ��������������ۼ�
            PID->OUT += PID->Pout + PID->Iout + PID->Dout;  
            LimitMax(PID->OUT, PID->Max_out);                       // ���������
        }
        else if (PID->mode == PID_POSITION_SPEED || PID->mode == PID_POSITION_ANGLE) // λ��ʽPID����
        {
            if (PID->mode == PID_POSITION_ANGLE)
            {
                // ����Ƕ���ʹ���� -4096 �� +4096 ��Χ��
                if (PID->error[0] > 4096) PID->error[0] -= 8192;
                else if (PID->error[0] < -4096) PID->error[0] += 8192;
            }
            
            // λ��ʽPID����
            PID->Pout = PID->Kp * PID->error[0];                     // P���
            PID->Iout += PID->Ki * PID->error[0];                    // �����ۼ�
            PID->Dout = PID->Kd * (PID->error[0] - PID->error[1]);   // D��

            // ���ֿ����ʹ���
            LimitMax(PID->Iout, PID->Max_iout);                      // ����I���

            // ���������
            PID->OUT = PID->Pout + PID->Iout + PID->Dout;            // ���ֵ�ۼ�
            LimitMax(PID->OUT, PID->Max_out);                        // ���������
        }
    }
    else
    {
        PID->OUT = 0;  // ����������ڣ����Ϊ0
    }
    
    return PID->OUT;  // ����PID���ֵ
}

/**
  * @brief          PID����
  * @param[out]     PID: PID�ṹ����ָ��
  * @param[in]      measure: ������������
  * @param[in]      target: Ŀ��ֵ
  * @retval         PID���ֵ
  */
float Motor_PID_calc(Motor_PID_typedef *PID, float measure, float target)
{
    if (PID == NULL)
        return 0;
    
    float StaticDif_Err = 1.0f;     //����(���)
    float StaticDif_Out = 1.0f;     //����
	
    // �������ֵ��2Ϊ���ϴ���1Ϊ�ϴ���0Ϊ��ǰ���
    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->measure = measure;
    PID->target = target;
	
	/*������*/
    PID->error[0] = target - measure;
	
    /* ����Сʱȡ����� */
	
    if(fabs(PID->error[0]) < StaticDif_Err) {
        PID->error[0]= 0;
    }

    //if ((fabs(PID->error[0]) > PID->DeadBand) || PID->DeadBand == 0) //����д����������һ�������С�������������������
	if ((fabs(PID->target) > PID->DeadBand) || PID->DeadBand == 0)
    {
        // �ж��Ƿ��ڲ�ͬģʽ�½���PID����
        if (PID->mode == PID_DELTA_SPEED) // ����ʽPID����
        { 
            // PID���㹫ʽ
            PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]);  // ����ʽP��� (P��ʹ���������)
            PID->Iout = PID->Ki * PID->error[0];                    // I��� (��Ҫ�ۼ�)
			PID->D_item = (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]); // D�����
			PID->Dout = PID->Kd * PID->D_item;                    // D���

            // ���������������ʽPID��Ҫ��������������ۼ�
            PID->OUT += PID->Pout + PID->Iout + PID->Dout;  
            LimitMax(PID->OUT, PID->Max_out);                       // ���������
        }
        else if (PID->mode == PID_POSITION_SPEED || PID->mode == PID_POSITION_ANGLE) // λ��ʽPID����
        {
            if (PID->mode == PID_POSITION_ANGLE)
            {
                // ����Ƕ���ʹ���� -4096 �� +4096 ��Χ��
                if (PID->error[0] > 4096) PID->error[0] -= 8192;
                else if (PID->error[0] < -4096) PID->error[0] += 8192;
            }
            
            // λ��ʽPID����
            PID->Pout = PID->Kp * PID->error[0];                     // P���
            PID->Iout += PID->Ki * PID->error[0];                    // �����ۼ�
            PID->Dout = PID->Kd * (PID->error[0] - PID->error[1]);   // D��

            // ���ֿ����ʹ���
            LimitMax(PID->Iout, PID->Max_iout);                      // ����I���

            // ���������
            PID->OUT = PID->Pout + PID->Iout + PID->Dout;            // ���ֵ�ۼ�
            LimitMax(PID->OUT, PID->Max_out);                        // ���������
        }
	}
	else
	{
		PID->OUT = 0;  // ����������ڣ����Ϊ0
	}
    
    /* �������� */
    if(fabs(PID->OUT) < StaticDif_Out) {
        PID->OUT = 0.0f;	
	}
    return PID->OUT;  // ����PID���ֵ
}


float PID_calc_lite(Motor_PID_typedef *PID, float measure, float target, int mode)
{
    if (PID == NULL)
        return 0;

    // �������ֵ��2Ϊ���ϴ���1Ϊ�ϴ���0Ϊ��ǰ���
    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->measure = measure;
    PID->target = target;
    PID->error[0] = target - measure;

    // �ж��Ƿ��ڲ�ͬģʽ�½���PID����
    if (mode == PID_DELTA_SPEED)
    { 
        // ����ʽPID����
        if (mode == PID_POSITION_ANGLE)
        {
            // ����Ƕ���ʹ���� -4096 �� +4096 ��Χ��
            if (PID->error[0] > 4096) PID->error[0] = PID->error[0] - 8191;
            else if (PID->error[0] < -4096) PID->error[0] = PID->error[0] + 8191;
        }

        // PID���㹫ʽ
        PID->Pout = PID->Kp * PID->error[0];             // P���
        PID->Iout += PID->Ki * PID->error[0];            // I����ۼ�
        PID->D_item = (PID->error[0] - PID->error[1]);   // ����D��
        PID->Dout = PID->Kd * PID->D_item;               // D���

        LimitMax(PID->Iout, PID->Max_iout);              // ����I���
        PID->OUT = PID->Pout + PID->Iout + PID->Dout;    // ���������
        LimitMax(PID->OUT, PID->Max_out);                // ���������
    }
    else if (mode == PID_POSITION_SPEED || mode == PID_POSITION_ANGLE)
    {
        // λ��ʽPID����
        PID->Pout = PID->Kp * (PID->error[0] - PID->error[1]); // P���
        PID->Iout = PID->Ki * PID->error[0];                  // I���
        PID->D_item = (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]); // D�����
        PID->Dout = PID->Kd * PID->D_item;                    // D���

        PID->OUT += PID->Pout + PID->Iout + PID->Dout;        // λ��ʽ�����
        LimitMax(PID->OUT, PID->Max_out);                     // �������
    }
    else
    {
        PID->OUT = 0;  // ����������ڣ����Ϊ0
    }

    return PID->OUT;  // ����PID���ֵ
}

/**
  * @brief          PID�����ܳ�ʼ��
  *                 ��ʼ��ÿ�������PID����
  * @retval         none
  */
void PID_Total_Init(void)
{
    // ��ʼ�����1��PID����
    //PID_Init(&Motor_pid_POSITION[0], PID_POSITION_SPEED, 0, 0, 0, 8000, 16384, 0);  // M2006 ��� ID:1
	
	//����ʽPID������ʼ��
	//������300 
	Motor_PID_Init(&Motor_pid_SPEED[0], PID_DELTA_SPEED, 9.449, 0.655, 5, 8000, 16384, 100);  // M2006 ��� ID:1
    // ������Լ�����ʼ�����������PID����
}

/**
  * @brief          �Ӽ��ٻ��壬��ֹ�ٶȱ仯����
  * @param[in]      target_speed: ָ��Ŀ���ٶȵ�ָ��
  * @param[in]      current_speed: ��ǰ�ٶ�
  * @param[in]      max_speed_delta: ���������ٶȱ仯�������ٶ�/���ٶ����ƣ�
  * @param[out]     buffered_speed: ָ�򻺳���ٶȵ�ָ��
  * @retval         none
  */
void LimitAcceleration(float target_speed, int16_t current_speed, int16_t max_speed_delta, float *buffered_speed)
{
    int16_t speed_difference = (int16_t)(target_speed) - current_speed;  // ����Ŀ���ٶ��뵱ǰ�ٶȵĲ�ֵ
	
	if ((speed_difference > max_speed_delta) && (*buffered_speed < target_speed))
	{
		*buffered_speed += max_speed_delta; // ���������ٶ�
		
		if(fabs(*buffered_speed) >= fabs(target_speed))
		{
			*buffered_speed = target_speed;  // ���������Χ�ڣ���ֱ��ʹ��Ŀ���ٶ�		
		}
	}
	else if ((speed_difference < -max_speed_delta) && (*buffered_speed > target_speed))
	{
		*buffered_speed -= max_speed_delta;  // ���������ٶ�	
		if(fabs(*buffered_speed) <= fabs(target_speed))
		{
			*buffered_speed = target_speed;  // ���������Χ�ڣ���ֱ��ʹ��Ŀ���ٶ�		
		}
	}
	else
	{
		*buffered_speed = target_speed;  // ���������Χ�ڣ���ֱ��ʹ��Ŀ���ٶ�		
	}
	
	
}