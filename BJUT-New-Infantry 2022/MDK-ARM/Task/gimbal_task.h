/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  ������ҵ��ѧ-������
  * @version V1.0
  * @date    2022/2/7
  * @brief   ��̨����ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/


#include "universal.h"
#define NORMAL_PI 3.14159265f
typedef struct
{
	u8 Last_s1;//��һ�β���ֵ�ô���
	u8 Friction;//Ħ���ֿ���״̬��־��1Ϊ������0Ϊ�ر�
	u8 PosSetCount;//�������𵥷�������
	s8 PosRatio;//б��б��
	u8 Status;//����״̬
	u8 Allow;//�Ƿ�������
	u16 Period;//��������
	s16 RemainHeating;//ʣ������
	u16 Speed;
	u16 PressTime;
}Shooting_t;


typedef struct
{
	float Error;
	s16 Remote;	
}Stable_t; 


typedef struct
{
	struct
	{
		float Real;
		float Step;
		float Error;
	}Position;
	struct
	{
		float Real;
		float Step;
		float Error;
	}Speed;
	struct
	{
		float Real;
		float Step;
		float Error;
	}Imu;
}Filter_t;

typedef struct
{
	u16 Max;
	u16 Min;
	u16 Dynamic_Max;
	u16 Dynamic_Min;
}AutoLimit_t; 

void Gimbal_Task(void const * argument);
void Shooting_Control(void);
void Gimbal_Dynamic_Limit(s16 *input,u16 encoder,u16 imu,AutoLimit_t *autolimit);
void YAW_PID_WithIMU(void);
void YAW_PID_WithoutIMU(void);
void PITCH_PID_Control(void);
void PITCH_ADRC_Control(void);
void Feed_PID_Control(void);
s16 Gravity_Compensate(void);
void Pitch_Stable(void);
s16 Bullet_Amend(float Angle,int bullet_Speed,u16 distance);
s16 Yaw_Predict(TX1_Data_t *data,float angle,int bullet_Speed);
//s8 Rotate_Allow_Shoot(Shooting_t *shoot,TX1_Data_t *data,Command_t *command);
void Vision_Control_Gimbal(void);
void Gimbal_Disable(void);
void Gimbal_FirstOrder_KalmanFilter(void);
extern Motor_t GM6020[2];
extern Motor_t GM2006;
extern Shooting_t Shooting;
extern Stable_t Stable;
extern Filter_t Kalman[2];

