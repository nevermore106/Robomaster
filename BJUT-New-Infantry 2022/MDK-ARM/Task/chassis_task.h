/**
  ******************************************************************************
  * @file    chassis_task.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ���̽���ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "universal.h"


void Chassis_Task(void const * argument);
s8 Chassis_Power_On(void);
s16 Twist_Control(void);
void M3508Ramp_Self_Deal(void);
void Imu_Temperature_Compensate(void);

typedef struct//������ݽṹ��
{
	struct
	{
		s16 Set;//�ٶ��趨ֵ
		s16 Real;//�ٶ���ʵֵ
		s16 Step;//�ٶ�б���趨ֵ
	}Speed;
	struct
	{
		s16 Set;//λ���趨ֵ
		s16 Real;//λ����ʵֵ
		s16 Step;//λ��б���趨ֵ
		s16 LastReal;//�ϴε���ʵֵ
		s16 Convert;//���Ա��������ת��ֵ
		s16 InitConvert;//�ϵ����λ��ת��ֵ
		s8  Count;//��Ա�����Ȧ������
		s8  Flag;//��־λ
	}Position;
	s16 TCurrent;//���ת�أ�������ת��ֵ�����嵥λ��δ֪
	s16 Temperature;//ʵ���¶�ֵ
	s16 Rotate;//ת������
	s16 Output;
}Motor_t;

//���̵����б��ֵ
typedef struct
{
	s16   Mis[4];//�ĸ����ӵ�ʵ��ƫ��
	float Increment[5];//�������
}M3508_Ramp_t;

typedef struct
{
	struct
	{
		s16 Set;//�趨ֵ
		s16 Real;//��ʵֵ
		s16 LastReal;//�ϴε���ʵֵ
	}Vx;
	struct
	{
		s16 Set;//�趨ֵ
		s16 Real;//��ʵֵ
		s16 LastReal;//�ϴε���ʵֵ
	}Vy;
	struct
	{
		float Set;//�趨ֵ
		float Real;//��ʵֵ
		float Transmit;//
		s16 TractionCompensate;//ǣ��������
	}Angle;
	s16 Speed;
}SpinTop_t;

extern int16_t Yaw_Position_Move;
extern M3508_Ramp_t M3508_Ramp;
extern Motor_t M3508[4];
extern Motor_t Chassis;
extern SpinTop_t SpinTop;

