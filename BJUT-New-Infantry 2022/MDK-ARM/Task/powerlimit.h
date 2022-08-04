/**
  ******************************************************************************
  * @file    powerlimit.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/6
  * @brief   ��������ͷ����
  ******************************************************************************
  * @attention

  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

typedef struct
{
  float Real_Power[3]; //2���� 1�ϴ� 0���ϴ�
	float RemainPower[3];
	s16 MaxSpeed;  //����ٶ�ֵ
	u8 Cut;        //�Ƿ��жϵ������
	float Zoom;    //���ű��ʷ�ֹ�������
	float ZoomTemp;//�����ݴ�ֵ
	u8 ZoomCount;//����Forѭ������
	u8 Flag;//��־λ����������Ƿ��˳�������
	u8 Restore;
} PowerLimit_t;



void CMControlLoop(void);
void Direction_Keep(void);
void Power_Deal(void);
void Power_Control(void);
void Motor_Increment_Self_Deal(void);
void Power_PID_Calc(PidTypeDef * pid, float real_val, float set_val);
extern PowerLimit_t PowerLimit;



