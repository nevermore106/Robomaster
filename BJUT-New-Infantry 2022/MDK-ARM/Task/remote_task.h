/**
  ******************************************************************************
  * @file    remote_task.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ң�ؽ���ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "universal.h"
#define MANUAL_ATTACK        0
#define AUTO_ATTACK          1

#define ATTACK_NORMAL_ARMOR  0
#define ARMOR_ROTATE         1
#define ARMOR_NONE_PREDICT   2


typedef struct              
{                                                                    //        ��vx
	s16 Vx;//x���ٶ�                                                      1 %++++++++% 2
	s16 Vy;//y���ٶ�                                                           ++++
	s16 W0;//��ת�ٶ�                                                          ++++      ��vy
	s16 Vz;//������̨                                                          ++++
	float SpeedZoom;//shift���ٶȵĿ�����                                 3 %++++++++% 4
	float SpeedMinish;//ctrl���ٶȵĿ����� 
	u8 RotateFlag;//���ݱ�־λ
	u8 AimAssitFlag;//������׼��־λ
	u8 GimbalFollowOFF;//��̨���濪��
	u8 AimRotateAssitFlag;//������׼���ݱ�־λ
	u8 TriggerAnti;//���ֵ����ת
	float Mouse_x;
	float Mouse_y;
}Command_t;    //ģʽ����

void Remote_Task(void const * argument);
void Remote_Control(void);
void Computer_control(void);
void Automatic_Find_Enemy(void);
void Mouse_Control(void);

extern Command_t Command;

