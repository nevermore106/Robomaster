/**
  ******************************************************************************
  * @file    bsp_TX1.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/5/9
  * @brief   TX1���յ�ͷ�ļ�
  ******************************************************************************
  * @attention
	* 
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "universal.h"

//����λ��ͨ�Ŷ���
#define ATTACK_RED_ARMOR        0x00
#define ATTACK_BLUE_ARMOR       0xff
#define WINDMILL_CLOCKWISE      0x77
#define WINDMILL_ANTI_CLOCKWISE 0x66
#define ATTACK_WINDMILL         0x44


typedef struct
{
	u8 mode;//��λ������ģʽ������λ�����
	u8 hitmode;
	s16 Yaw_Angle;//Yaw��Ƕȣ�Ϊ�������˲�ǰ�Ƕ�
	s16 Pitch_Angle;//Pitch��Ƕȣ�ͬ��
	u16 Distance;//Ŀ�����ֵ
	s16 Speed;//Ŀ���ƶ��ٶȣ�ͬΪ�˲�ǰ����
	s16 Center_Angle;//����ƫ��
	s16 Yaw_Offset;//Yaw���ĵ�����ƫ��
	u16 ArmorLost;
	s16 Wrongdata;
}TX1_Data_t;



void TX1_DataReceive(void);
void TX1_DataTransmit(void);
void TX1_Mode_Switch(void);

extern u8 TX1_Receive_Data[30];
extern u8 TX1_Transmit_Data[22];
extern TX1_Data_t TX1_Data;
extern s16 pitcom;

