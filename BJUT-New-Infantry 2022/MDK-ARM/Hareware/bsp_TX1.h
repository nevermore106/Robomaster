/**
  ******************************************************************************
  * @file    bsp_TX1.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/5/9
  * @brief   TX1接收的头文件
  ******************************************************************************
  * @attention
	* 
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "universal.h"

//上下位机通信定义
#define ATTACK_RED_ARMOR        0x00
#define ATTACK_BLUE_ARMOR       0xff
#define WINDMILL_CLOCKWISE      0x77
#define WINDMILL_ANTI_CLOCKWISE 0x66
#define ATTACK_WINDMILL         0x44


typedef struct
{
	u8 mode;//上位机工作模式，由下位机给�
	u8 hitmode;
	s16 Yaw_Angle;//Yaw轴角度，为卡尔曼滤波前角度
	s16 Pitch_Angle;//Pitch轴角度，同上
	u16 Distance;//目标距离值
	s16 Speed;//目标移动速度，同为滤波前数据
	s16 Center_Angle;//中心偏角
	s16 Yaw_Offset;//Yaw中心点坐标偏差
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

