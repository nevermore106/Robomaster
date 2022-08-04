/**
  ******************************************************************************
  * @file    remote_task.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   遥控进程头文件
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
{                                                                    //        ↑vx
	s16 Vx;//x向速度                                                      1 %++++++++% 2
	s16 Vy;//y向速度                                                           ++++
	s16 W0;//旋转速度                                                          ++++      →vy
	s16 Vz;//俯仰云台                                                          ++++
	float SpeedZoom;//shift对速度的控制量                                 3 %++++++++% 4
	float SpeedMinish;//ctrl对速度的控制量 
	u8 RotateFlag;//陀螺标志位
	u8 AimAssitFlag;//辅助瞄准标志位
	u8 GimbalFollowOFF;//云台跟随开关
	u8 AimRotateAssitFlag;//辅助瞄准陀螺标志位
	u8 TriggerAnti;//拨轮电机反转
	float Mouse_x;
	float Mouse_y;
}Command_t;    //模式处理

void Remote_Task(void const * argument);
void Remote_Control(void);
void Computer_control(void);
void Automatic_Find_Enemy(void);
void Mouse_Control(void);

extern Command_t Command;

