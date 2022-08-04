/**
  ******************************************************************************
  * @file    chassis_task.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   底盘进程头文件
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

typedef struct//电机数据结构体
{
	struct
	{
		s16 Set;//速度设定值
		s16 Real;//速度真实值
		s16 Step;//速度斜坡设定值
	}Speed;
	struct
	{
		s16 Set;//位置设定值
		s16 Real;//位置真实值
		s16 Step;//位置斜坡设定值
		s16 LastReal;//上次的真实值
		s16 Convert;//绝对编码器后的转换值
		s16 InitConvert;//上电初试位置转换值
		s8  Count;//相对编码器圈数计算
		s8  Flag;//标志位
	}Position;
	s16 TCurrent;//电磁转矩，但不是转矩值，具体单位量未知
	s16 Temperature;//实际温度值
	s16 Rotate;//转动方向
	s16 Output;
}Motor_t;

//底盘电机的斜坡值
typedef struct
{
	s16   Mis[4];//四个轮子的实际偏差
	float Increment[5];//电机增量
}M3508_Ramp_t;

typedef struct
{
	struct
	{
		s16 Set;//设定值
		s16 Real;//真实值
		s16 LastReal;//上次的真实值
	}Vx;
	struct
	{
		s16 Set;//设定值
		s16 Real;//真实值
		s16 LastReal;//上次的真实值
	}Vy;
	struct
	{
		float Set;//设定值
		float Real;//真实值
		float Transmit;//
		s16 TractionCompensate;//牵引力补偿
	}Angle;
	s16 Speed;
}SpinTop_t;

extern int16_t Yaw_Position_Move;
extern M3508_Ramp_t M3508_Ramp;
extern Motor_t M3508[4];
extern Motor_t Chassis;
extern SpinTop_t SpinTop;

