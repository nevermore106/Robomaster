/**
  ******************************************************************************
  * @file    powerlimit.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/6
  * @brief   功率限制头函数
  ******************************************************************************
  * @attention

  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

typedef struct
{
  float Real_Power[3]; //2最新 1上次 0上上次
	float RemainPower[3];
	s16 MaxSpeed;  //最大速度值
	u8 Cut;        //是否切断底盘输出
	float Zoom;    //缩放比率防止方向错误
	float ZoomTemp;//缩放暂存值
	u8 ZoomCount;//缩放For循环计数
	u8 Flag;//标志位，用来检测是否退出限速区
	u8 Restore;
} PowerLimit_t;



void CMControlLoop(void);
void Direction_Keep(void);
void Power_Deal(void);
void Power_Control(void);
void Motor_Increment_Self_Deal(void);
void Power_PID_Calc(PidTypeDef * pid, float real_val, float set_val);
extern PowerLimit_t PowerLimit;



