/**
  ******************************************************************************
  * @file    pid.c
  * @author  Tinker.Jia
  * @version V1.1
  * @date    2018/11/12
  * @brief   PID计算函数
  ******************************************************************************
  * @attention
  *	V1.1 删除了PID中的过零处理，并对PID函数进行了优化
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
PidTypeDef M3508_Speed_pid[4]={0};
PidTypeDef Gimbal_Speed_pid[3]={0};
PidTypeDef Gimbal_Position_pid[3]={0};
PidTypeDef Gimbal_TCurrent_pid[3]={0};
PidTypeDef Chassis_Rotate_pid={0};
PidTypeDef Power_Limit_pid={0};
PidTypeDef Spin_Top_pid={0};
PidTypeDef Pitch_Stable_pid={0};
/**
  * @brief  PID参数的初始化
  * @param  参数太多了，就是一些基本设置
  * @retval None
  */
void PID_Init(PidTypeDef * pid,float kp,float ki,float kd,float max_out,float dead_band,float i_band,float max_input,float i_max_out,u8 mode)
{
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	pid->Max_out=max_out;
	pid->I_max_out=i_max_out;
	pid->Dead_band=dead_band;
	pid->Intergral_band=i_band;
	pid->Max_input=max_input;
	//PID输出值
	pid->Input=0;
	pid->Output=0;
	pid->Intergral=0;
	pid->Mode=mode;
	//误差初始化
	pid->E[0]=0;
	pid->E[1]=0;
	pid->E[2]=0;//2最新 1上一次 0上上次
	pid->D_last=0;
}


/**
  * @brief  PID计算
  * @param  输入、反馈
  * @retval None
  */
void PID_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float p = 0,//设定计算的PID值
				i = 0,
				d = 0;
	pid->E[2]=Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//当前误差	
	
	if(pid==&Gimbal_Position_pid[0]||pid==&Gimbal_Position_pid[1]||pid==&Gimbal_Position_pid[2])//过零处理，无论怎么转，都走劣弧，需要最大输入为码盘最大编码
	{
		if(pid->E[2]>4096)
			pid->E[2]=pid->E[2]-8192;
		else if((-pid->E[2])>4096)
			pid->E[2]=8192+pid->E[2];
	}
	if(pid == &Spin_Top_pid)
	{
		if(pid->E[2]>180)
			pid->E[2]=pid->E[2]-360;
		else if((-pid->E[2])>180)
			pid->E[2]=360+pid->E[2];
	}

	if(Func_Abs(pid->E[2]) >=  pid->Dead_band)//当偏差值大于等于死区值，进入这个if
	{
		if(pid->Mode==Positional)
		{	
			if(Func_Abs(pid->E[2]) <= pid->Intergral_band)             //当偏差小于积分范围的时候，进入积分  注意！只有位置式PID有积分范围
//				pid->Intergral =pid->Intergral+ (pid->Ki) * (pid->E[2]);//积分值
			 pid->Intergral =pid->Intergral+ ((pid->Ki) * (pid->E[2]) + (pid->Ki) * (pid->E[1]))/2;//梯形积分
			else
				{pid->Intergral=pid->Intergral;}                
			pid->Intergral=Func_Limit(pid->Intergral,pid->I_max_out,-(pid->I_max_out));//积分限幅			
  		p = pid->Kp * (pid->E[2]);
			i = pid->Intergral;
			d = pid->Kd * (pid->E[2]-pid->E[1]);
			pid->D_last=d;
			pid->Output=p+i+d;
		}
		else if(pid->Mode==Incremental)
		{
			p=pid->Kp*((pid->E[2]-pid->E[1]));
			i=pid->Ki*pid->E[2];
			d=pid->Kd*(pid->E[2]-pid->E[1]*2+pid->E[0]);
			pid->Output+=p+i+d;
		}
//YAW和PITCH加入前馈补偿
		if(pid == &Gimbal_Speed_pid[0])
			pid->Output += Yaw_Feedforward_Controler(Gimbal_Position_pid[YAW].Output);
//		if(pid == &Gimbal_Speed_pid[1])
//			pid->Output += Pitch_Feedforward_Controler(Gimbal_Position_pid[PITCH].Output);
//是否超出最大输出
		pid->Output=Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));

	}
	else
		pid->Output=0;	
	/*迭代误差*/
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

/**
  * @brief  底盘跟随PD计算
  * @param  输入、反馈
  * @retval None
  */
void Chassis_Follow_PD_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float Abs_Error = 0,
				p = 0,//设定计算的PID值
				d = 0;

	pid->E[2] = Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//当前误差	
	if(pid->E[2]>4096)
		pid->E[2]=pid->E[2]-8192;
	else if((-pid->E[2])>4096)
		pid->E[2]=8192+pid->E[2];
	Abs_Error = Func_Abs(pid->E[2]);
	if(Abs_Error >  3300)
	{
		pid->Kp = 6;
		pid->Kd = 0;
	}
	else if(Abs_Error >  2200)
	{
		pid->Kp = 5.5;
		pid->Kd = 150;
	}
	else if(Abs_Error >  1300)
	{
		pid->Kp = 5;
		pid->Kd = 200;
	}
	else if(Abs_Error >  550)
	{
		pid->Kp = 5;
		pid->Kd = 220;
	}
	else 
	{
		pid->Kp = 4.5f;			
		pid->Kd = 220;
	}			
						
	p = pid->Kp * (pid->E[2]);
	d = pid->Kd * (pid->E[2]-pid->E[1]);
	pid->Output = p+d;
	pid->Output = Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));
	
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

/**
  * @brief  PID缓存清空
  * @param  要清空的地址
  * @retval None
  */
void PID_Clear(PidTypeDef * pid)
{
	pid->E[0] = 0;
	pid->E[1] = 0;
	pid->E[2] = 0;
	pid->Intergral = 0;
	pid->Output = 0;
}


//YAW轴电机前馈补偿函数
//其实可以写在一起的，但是时间比较紧迫，就没有合并，有时间改进一下。
//通过系统辨识对云台建模，然后对其补偿
s16 Yaw_Feedforward_Controler(s16 set)
{
	s16 output;
 /* n为传递函数分子分母最高阶次 */
	const uint8_t n = 4;
//分子num 要注意前馈补偿函数的分子和分母是传递函数的倒数
	//const float b[n + 1] = {1.0000, -2.880214963923001, 2.761097200406337, -0.880881688211085};
	const float b[n + 1] = {1.0000, -3.893807793087997, 5.686900214928543, -3.692246693791884,0.899154974898897};
/* 分母den */
	//const float a[n + 1] = {-3.644641836338100e-05, 5.655192265084034e-05, 3.644641836338100e-05, -5.655192265084034e-05};
	const float a[n + 1] = {2.614389083511688e-05, -4.583097707295865e-05, -6.103729090941078e-06, 4.584276961172979e-05,-2.002836920540466e-05};
	static float x[n + 1] = {0};
	static float y[n + 1] = {0};
	
	for (uint8_t i = n; i > 0; i--)
	{
		y[i] = y[i - 1];
		x[i] = x[i - 1];
	}
	x[0] = set;
	y[0] = 0;
	
	for (uint8_t i = n; i > 0; i--)
	{
		y[0] = y[0] + b[i] * x[i];
		y[0] = y[0] - a[i] * y[i];
	}	
	output = y[0] + b[0] * x[0];
	output = Func_Limit(output,4000,-4000);
	return output;
}

//PITCH轴电机前馈补偿函数
s16 Pitch_Feedforward_Controler(s16 set)
{
	s16 output;
	float num;
	float den;
/* n为传递函数分子分母最高阶次 */
	const uint8_t n = 3;
//分子num 要注意前馈补偿函数的分子和分母是传递函数的倒数
	const float b[n + 1] = {1, 157.9, 319.3, 3.664};
/* 分母den */
	const float a[n + 1] = {0,0.1168, 0.2405, 0};
  if(set <= 3)
		return 0;
	else
	{
	num=b[3]*set*set*set + b[2]*set*set + b[1]*set +b[0];
	den=a[3]*set*set*set + a[2]*set*set + a[1]*set +a[0];
	output = num/den;
	output = Func_Limit(output,4000,-4000);
	return output;
	}
}

//原型函数
//float function(float input)
//{
///* n为传递函数分子分母最高阶次 */
//	const uint8_t n = 2;
///* 分子num */
//	const float b[n + 1] = {1.0e-03 * 0, 1.0000, -0.9990};
///* 分母den */
//	const float a[n + 1] = {1.0000, -1.9990, 0.9990};
// 
//	static float x[n + 1] = {0};
//	static float y[n + 1] = {0};
//	
//	for (uint8_t i = n; i > 0; i--)
//	{
//		y[i] = y[i - 1];
//		x[i] = x[i - 1];
//	}
//	x[0] = input;
//	y[0] = 0;
//	
//	for (uint8_t i = n; i > 0; i--)
//	{
//		y[0] = y[0] + b[i] * x[i];
//		y[0] = y[0] - a[i] * y[i];
//	}	
//	return y[0] + b[0] * x[0];
//}

