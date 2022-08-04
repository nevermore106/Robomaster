/**
  ******************************************************************************
  * @file    bsp_friciton.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2018/11/7
  * @brief   摩擦轮操作函数
  ******************************************************************************
  * @attention
	* 
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
/**
  * @brief  摩擦轮与激光启动，并对摩擦轮的输入值进行矫正
  * @param  摩擦轮启动参数
  * @retval None
  */
float pause = 1000;
void Friction_ON(s16 pwm)
{
//	Laser_ON();	
	pwm = Func_Limit(pwm,1800,1000); 
	Func_FRamp(pwm,&pause,0.5);
	TIM5->CCR1 = (s16)pause;
	TIM5->CCR2 = (s16)pause;
}
//void Friction1_ON(s16 pwm)
//{
//	Laser_ON();	
//	pwm = Func_Limit(pwm,1800,1000); 
//	Func_FRamp(pwm,&pause,0.5);
//	TIM5->CCR1 = (s16)pause;
//}
//void Friction2_ON(s16 pwm)
//{
//	Laser_ON();	
//	pwm = Func_Limit(pwm,1800,1000); 
//	Func_FRamp(pwm,&pause,0.5);
//	TIM5->CCR2 = (s16)pause;
//}
//void Friction_ON(s16 pwm)
//{ 
//	Friction1_ON(pwm);
//	tb++;
//	if(tb>1500)
//	 {
//	  Friction2_ON(pwm);
//		if(tb>8000);
//	   {
//		  tb=1501;
//	   }
//	 }

//}

/**
  * @brief  舵机启动
  * @param  舵机启动参数
  * @retval None
  */
float pause1 = 500;
void Steering_Engine_ON(s16 pwm)
{
	pwm = Func_Limit(pwm,1996,500); 
	TIM5->CCR3 = (s16)pause1;
	
}

/**
  * @brief  激光与摩擦轮关闭
  * @param  None
  * @retval None
  */
void Friction_OFF(void)
{ 

//	Laser_OFF();
	pause = 1000;
	TIM5->CCR1 = 1000;
	TIM5->CCR2 = 1000;
}
