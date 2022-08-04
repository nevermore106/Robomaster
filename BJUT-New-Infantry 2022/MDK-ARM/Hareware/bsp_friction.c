/**
  ******************************************************************************
  * @file    bsp_friciton.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2018/11/7
  * @brief   Ħ���ֲ�������
  ******************************************************************************
  * @attention
	* 
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
/**
  * @brief  Ħ�����뼤������������Ħ���ֵ�����ֵ���н���
  * @param  Ħ������������
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
  * @brief  �������
  * @param  �����������
  * @retval None
  */
float pause1 = 500;
void Steering_Engine_ON(s16 pwm)
{
	pwm = Func_Limit(pwm,1996,500); 
	TIM5->CCR3 = (s16)pause1;
	
}

/**
  * @brief  ������Ħ���ֹر�
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
