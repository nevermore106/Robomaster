/**
  ******************************************************************************
  * @file    friciton.h
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
#define Laser_ON()  HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET)
#define Laser_OFF() HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_RESET)

void Friction_OFF(void);
void Friction_ON(s16 pwm);

void Steering_Engine_ON(s16 pwm);

