/**
  ******************************************************************************
  * @file    friciton.h
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
#define Laser_ON()  HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET)
#define Laser_OFF() HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_RESET)

void Friction_OFF(void);
void Friction_ON(s16 pwm);

void Steering_Engine_ON(s16 pwm);

