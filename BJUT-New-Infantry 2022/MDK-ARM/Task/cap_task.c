/**
  ******************************************************************************
  * @file    cap_task.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2021/9/1
  * @brief   电容进程
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
#include <stdio.h>
#include <stdlib.h>
uint16_t power = 5000;

void Cap_Task(void const * argument)
{ 
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		osDelayUntil(&xLastWakeTime,100);//10HZ
		power = InfantryJudge.ChassisPowerLimit * 100;//发送电容数据
		Cap_Output(power);
	}
}


