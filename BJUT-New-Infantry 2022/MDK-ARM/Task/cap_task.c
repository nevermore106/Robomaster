/**
  ******************************************************************************
  * @file    cap_task.c
  * @author  ������ҵ��ѧ-������
  * @version V1.0
  * @date    2021/9/1
  * @brief   ���ݽ���
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
		power = InfantryJudge.ChassisPowerLimit * 100;//���͵�������
		Cap_Output(power);
	}
}


