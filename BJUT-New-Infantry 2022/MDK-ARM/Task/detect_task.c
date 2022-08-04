/**
  ******************************************************************************
  * @file    detect_task.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ���ӽ���
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"


u16 Frequency[10]={1000,1000,1000,1000,1000,1000,1000,1000,1000,100};

u16 RefereeCount = 40;
void Detect_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
		osDelayUntil(&xLastWakeTime,500);//2HZ
		HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);
    HAL_GPIO_TogglePin(LED_Green_GPIO_Port,LED_Green_Pin);
		TX1_DataTransmit();
		Facility_Frequency();
  } 
}




/**
  * @brief  �豸֡�ʼ�⣬������λ������
  * @param  None
  * @retval ״ֵ̬
  */
const u16 Frequency_Standard[10]={30,480,480,480,480,480,480,400,430};
void Facility_Frequency(void)
{
	u8 count;
	u16 frequency_display[10];
	
	
	u16 referee_display = 0;
	referee_display = RefereeCount;
	RefereeCount = 0;
	
	
	for(count = 0;count < 8;count++)
	{
		frequency_display[count] = Frequency[count];
		Frequency[count] = 0;
		if(frequency_display[count] > Frequency_Standard[count])
		{
			//HAL_GPIO_WritePin(LED_A_GPIO_Port,pow(2,count+1),GPIO_PIN_SET);
		}
		else
		{
			if(count == 0)
				HAL_NVIC_SystemReset();
			//HAL_GPIO_WritePin(LED_A_GPIO_Port,pow(2,count+1),GPIO_PIN_RESET);
		}
	}
}

/**
  * @brief  ң�������
  * @param  None
  * @retval ״ֵ̬
  */
u8 Remote_Detect(void)
{
	u8 Restart = 0;
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch0);
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch1);
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch2);
	Restart += Remote_Value_Detect(RC_Ctl.rc.ch3);
	if(Restart != 0)
		return ERROR;
	else
		return NORMAL;
}

/**
  * @brief  ���������֡ͷ��� 
  * @param  None
  * @retval ״ֵ̬
  */
u8 Imuex_Detect(void)//ά��
{
	if(Imuex_Receive_Data[0] == 0x55)
		return NORMAL;
	else
		return ERROR;
}

//u8 Imuex_Detect(void)//���˵���
//{
//	if(Imuex_Receive_Data[0] == 0x5A && Imuex_Receive_Data[1] == 0xA5)
//		return NORMAL;
//	else
//		return ERROR;
//}

/**
  * @brief  ң������ֵ���
  * @param  ͨ��ֵ
  * @retval ״ֵ̬
  */
u8 Remote_Value_Detect(u16 Channel)
{
	if((Channel > 1684) || (Channel < 364))
		return ERROR;
	else
		return NORMAL;
}

void Buzzer_on(void)
{
	TIM12->CCR1 = 130;
}

void Buzzer_off(void)
{
	TIM12->CCR1 = 0;
}
