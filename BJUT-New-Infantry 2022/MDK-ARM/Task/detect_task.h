/**
  ******************************************************************************
  * @file    detect_task.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ��������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/




void Detect_Task(void const * argument);
void Buzzer_on(void);
void Buzzer_off(void);
void Facility_Frequency(void);
u8 Imuex_Detect(void);
u8 Remote_Detect(void);
u8 Remote_Value_Detect(u16 Channel);
extern u16 Frequency[10];
extern u16 RefereeCount;
