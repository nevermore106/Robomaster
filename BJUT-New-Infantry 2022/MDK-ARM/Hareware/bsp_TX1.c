/**
  ******************************************************************************
  * @file    bsp_TX1.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/5/9
  * @brief   TX1���ݵĽ���
  ******************************************************************************
  * @attention

  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

u8 TX1_Receive_Data[30]={0};
u8 TX1_Transmit_Data[22]={0};
TX1_Data_t TX1_Data={0};
s16 pitcom = 0;
s16 yawcom = 0;

/**
  * @brief  TX1���ݽ���
  * @param  None
  * @retval None
  */
void TX1_DataReceive(void)
{
	Laser_ON(); 	
	Frequency[8]++;
//	if(TX1_Receive_Data[0] == 0x5A && Verify_CRC16_Check_Sum(TX1_Receive_Data,15))
//	{	
//		TX1_Data.ArmorLost     = 0;
//		TX1_Data.Yaw_Angle     =  TX1_Receive_Data[1] << 8 | TX1_Receive_Data[2] + yawcom;
//		TX1_Data.Pitch_Angle   =  TX1_Receive_Data[3] << 8 | TX1_Receive_Data[4] + pitcom;
//		TX1_Data.Distance      =  TX1_Receive_Data[5] << 8 | TX1_Receive_Data[6];
//		TX1_Data.Speed         =  TX1_Receive_Data[7] << 8 | TX1_Receive_Data[8];
//		TX1_Data.Center_Angle  =  TX1_Receive_Data[9] << 8 | TX1_Receive_Data[10];
//		TX1_Data.Yaw_Offset    =  TX1_Receive_Data[11]<< 8 | TX1_Receive_Data[12];
//		KF.Kalman_Filter_Yaw   =  KF.Yaw_angle[0]   - TX1_Data.Yaw_Angle;// + Yaw_Predict(&TX1_Data);;
//		KF.Kalman_Filter_Pitch =  KF.Pitch_angle[0] + TX1_Data.Pitch_Angle;
//		KF.Kalman_Filter_Speed =  TX1_Data.Speed - TX1_Data.Distance * imu.wz * NORMAL_PI / 180;
//		KF.Kalman_Filter_CenterAngle = KF.Yaw_angle[0]   - TX1_Data.Center_Angle;
//	}
//	else if(TX1_Receive_Data[0] == 0xAA && Verify_CRC16_Check_Sum(TX1_Receive_Data,13))
//	{	
//		TX1_Data.ArmorLost++;
//		if(TX1_Data.ArmorLost == 1)//��Ŀ�궪ʧʱ��ʹ��̨ͣ�ڵ�ǰλ�ã���ҪӦ���ڴ����ݣ�ʵ�ʺò������д�����
//		{
//			TX1_Data.Yaw_Angle     =  0;
//			TX1_Data.Pitch_Angle   =  0;
//			TX1_Data.Distance      =  TX1_Receive_Data[5] << 8 | TX1_Receive_Data[6];
//			TX1_Data. Center_Angle =  0;
//			TX1_Data.Speed         =  0;
//			KF.Kalman_Filter_Yaw   = KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1];
//			KF.Kalman_Filter_Pitch = KF.Pitch_angle[KALMAN_FILTER_PAST_TIME - 1];
//			KF.Kalman_Filter_CenterAngle = KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1];
//		}
//		KF.Kalman_Filter_Speed = 0;
//	}
//	else if(TX1_Receive_Data[0] != 0x5A && TX1_Receive_Data[0] != 0xAA)
//    {
//        HAL_UART_DMAStop(&huart3);
//        osDelay(1);
//        HAL_UART_Receive_DMA(&huart3,TX1_Receive_Data,15);
//    }
    if(TX1_Receive_Data[0] == 0x5A)
			;
}

//void TX1_DataReceive(void)
//{
//	Laser_ON();	
//	if(TX1_Receive_Data[0] == 0x5A && Verify_CRC16_Check_Sum(TX1_Receive_Data,11))
//	{	
//		TX1_Data.Yaw_Angle     = TX1_Receive_Data[1] << 8 | TX1_Receive_Data[2];
//		TX1_Data.Pitch_Angle   = TX1_Receive_Data[3] << 8 | TX1_Receive_Data[4];
//		TX1_Data.Distance      = TX1_Receive_Data[5] << 8 | TX1_Receive_Data[6];
//		TX1_Data.Speed         = TX1_Receive_Data[7] << 8 | TX1_Receive_Data[8];
//		KF.Kalman_Filter_Yaw   = KF.Yaw_angle[0]   - TX1_Data.Yaw_Angle;// + Yaw_Predict(&TX1_Data);;
//		KF.Kalman_Filter_Pitch = KF.Pitch_angle[0] + TX1_Data.Pitch_Angle;
//		if(Command.AimRotateAssitFlag == 1)//����E�����Թر������Ԥ��
//			KF.Kalman_Filter_Speed = 0;
//		else
//			KF.Kalman_Filter_Speed = TX1_Data.Speed - TX1_Data.Distance * Imuex.Speed.Yaw * NORMAL_PI / 180;
//	}
//	else if(TX1_Receive_Data[0] == 0xA5 && Verify_CRC16_Check_Sum(TX1_Receive_Data,9))
//	{	
//		TX1_Data.Yaw_Angle     = TX1_Receive_Data[1] << 8 | TX1_Receive_Data[2];
//		TX1_Data.Pitch_Angle   = TX1_Receive_Data[3] << 8 | TX1_Receive_Data[4];
//		TX1_Data.Distance      = TX1_Receive_Data[5] << 8 | TX1_Receive_Data[6];
//		KF.Kalman_Filter_Yaw   = KF.Yaw_angle[0]   - TX1_Data.Yaw_Angle;// + Yaw_Predict(&TX1_Data);;
//		KF.Kalman_Filter_Pitch = KF.Pitch_angle[0] + TX1_Data.Pitch_Angle;
//		KF.Kalman_Filter_Speed = 0 ;//װ��ƬΪ�̶��У��������ٶ�
//	}
//}


//  0x00 ��ɫ 0xff ��ɫ
//���������������ڼ�⺯�����ˣ���Ϊ��⺯����Ƶ�ʱȽϵ�
void TX1_DataTransmit(void)
{
	TX1_Transmit_Data[0] = 0xA5;
	TX1_Transmit_Data[1] = TX1_Data.mode;
	TX1_Transmit_Data[2] = TX1_Data.hitmode;
	Append_CRC8_Check_Sum(TX1_Transmit_Data,4);
	HAL_UART_Transmit_DMA(&huart3,TX1_Transmit_Data,4);
}

//��ң�����Ҳದ�˴�����λ��ʱ��
//������ದ�˿����޸Ĺ�����ɫ
void TX1_Mode_Switch(void)
{
    if(InfantryJudge.RobotID < 100)
        TX1_Data.mode = ATTACK_BLUE_ARMOR;
    else if(InfantryJudge.RobotID > 100)
        TX1_Data.mode = ATTACK_RED_ARMOR;
}

