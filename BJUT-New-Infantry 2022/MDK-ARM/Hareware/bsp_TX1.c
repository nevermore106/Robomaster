/**
  ******************************************************************************
  * @file    bsp_TX1.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/5/9
  * @brief   TX1数据的接收
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
  * @brief  TX1数据接收
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
//		if(TX1_Data.ArmorLost == 1)//当目标丢失时，使云台停在当前位置，主要应用于打陀螺，实际好不好用有待测试
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
//		if(Command.AimRotateAssitFlag == 1)//按下E键可以关闭自瞄的预测
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
//		KF.Kalman_Filter_Speed = 0 ;//装甲片为固定靶，所以无速度
//	}
//}


//  0x00 红色 0xff 蓝色
//把这个发送任务放在检测函数里了，因为检测函数的频率比较低
void TX1_DataTransmit(void)
{
	TX1_Transmit_Data[0] = 0xA5;
	TX1_Transmit_Data[1] = TX1_Data.mode;
	TX1_Transmit_Data[2] = TX1_Data.hitmode;
	Append_CRC8_Check_Sum(TX1_Transmit_Data,4);
	HAL_UART_Transmit_DMA(&huart3,TX1_Transmit_Data,4);
}

//当遥控器右侧拨杆处于中位的时候
//调整左侧拨杆可以修改攻击颜色
void TX1_Mode_Switch(void)
{
    if(InfantryJudge.RobotID < 100)
        TX1_Data.mode = ATTACK_BLUE_ARMOR;
    else if(InfantryJudge.RobotID > 100)
        TX1_Data.mode = ATTACK_RED_ARMOR;
}

