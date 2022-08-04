/**
  ******************************************************************************
  * @file    bsp_imuex.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2021/11/24
  * @brief   外接陀螺仪
  ******************************************************************************
  * @attention

  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

u8 Imuex_Receive_Data[40]={0};
s32 timu[4] = {0};
Imuex_t Imuwt={0};
Imuex_t Imuex={0};

void Imuex_Receive(void)//维特接收协议
{
	timu[0]++;
	if(Imuex_Receive_Data[0] == 0x55 )
	{
		timu[1]++;
		switch(Imuex_Receive_Data[1])
		{
			case 0x52:
			{
   		 Imuwt.Speed.Roll  =   (short)(Imuex_Receive_Data[2] | Imuex_Receive_Data[3] << 8);
   		 Imuwt.Speed.Pitch =   (short)(Imuex_Receive_Data[4] | Imuex_Receive_Data[5] << 8);
  		 Imuwt.Speed.Yaw   =   (short)(Imuex_Receive_Data[6] | Imuex_Receive_Data[7] << 8);
			 Imuwt.Speed.Roll  =   Imuwt.Speed.Roll * 0.0610351f;//2000/32768
			 Imuwt.Speed.Pitch =   Imuwt.Speed.Pitch * 0.0610351f;
			 Imuwt.Speed.Yaw   =   Imuwt.Speed.Yaw * 0.0610351f;
			}
			break;
			case 0x53:
			{
				timu[2]++;
  		 Imuwt.Angle.Roll  =   (short)(Imuex_Receive_Data[2] | Imuex_Receive_Data[3] << 8);
  		 Imuwt.Angle.Pitch =   (short)(Imuex_Receive_Data[4] | Imuex_Receive_Data[5] << 8);
  	   Imuwt.Angle.Yaw   =   (short)(Imuex_Receive_Data[6] | Imuex_Receive_Data[7] << 8);
			 Imuwt.Angle.Roll  =   Imuwt.Angle.Roll  * 0.0054931f;//180/32768
			 Imuwt.Angle.Pitch =   Imuwt.Angle.Pitch * 0.0054931f;
			 Imuwt.Angle.Yaw   =   Imuwt.Angle.Yaw * 0.0054931f;
			}
			break;
		  default:
			break;
		}		
	}	
}

void imuex_attitude_update(void)
{
	Imuex.Angle.Yaw  = (Imuwt.Angle.Yaw*8192/360 + 4096); 

	Imuex.Angle.Pitch = (-Imuwt.Angle.Pitch*8192/360 + 4096);   

	Imuex.Angle.Roll = (Imuwt.Angle.Roll*8192/360 + 4096);
	
	Imuex.Speed.Yaw = Imuwt.Speed.Yaw;
	
	Imuex.Speed.Pitch = -Imuwt.Speed.Pitch;
	
	Imuex.Speed.Roll = Imuwt.Speed.Roll;
}

//void Imuex_Receive()//超核电子接收协议
//{
//	if(Imuex_Receive_Data[0] == 0x5A &&Imuex_Receive_Data[1] == 0xA5)
//	{
//		Frequency[8]++;
//		Imuex.Speed.Roll   =   Imuex_Receive_Data[7]  | Imuex_Receive_Data[8]  << 8;
//		Imuex.Speed.Pitch  =   Imuex_Receive_Data[9]  | Imuex_Receive_Data[10] << 8;
//		Imuex.Speed.Yaw    =   Imuex_Receive_Data[11] | Imuex_Receive_Data[12] << 8;
//		Imuex.Angle.Pitch  =   Imuex_Receive_Data[14] | Imuex_Receive_Data[15] << 8;
//		Imuex.Angle.Roll   =   Imuex_Receive_Data[16] | Imuex_Receive_Data[17] << 8;
//		Imuex.Angle.Yaw    =   Imuex_Receive_Data[18] | Imuex_Receive_Data[19] << 8;
//		Imuex.Speed.Roll  /=   10;
//		Imuex.Speed.Pitch /=   10;
//	  Imuex.Speed.Yaw   /=   10;
//		Imuex.Angle.Pitch = Imuex.Angle.Pitch * 0.2275f + 4096;
//		Imuex.Angle.Yaw   = Imuex.Angle.Yaw * 2.275f + 4096;
//	}
//}

