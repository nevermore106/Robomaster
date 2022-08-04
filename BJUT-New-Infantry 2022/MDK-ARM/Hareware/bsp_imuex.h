/**
  ******************************************************************************
  * @file    bsp_imuex.h
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2021/11/24
  * @brief   外接陀螺仪头文件
  ******************************************************************************
  * @attention

  ******************************************************************************
  */

typedef struct
{
	struct
	{
		float Pitch;
		float Roll;
		float Yaw;
	}Angle;
	struct
	{
		float Pitch;
		float Roll;
		float Yaw;
	}Speed;
}Imuex_t;

void Imuex_Receive(void);
void imuex_attitude_update(void);
extern u8 Imuex_Receive_Data[40];
extern Imuex_t Imuex;
extern Imuex_t Imuwt;
