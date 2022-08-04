/**
  ******************************************************************************
  * @file    motor.c
  * @author  Tinker.Jia
  * @version V1.0
  * @date    2018/11/7
  * @brief   M3508与轮毂电机的操作函数
  ******************************************************************************
  * @attention
	* 
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

static CAN_TxHeaderTypeDef		TxMessage;
static CAN_RxHeaderTypeDef 		RxMessage;
float PowerData[4] = {0};
/***********************************************************************************/
/***********************************************************************************/
/******************************有关M3508的处理函数**********************************/
/***********************************************************************************/
/***********************************************************************************/
/**
  * @brief  M3508驱动函数
  * @param  输入为四个电机的电流值
  * @retval None
  */
void M3508_Output(s16 cm1_iq,s16 cm2_iq,s16 cm3_iq,s16 cm4_iq)
{
	u8 Data[8]={0};
	TxMessage.StdId=0x200;//CAN通信协议标准帧ID        
  TxMessage.DLC=8;  //数据值长度
	Data[0]=(uint8_t)(cm1_iq >> 8);//8位数据位，具体协议看M3508通信协议
	Data[1]=(uint8_t)cm1_iq;
	Data[2]=(uint8_t)(cm2_iq >> 8);
	Data[3]=(uint8_t)cm2_iq;
	Data[4]=(uint8_t)(cm3_iq >> 8);
	Data[5]=(uint8_t)cm3_iq;
	Data[6]=(uint8_t)(cm4_iq >> 8);
	Data[7]=(uint8_t)cm4_iq;
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0);//发送
}


/**
  * @brief  M3508反馈数据接收
  * @param  None
  * @retval None
  */
void Motor_Data_Receive(void)
{
	u8 Data_Buf[8]={0};
	
  HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMessage,Data_Buf);//读取缓冲区数据
	switch(RxMessage.StdId)//通过帧的ID判断来自哪个那个电机
	{
		case 0x201:
		{
			Frequency[1]++;//设备帧率检测
			Motor_Data_Deal(&M3508[0],Data_Buf);//电机反馈数据处理
			break;	
		}
		case 0x202:
		{
			Frequency[2]++;
			Motor_Data_Deal(&M3508[1],Data_Buf);
			break;
		}
		case 0x203:
		{
			Frequency[3]++;
			Motor_Data_Deal(&M3508[2],Data_Buf);
			break;
		}
		case 0x204:
		{
			Frequency[4]++;
			Motor_Data_Deal(&M3508[3],Data_Buf);
		break;
		}
		case 0x205:
		{
			Frequency[5]++;
			Motor_Data_Deal(&GM6020[YAW],Data_Buf);
			break;	
		}
		case 0x206:
		{
			Frequency[6]++;
			Motor_Data_Deal(&GM6020[PITCH],Data_Buf);
			break;
		}
		case 0x207:
		{
			Frequency[7]++;
			Motor_Data_Deal(&GM2006,Data_Buf);
			GM2006_Position_Deal(&GM2006);//拨弹电机是相对编码器，涉及到多圈旋转，所以需要对其进行相对编码器到绝对编码器的一个转换
			break;
		}
		case 0x211:
		{
			//Frequency[8]++;
			//extern float PowerData[4];
			uint16_t * pPowerdata = (uint16_t *)Data_Buf;
			PowerData[0] = (float)pPowerdata[0]/100.f;//输入电压
			PowerData[1] = (float)pPowerdata[1]/100.f;//电容电压
			PowerData[2] = (float)pPowerdata[2]/100.f;//输入电流
			PowerData[3] = (float)pPowerdata[3]/100.f;//设定功率
			break;
		}
		default:
			break;
	}
}

/**
  * @brief  对接收到的数据进行重组还原
  * @param  数据存储位置与数据输入
  * @retval None
  */
void Motor_Data_Deal(Motor_t *Receive,u8 Data[])
{
		Receive->Position.Real    =  (Data[0]<<8)|Data[1];//实时位置值，M3508为相对编码器，GM6020为绝对编码器
		Receive->Speed.Real       =  (Data[2]<<8)|Data[3];//实时速度值
		Receive->TCurrent         =  (Data[4]<<8)|Data[5];//实时电磁转矩，这个东西不是电机实时输出的扭矩，具体是什么我不太清楚，据说和电流值成比例关系
		Receive->Temperature      =   Data[6];//电机的实时温度值
}


/***********************************************************************************/
/***********************************************************************************/
/**************************有关GM6020和2006的处理函数*******************************/
/***********************************************************************************/
/***********************************************************************************/
/**
  * @brief  云台电机驱动函数
  * @param  输入为四个电机的电流值
  * @retval None
  */
void Gimbal_Output(s16 gm1_iq,s16 gm2_iq,s16 gm3_iq)
{
	u8 Data[8]={0};
	TxMessage.StdId=0x1FF;//云台电机和拨弹电机的标准帧ID        
  TxMessage.DLC=8;//数据长度  
	Data[0]=(uint8_t)(gm1_iq >> 8);//Yaw
	Data[1]=(uint8_t)gm1_iq;
	Data[2]=(uint8_t)(gm2_iq >> 8);//Pitch
	Data[3]=(uint8_t)gm2_iq;
	Data[4]=(uint8_t)(gm3_iq >> 8);//拨轮
	Data[5]=(uint8_t)gm3_iq;
	Data[6]=0;
	Data[7]=0;
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0);//发送
}



/**
  * @brief  2006编码器处理
  * @param  None
  * @retval None
  */
void GM2006_Position_Deal(Motor_t *Receive)
{
	if(Receive->Position.Flag==0)//记录初试位置
	{
		Receive->Position.Flag = 1;
		Receive->Position.InitConvert=Receive->Position.Real/64;//对编码器进行缩小，缩小倍数为64
		Receive->Position.LastReal   =  Receive->Position.Real;
	}

	if(Receive->Position.Real - Receive->Position.LastReal>4096)//前后两次数据差值超过4096，就认为其转了一圈
		Receive->Position.Count--;
	else if(Receive->Position.LastReal - Receive->Position.Real>4096)
		Receive->Position.Count++;
	if(Receive->Position.Count<0)//减速比为1：36，当转了36圈之后，就认为其绝对编码器转了一圈
		Receive->Position.Count=35;
	else if(Receive->Position.Count>35)
		Receive->Position.Count=0;
	Receive->Position.Convert=(Receive->Position.Count*128)%4608+(Receive->Position.Real/64)-Receive->Position.InitConvert;//相对编码器转换为绝对编码器
	Receive->Position.Convert=((Receive->Position.Convert+4608)%4608)*1.77777778f;//将编码器的值转换到0-8191
	Receive->Position.LastReal   =  Receive->Position.Real;//数据迭代
	
}


/**
  * @brief  超级电容驱动函数
  * @param  输入为超级电容设定功率
  * @retval None
  */
void Cap_Output(uint16_t temPower)
{
	u8 Data[8]={0};
	TxMessage.StdId=0x210;//超级电容的标准帧ID        
  	TxMessage.DLC=8;//数据长度  
	Data[0]=(uint8_t)(temPower >> 8);//Yaw
	Data[1]=(uint8_t)temPower;
	HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Data,(uint32_t*)CAN_TX_MAILBOX0);//发送
}

