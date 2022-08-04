/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @author  北京工业大学-贾桐
  * @version V1.1
  * @date    2018/11/8
  * @brief   串口的初始化
  ******************************************************************************
  * @attention

  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
RC_Ctl_t RC_Ctl={0};
Key_t Key={0};
int uartcount =0;

/**
  * @brief  串口的初始化任务
  * @param  USART1 ： 只用了RX，采用DMA中断接收遥控器数据
  * @param  USART3 ： 采用串口空闲中断+DMA接收TX1数据
  * @param  USART6 ： 采用变长度DMA中断接收裁判系统数据
  * @param  UART8  ： 采用串口空闲中断+DMA接收外置陀螺仪数据
  */
void UART_Start(void)
{
	//HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	//HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
	//__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	//__HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,Sbus_Receive_Data,18);
	HAL_UART_Receive_DMA(&huart3,TX1_Receive_Data,15);
	HAL_UART_Receive_DMA(&huart8,Imuex_Receive_Data,11);
	HAL_UART_Receive_DMA(&huart6,JudgeDataBuffer,7);
}

/**
  * @brief  DMA接收完成回调函数
  * @param  huart
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance == USART1)
	{
		Remote_DateReceive();
	}
	if(huart->Instance == USART3)
	{
		TX1_DataReceive();
	}
	if(huart->Instance == UART8)
	{
	  Imuex_Receive();
	}
	if(huart->Instance == USART6)
	{
		uartcount++;
		Referee_Receive();
	}

}

/**
  * @brief  串口空闲中断函数
  * @param  huart
  * @retval None
  */
void UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
/*		HAL_UART_DMAStop(huart);
		if(huart->Instance == USART3)
		{
			if(hdma_usart3_rx.Instance->NDTR == 15)
				TX1_DataReceive();
			HAL_UART_Receive_DMA(huart,TX1_Receive_Data,30);
		}
		
		if(huart->Instance == UART8)
		{
			if(hdma_uart8_rx.Instance->NDTR == 20)
				Imuex_Receive();
			HAL_UART_Receive_DMA(huart,Imuex_Receive_Data,40);
		}
*/
  }
}

/**
	* @brief  发送完成回调函数，需要开启串口中断才能回调
  * @param  huart
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart->Instance == UART8)
//	{
//		huart->gState=HAL_UART_STATE_READY; 
//	}
	if(huart->Instance == USART6)
	{
		huart->gState=HAL_UART_STATE_READY; 
	}
//	if(huart->Instance == USART3)
//	{
//		huart->gState=HAL_UART_STATE_READY; 
//	}
}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART6)
//	{
//		HAL_NVIC_SystemReset(); 
//	}
//	if(huart->Instance == USART3)
//	{
//		HAL_NVIC_SystemReset(); 
//	}
//}



u8 Sbus_Receive_Data[18]={0};
/**
  * @brief  遥控器数据接收
  * @param  None
  * @retval None
  */
void Remote_DateReceive(void)
{
	Frequency[0]++;
	RC_Ctl.rc.ch0        = ( Sbus_Receive_Data[0]       | (Sbus_Receive_Data[1]  << 8)) & 0x07ff; // Channel 0
	RC_Ctl.rc.ch1        = ((Sbus_Receive_Data[1] >> 3) | (Sbus_Receive_Data[2]  << 5)) & 0x07ff; // Channel 1
	RC_Ctl.rc.ch2        = ((Sbus_Receive_Data[2] >> 6) | (Sbus_Receive_Data[3]  << 2)  | (Sbus_Receive_Data[4] << 10)) & 0x07ff;// Channel 2	
	RC_Ctl.rc.ch3        = ((Sbus_Receive_Data[4] >> 1) | (Sbus_Receive_Data[5]  << 7)) & 0x07ff; // Channel 3
	RC_Ctl.rc.s1         = ((Sbus_Receive_Data[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.s2         = ((Sbus_Receive_Data[5] >> 4)& 0x0003); //!< Switch right
	RC_Ctl.mouse.x       =   Sbus_Receive_Data[6]       | (Sbus_Receive_Data[7]  << 8); //!< Mouse X axis
	RC_Ctl.mouse.y       =   Sbus_Receive_Data[8]       | (Sbus_Receive_Data[9]  << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z       =   Sbus_Receive_Data[10]      | (Sbus_Receive_Data[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l =   Sbus_Receive_Data[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r =   Sbus_Receive_Data[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v_l       =   Sbus_Receive_Data[14]; //!< KeyBoard value
	RC_Ctl.key.v_h       =   Sbus_Receive_Data[15];
	RC_Ctl.rc.ch4        =   Sbus_Receive_Data[16]      | (Sbus_Receive_Data[17] << 8); 
	Key_Deal();
}

/**
  * @brief  遥控器连接电脑时，按键接收
  * @param  None
  * @retval None
  */
void Key_Deal(void)
{
	Key.W[0]     =  RC_Ctl.key.v_l & 0x01;
	Key.S[0]     = (RC_Ctl.key.v_l & 0x02) >> 1;
	Key.A[0]     = (RC_Ctl.key.v_l & 0x04) >> 2;
	Key.D[0]     = (RC_Ctl.key.v_l & 0x08) >> 3;
	Key.Shift[0] = (RC_Ctl.key.v_l & 0x10) >> 4;
	Key.Ctrl[0]  = (RC_Ctl.key.v_l & 0x20) >> 5;
	Key.Q[0]     = (RC_Ctl.key.v_l & 0x40) >> 6;
	Key.E[0]     = (RC_Ctl.key.v_l & 0x80) >> 7;
	
	Key.R[0]     =  RC_Ctl.key.v_h & 0x01;
	Key.F[0]     = (RC_Ctl.key.v_h & 0x02) >> 1;
	Key.G[0]     = (RC_Ctl.key.v_h & 0x04) >> 2;
	Key.Z[0]     = (RC_Ctl.key.v_h & 0x08) >> 3;
	Key.X[0]     = (RC_Ctl.key.v_h & 0x10) >> 4;
	Key.C[0]     = (RC_Ctl.key.v_h & 0x20) >> 5;
	Key.V[0]     = (RC_Ctl.key.v_h & 0x40) >> 6;
	Key.B[0]     = (RC_Ctl.key.v_h & 0x80) >> 7;
}

/**
  * @brief  按键检测
  * @param  键值、按下时间
  * @retval 无按下0，短按1，长按2
  */
u8 Key_Press(u16 key[],u16 SetTime)
{
	if(key[0]==1 && key[1]==0)
		return 2;
	else if(key[0]==1 && key[1]!=0)
	{
		(key[1])--;
		return 1;
	}
	key[1] = SetTime;
	return 0;
}


