/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  北京工业大学-贾桐
  * @version V1.1
  * @date    2018/11/8
  * @brief   CAN总线滤波器的初始化
  ******************************************************************************
  * @attention
  *  这次固件库的版本比我以前用的版本要高，很多底层函数的名称和参数发生了改了，
	*使用起来比较困难。
  *  CAN1和CAN2的初始化一定要注意，CAN2是挂载在CAN1上的，CUBEMX在生成的时候一定
	*要看一下，是否把CAN1的初始化放在了CAN2的前边，否则是不能使用的。
	*  初始化的时候滤波器问题较多，HAL库固件的版本会影响初始化，初始化不成功的话
	*要冷静，控制单一变量去检查出问题的地方，查底层代码的时候一定要细心。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

/**
  * @brief  CAN总线启动
  * @param  None
  * @retval None
  */
void CAN_Start(void)
{
	HAL_CAN_Start(&hcan1);
	CAN1_Filter_Init(&hcan1);
//	HAL_CAN_Start(&hcan2);
//	CAN2_Filter_Init(&hcan2);
}



/**
  * @brief  CAN1的滤波器初始化
  * @param  CAN_HandleTypeDef
  * @retval None
  */
void CAN1_Filter_Init(CAN_HandleTypeDef* _hcan)
{
	CAN_FilterTypeDef  CAN1_FilterConfig;

	CAN1_FilterConfig.FilterIdHigh = 0x0000;
	CAN1_FilterConfig.FilterIdLow = 0x0000;
	CAN1_FilterConfig.FilterMaskIdHigh = 0x0000;
	CAN1_FilterConfig.FilterMaskIdLow = 0x0000;
	CAN1_FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN1_FilterConfig.FilterBank = 0;
	CAN1_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN1_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN1_FilterConfig.FilterActivation = ENABLE;
	CAN1_FilterConfig.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterConfig) != HAL_OK)
	{
			while(1);
	}
//这里为什么要加一个判定语句呢，这初始化差点给我搞死了。手贱更新了一下
//库，这库更新之后底层的一些封装函数全变了，我只想骂人。。。
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  CAN2的滤波器初始化
  * @param  CAN_HandleTypeDef
  * @retval None
  */
//void CAN2_Filter_Init(CAN_HandleTypeDef* _hcan)
//{
//	CAN_FilterTypeDef  CAN2_FilterConfig;
//	CAN2_FilterConfig.FilterIdHigh = 0x0000;
//	CAN2_FilterConfig.FilterIdLow = 0x0000;
//	CAN2_FilterConfig.FilterMaskIdHigh = 0x0000;
//	CAN2_FilterConfig.FilterMaskIdLow = 0x0000;
//	CAN2_FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
//	CAN2_FilterConfig.FilterBank = 14;
//	CAN2_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//	CAN2_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	CAN2_FilterConfig.FilterActivation = ENABLE;
//	CAN2_FilterConfig.SlaveStartFilterBank = 14;
//	if(HAL_CAN_ConfigFilter(&hcan2, &CAN2_FilterConfig) != HAL_OK)
//	{
//			while(1);
//	}
//这里写的还是CAN1，讲道理应该是CAN2，但是CAN2滤波器给的偏移地址下并没有
//寄存器，而且CAN1 CAN2共用滤波器，所以把偏移地址指向了CAN1，由于更新了固
//件版本，不知道这个BUG有没有更新，需要检查一下。  ---2018.11.7
//		HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
//}

/**
  * @brief  FIFO0中断接收任务的回调函数
  * @param  *hcan
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
	{
		Motor_Data_Receive();
	}
	if(hcan->Instance==CAN2)
	{
		
	}
}

