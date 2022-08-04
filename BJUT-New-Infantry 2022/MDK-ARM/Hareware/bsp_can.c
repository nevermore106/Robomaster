/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.1
  * @date    2018/11/8
  * @brief   CAN�����˲����ĳ�ʼ��
  ******************************************************************************
  * @attention
  *  ��ι̼���İ汾������ǰ�õİ汾Ҫ�ߣ��ܶ�ײ㺯�������ƺͲ��������˸��ˣ�
	*ʹ�������Ƚ����ѡ�
  *  CAN1��CAN2�ĳ�ʼ��һ��Ҫע�⣬CAN2�ǹ�����CAN1�ϵģ�CUBEMX�����ɵ�ʱ��һ��
	*Ҫ��һ�£��Ƿ��CAN1�ĳ�ʼ��������CAN2��ǰ�ߣ������ǲ���ʹ�õġ�
	*  ��ʼ����ʱ���˲�������϶࣬HAL��̼��İ汾��Ӱ���ʼ������ʼ�����ɹ��Ļ�
	*Ҫ�侲�����Ƶ�һ����ȥ��������ĵط�����ײ�����ʱ��һ��Ҫϸ�ġ�
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

/**
  * @brief  CAN��������
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
  * @brief  CAN1���˲�����ʼ��
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
//����ΪʲôҪ��һ���ж�����أ����ʼ�������Ҹ����ˡ��ּ�������һ��
//�⣬������֮��ײ��һЩ��װ����ȫ���ˣ���ֻ�����ˡ�����
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  CAN2���˲�����ʼ��
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
//����д�Ļ���CAN1��������Ӧ����CAN2������CAN2�˲�������ƫ�Ƶ�ַ�²�û��
//�Ĵ���������CAN1 CAN2�����˲��������԰�ƫ�Ƶ�ַָ����CAN1�����ڸ����˹�
//���汾����֪�����BUG��û�и��£���Ҫ���һ�¡�  ---2018.11.7
//		HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
//}

/**
  * @brief  FIFO0�жϽ�������Ļص�����
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

