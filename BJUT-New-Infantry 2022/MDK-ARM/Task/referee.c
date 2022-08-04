/**
  ******************************************************************************
  * @file    referee.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.1
  * @date    2021/4/16
  * @brief   ����ϵͳ���ݴ���
  ******************************************************************************
  * @attention
  *	
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

InfantryJudge_Struct InfantryJudge={0};
FLAOT_UNION P_deal;//ʵʱ����
FLAOT_UNION V_deal;//ʵʱ����

FLAOT_UNION DataA;
FLAOT_UNION DataB;
FLAOT_UNION DataC;

u8 JudgeHeadBuffer[7]={0};
u8 JudgeCheckBuffer[80]={0};
u8 JudgeDataBuffer[80]={0};
u8 JudgeSendBuff[150]={0};

/**
  * @brief  ����ϵͳ���ݽ���
	*         ͨ��CRCУ������ݽ���֡�ʶ�����ʵʱ�Խ����ж�
	*         ��Щ����û�н��н��գ������Ҫ���Բ鿴�ֲ�
  * @param  None
  * @retval None
  */
void Referee_Receive(void)
{
	u8 group_count=0;
	if((JudgeDataBuffer[0] == 0XA5) && (Verify_CRC8_Check_Sum(JudgeDataBuffer, 5) == 1))//����֡ͷ�Ƿ���0XA5��CRCУ��
	{
		InfantryJudge.datasize = (JudgeDataBuffer[1])|(JudgeDataBuffer[2]<<8);//��ȡ�����ֽڳ���		
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,JudgeDataBuffer,InfantryJudge.datasize+2);
		InfantryJudge.Referee_count[2] = JudgeDataBuffer[3];//���ݷ��ʹ�������������Ƿ�ʧ��		
		InfantryJudge.CmdID = (JudgeDataBuffer[5])|(JudgeDataBuffer[6]<<4);//��ȡ����֡ID����16λѹ��Ϊ8λ��
		//���ݽ��մ������飬�����Ƿ�ʧ
		if((InfantryJudge.Referee_count[2] - InfantryJudge.Referee_count[1] != 1) && (InfantryJudge.Referee_count[2] - InfantryJudge.Referee_count[1] != (-255)))
		{
			InfantryJudge.ErrorFlag = ERROR;
			InfantryJudge.ErrorCount++;
		}
		InfantryJudge.Referee_count[1] = InfantryJudge.Referee_count[2];//���ݵ���
		InfantryJudge.DMA_flag = 0;//DMA��־λ
		for(group_count = 0;group_count < 7;group_count++)//��Ϊ���ݲ�ֽ��գ�CRCУ����Ҫ�������ݣ��ڴ˽���ǰ�벿����������
			{JudgeCheckBuffer[group_count] = JudgeDataBuffer[group_count];}
	}
	else if(InfantryJudge.DMA_flag == 0)
	{
		InfantryJudge.DMA_flag = 1;
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,JudgeDataBuffer,7);
		for(group_count = 0;group_count < InfantryJudge.datasize + 2;group_count++)//��벿����������
			JudgeCheckBuffer[7 + group_count] = JudgeDataBuffer[group_count];
		if(Verify_CRC16_Check_Sum(JudgeCheckBuffer, InfantryJudge.datasize + 9) == 1)//У��CRC
		{
			RefereeCount++;
			switch(InfantryJudge.CmdID)
			{
				case 0x01://����״̬���� 3bytes
					break;
				case 0x02://�����������
					break;
				case 0x03://���������˴������
					break;
				case 0x11://�����¼�����
					break;
				case 0x12://����վ��ʶ����
					break;
				case 0x13://ԤԼ�ӵ�����
					break;
				case 0x14://���о�����Ϣ
					break;
				case 0x21://������״̬���� 10Hz 15bytes  
				{
					InfantryJudge.RobotID           =  JudgeDataBuffer[0];
					InfantryJudge.RobotLevel        =  JudgeDataBuffer[1];//�����˵ȼ� 1��2��3��
					InfantryJudge.RemainBlood       = (JudgeDataBuffer[2]) | (JudgeDataBuffer[3] << 8);
					InfantryJudge.Shoot17mm1CoolingRate  = (JudgeDataBuffer[6])  | (JudgeDataBuffer[7] << 8);
					InfantryJudge.Shoot17mm1CoolingLimit = (JudgeDataBuffer[8])  | (JudgeDataBuffer[9] << 8);
					InfantryJudge.Shoot17mm1SpeedLimit = (JudgeDataBuffer[10])  | (JudgeDataBuffer[11] << 8);
					InfantryJudge.Shoot17mm2CoolingRate  = (JudgeDataBuffer[12]) | (JudgeDataBuffer[13] << 8);
					InfantryJudge.Shoot17mm2CoolingLimit = (JudgeDataBuffer[14]) | (JudgeDataBuffer[15] << 8);
					InfantryJudge.Shoot42mmCoolingRate  = (JudgeDataBuffer[18])  | (JudgeDataBuffer[19] << 8);
					InfantryJudge.Shoot42mmCoolingLimit = (JudgeDataBuffer[20])  | (JudgeDataBuffer[21] << 8);
					InfantryJudge.ChassisPowerLimit = (JudgeDataBuffer[24])  | (JudgeDataBuffer[25] << 8);//���̹�������
					break;
				}
				case 0x22://ʵʱ������������ 50Hz 14bytes
				{
					InfantryJudge.RealVoltage    = ((float)((JudgeDataBuffer[0]) | (JudgeDataBuffer[1] << 8)) / 1000);
					InfantryJudge.RealCurrent    = ((float)((JudgeDataBuffer[2]) | (JudgeDataBuffer[3] << 8)) / 1000);
					InfantryJudge.RemainPower    =          (JudgeDataBuffer[8]) | (JudgeDataBuffer[9] << 8);
					InfantryJudge.Real17mm1Heating   =          (JudgeDataBuffer[10])| (JudgeDataBuffer[11]<< 8);
                    InfantryJudge.Real17mm2Heating   =          (JudgeDataBuffer[12])| (JudgeDataBuffer[13]<< 8);
                    InfantryJudge.Real42mmHeating    =          (JudgeDataBuffer[14])| (JudgeDataBuffer[15]<< 8);
					P_deal.float_byte.low_byte   = JudgeDataBuffer[4];
					P_deal.float_byte.mlow_byte  = JudgeDataBuffer[5];
					P_deal.float_byte.mhigh_byte = JudgeDataBuffer[6];
					P_deal.float_byte.high_byte  = JudgeDataBuffer[7];
					InfantryJudge.RealPower      = P_deal.value;
					break;
				}
				case 0x23://ʵʱλ������ 10Hz 16bytes
					break;
				case 0x24://�������� 1byte
					break;
				case 0x25://���л������������� 10Hz 3bytes
					break;
				case 0x26://�˺�״̬���� 1byte
				{
						if(JudgeDataBuffer[0] == 0x00)
							{
								Command.RotateFlag = 1;
		            InfantryJudge.InjureCount ++;
								InfantryJudge.InjureMark[0] = 50;
							}
						else if(JudgeDataBuffer[0]  == 0x01)
							{
								Command.RotateFlag = 1;
	              InfantryJudge.InjureCount ++;
								InfantryJudge.InjureMark[1] = 50;
								GM6020[YAW].Rotate = 2048;
							}
						else if(JudgeDataBuffer[0] == 0x02)
							{
								Command.RotateFlag = 1;
	              InfantryJudge.InjureCount ++;
								InfantryJudge.InjureMark[2] = 50;
								GM6020[YAW].Rotate = 4096;
							}
						else if(JudgeDataBuffer[0] == 0x03)
							{
								Command.RotateFlag = 1;
	              InfantryJudge.InjureCount ++;
								InfantryJudge.InjureMark[3] = 50;
								GM6020[YAW].Rotate = - 2048;
							}
					break;
        }        				
				case 0x27://ʵʱ������� 6bytes
				{
					InfantryJudge.ShootFrequency = JudgeDataBuffer[3];
					V_deal.float_byte.low_byte   = JudgeDataBuffer[4];
					V_deal.float_byte.mlow_byte  = JudgeDataBuffer[5];
					V_deal.float_byte.mhigh_byte = JudgeDataBuffer[6];
					V_deal.float_byte.high_byte  = JudgeDataBuffer[7];
					InfantryJudge.ShootSpeed     = V_deal.value;
					break;
				}
                case 0x28://�ӵ�ʣ�෢������
                {
                    InfantryJudge.Bullet17RemainingNum = (JudgeDataBuffer[0]) | (JudgeDataBuffer[1] << 8);
                    InfantryJudge.Bullet42RemainingNum = (JudgeDataBuffer[2]) | (JudgeDataBuffer[3] << 8);
                    break;
                }
				case 0x31://ʵʱ�������� 10Hz n-byte		
					break;
				default:
					break;
			}
		}
		else//���CRCû������Ϊ����ʧ��
		{
			InfantryJudge.ErrorFlag = ERROR;
			InfantryJudge.ErrorCount++;
		}
	}
	else
	{
		HAL_UART_DMAStop(&huart6);
		HAL_UART_Receive_DMA(&huart6,JudgeDataBuffer,7);
	}
}


