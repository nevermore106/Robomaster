/**
  ******************************************************************************
  * @file    referee.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.1
  * @date    2021/4/16
  * @brief   ����ϵͳͷ����
  ******************************************************************************
  * @attention

  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "universal.h"
//����ϵͳ�ṹ��
typedef struct
{
	u8 CmdID;                           //���ݵ�ID
	u8 DMA_flag;                        //DMA��־λ
	u8 ErrorFlag;                       //��������Ƿ�ʧ���־λ
	u8 Referee_count[3];                //���ݷ��ʹ�������֤50HZ
	u16 ErrorCount;                     //�������ݼ���
	u16 datasize;                       //DMA���ݳ��ȴ���
	
	float RealVoltage;                  //ʵʱ��ѹ
	float RealCurrent;                  //ʵʱ����
	float RealPower;                    //ʵʱ����
	float RemainPower;                  //������ʣ������
	float ShootSpeed;                   //�ӵ�����ٶ�
    
	u16 Real17mm1Heating;                    //ʵʱ����
    u16 Real17mm2Heating;                    //ʵʱ����
    u16 Real42mmHeating;                    //ʵʱ����
	u16 RemainBlood;                	 	//ʣ��Ѫ��
	u16 Shoot17mm1CoolingRate;               //ǹ����ȴ�ٶ�
	u16 Shoot17mm1SpeedLimit;								//ǹ����������
	u16 Shoot17mm1CoolingLimit;              //ǹ����������
    u16 Shoot17mm2CoolingRate;               //ǹ����ȴ�ٶ�
	u16 Shoot17mm2CoolingLimit;              //ǹ����������
    u16 Shoot42mmCoolingRate;               //ǹ����ȴ�ٶ�
	u16 Shoot42mmCoolingLimit;              //ǹ����������
	u16 ShootFrequency;                 //�ӵ����Ƶ��
    u16 Bullet17RemainingNum;               //17mm�ɷ����ӵ�����
    u16 Bullet42RemainingNum;               //42mm�ɷ����ӵ�����
		
		u16 ChassisPowerLimit;                  //���̹�������
    
	u8 RobotID;
	u8 RobotLevel;
	u16 InjureCount;
	u8 InjureMark[4];
}InfantryJudge_Struct;


typedef union     //float��������
{  
    struct   
    {  
        unsigned char low_byte;  
        unsigned char mlow_byte;  
        unsigned char mhigh_byte;  
        unsigned char high_byte;  
     }float_byte;  
            
     float  value;  
}FLAOT_UNION; 

void Referee_Receive(void);
extern u8 JudgeDataBuffer[80];
extern u8 JudgeSendBuff[150];
extern InfantryJudge_Struct InfantryJudge;

