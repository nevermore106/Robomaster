/**
  ******************************************************************************
  * @file    referee.h
  * @author  北京工业大学-贾桐
  * @version V1.1
  * @date    2021/4/16
  * @brief   裁判系统头函数
  ******************************************************************************
  * @attention

  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "universal.h"
//裁判系统结构体
typedef struct
{
	u8 CmdID;                           //数据的ID
	u8 DMA_flag;                        //DMA标志位
	u8 ErrorFlag;                       //检测数据是否失真标志位
	u8 Referee_count[3];                //数据发送次数，保证50HZ
	u16 ErrorCount;                     //错误数据计数
	u16 datasize;                       //DMA数据长度处理
	
	float RealVoltage;                  //实时电压
	float RealCurrent;                  //实时电流
	float RealPower;                    //实时功率
	float RemainPower;                  //能量槽剩余能量
	float ShootSpeed;                   //子弹射击速度
    
	u16 Real17mm1Heating;                    //实时热量
    u16 Real17mm2Heating;                    //实时热量
    u16 Real42mmHeating;                    //实时热量
	u16 RemainBlood;                	 	//剩余血量
	u16 Shoot17mm1CoolingRate;               //枪口冷却速度
	u16 Shoot17mm1SpeedLimit;								//枪口射速上限
	u16 Shoot17mm1CoolingLimit;              //枪口热量上限
    u16 Shoot17mm2CoolingRate;               //枪口冷却速度
	u16 Shoot17mm2CoolingLimit;              //枪口热量上限
    u16 Shoot42mmCoolingRate;               //枪口冷却速度
	u16 Shoot42mmCoolingLimit;              //枪口热量上限
	u16 ShootFrequency;                 //子弹射击频率
    u16 Bullet17RemainingNum;               //17mm可发射子弹数量
    u16 Bullet42RemainingNum;               //42mm可发射子弹数量
		
		u16 ChassisPowerLimit;                  //底盘功率限制
    
	u8 RobotID;
	u8 RobotLevel;
	u16 InjureCount;
	u8 InjureMark[4];
}InfantryJudge_Struct;


typedef union     //float数据重组
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

