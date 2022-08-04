/**
  ******************************************************************************
  * @file    start_task.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   初始化任务函数
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
osThreadId Detect_TaskHandle;
osThreadId Chassis_TaskHandle;
osThreadId Gimbal_TaskHandle;
osThreadId Remote_TaskHandle;
osThreadId User_TaskHandle;
osThreadId Cap_TaskHandle;
extern osThreadId Start_TaskHandle;



void Start_Task(void)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	taskENTER_CRITICAL();//进入临界区
	//创建监视进程
	osThreadDef(Detect_Task, Detect_Task, osPriorityLow, 0, 128);
  Detect_TaskHandle = osThreadCreate(osThread(Detect_Task), NULL);
	
	taskEXIT_CRITICAL();            //退出临界区
	
	Imu_Init();//惯导模块初始化

	osDelayUntil(&xLastWakeTime,1000);
	taskENTER_CRITICAL();//进入临界区
	//创建底盘进程
	osThreadDef(Chassis_Task, Chassis_Task, osPriorityNormal, 0, 256);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);
	
	//创建云台进程
	osThreadDef(Gimbal_Task, Gimbal_Task, osPriorityNormal, 0, 512);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);
	
	//创建遥控进程
	osThreadDef(Remote_Task, Remote_Task, osPriorityHigh, 0, 128);
  Remote_TaskHandle = osThreadCreate(osThread(Remote_Task), NULL);

  //创建用户进程
	osThreadDef(User_Task, User_Task, osPriorityBelowNormal, 0, 256);
  User_TaskHandle = osThreadCreate(osThread(User_Task), NULL);
	
	//创建电容进程
	osThreadDef(Cap_Task, Cap_Task, osPriorityBelowNormal, 0, 128);
  Cap_TaskHandle = osThreadCreate(osThread(Cap_Task), NULL);
	
	User_Init();
	vTaskDelete(Start_TaskHandle);	//删除初始化任务
	taskEXIT_CRITICAL();            //退出临界区
}

void User_Init(void)
{
	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;	
	RC_Ctl.rc.s1  = 3;
	RC_Ctl.rc.s2  = 3;


	InfantryJudge.Shoot17mm1CoolingLimit = 100;
	InfantryJudge.RealVoltage 			= 25.2F;
	InfantryJudge.RealCurrent 			= 0;
	InfantryJudge.RemainBlood			  = 250;
	InfantryJudge.RemainPower       = 60;
	
	Shooting.RemainHeating          = 360;
	Shooting.Period                 = 64;//此数值最大80，射频 = 1000 / 周期
	Shooting.PosRatio               = 16;
	PowerLimit.Flag = 0;
	PowerLimit.RemainPower[2] = 60;
	PowerLimit.Real_Power[2]  = 75;
	PowerLimit.MaxSpeed       = CHASSIS_MAX_SPEED;//设定最大转速
	Chassis.Position.Flag     = 1;
	
#if YAW_USE_IMUEX == 1
	GM6020[YAW].Position.Set   = Imuex.Angle.Yaw;
	GM6020[YAW].Position.Step  = Imuex.Angle.Yaw;
#elif YAW_USE_IMUEX == 0
	GM6020[YAW].Position.Set   = imu.yaw;
	GM6020[YAW].Position.Step  = imu.yaw;
#endif
	
#if PITCH_POSITION_USE_IMU == 1                    
	GM6020[PITCH].Position.Set  = PITCH_POSITION_INIT;
	GM6020[PITCH].Position.Step = Imuex.Angle.Pitch;
#elif PITCH_POSITION_USE_IMU == 0
	GM6020[PITCH].Position.Set  = PITCH_POSITION_INIT;
	GM6020[PITCH].Position.Step = GM6020[PITCH].Position.Real;
#endif

  SpinTop.Speed = ROTATE_NORMAL_SPEED;
	Command.SpeedMinish = 1;
	Command.SpeedZoom = 1;
  Command.Mouse_x = imu.yaw;
  Command.Mouse_y = PITCH_POSITION_INIT;
	
  FirstOrder_KalmanFilter_Init(&Gimbal_Position_Kalman[PITCH],1,5);
	FirstOrder_KalmanFilter_Init(&Gimbal_Speed_Kalman[PITCH],1,1);
//	FirstOrder_KalmanFilter_Init(&Imu_Kalman,1,40);
	//PID初始化分别为KP KI KD 最大输出 死区 积分范围 最大输入 最大积分值（输出值=最大积分值*KI）
	//积分范围只有在位置式PID计算的时候起作用 
/*****************************************************************************************/
/****************************************底盘*********************************************/
/*****************************************************************************************/
	//底盘电机
	PID_Init(&M3508_Speed_pid[0],12,0.04f,0,13000,0,2000,CHASSIS_MAX_SPEED,1500,INCREMENTAL);
	PID_Init(&M3508_Speed_pid[1],12,0.04f,0,13000,0,2000,CHASSIS_MAX_SPEED,1500,INCREMENTAL);
	PID_Init(&M3508_Speed_pid[2],12,0.04f,0,13000,0,2000,CHASSIS_MAX_SPEED,1500,INCREMENTAL);
	PID_Init(&M3508_Speed_pid[3],12,0.04f,0,13000,0,2000,CHASSIS_MAX_SPEED,1500,INCREMENTAL);
	
	//底盘回正
	PID_Init(&Chassis_Rotate_pid,5,0,200,5000,0,2400,8192,0,POSITIONAL);

  //小陀螺
	PID_Init(&Spin_Top_pid,0.5f,0,0,180,0,90,180,90,POSITIONAL);
		
	//功率限制
	PID_Init(&Power_Limit_pid,0.01f,0.00008f,0,0.9,0,80,200,0.8,POSITIONAL);
/*****************************************************************************************/
/*****************************************云台********************************************/
/*****************************************************************************************/
	//PITCH云台		
#if PITCH_USE_ADRC == 1 
  //ADRC位置环
   ADRC_Init(&Gimbal_adrc[YAW],&Gimbal_adrc[PITCH]);
	//ADRC速度环
	 #if PITCH_SPEED_USE_IMU == 1
    PID_Init(&Gimbal_Speed_pid[PITCH],35,0.8f,0,29000,0,10000,250,1400,POSITIONAL);
   #elif PITCH_SPEED_USE_IMU == 0
    PID_Init(&Gimbal_Speed_pid[PITCH],150,1.5f,0,29000,0,8,250,900,POSITIONAL);
   #endif
	
#elif PITCH_USE_ADRC == 0 
	//PID位置环
	//PID速度环		
	 #if (PITCH_SPEED_USE_IMU == 1)&&(PITCH_POSITION_USE_IMU == 1)
	  PID_Init(&Gimbal_Position_pid[PITCH],0.35f,0,0,10000,0,4000,8192,10,POSITIONAL);
    PID_Init(&Gimbal_Speed_pid[PITCH],280,3.0f,0,30000,0,10000,250,1500,POSITIONAL);
   #elif (PITCH_SPEED_USE_IMU == 1)&&(PITCH_POSITION_USE_IMU == 0)
	  PID_Init(&Gimbal_Position_pid[PITCH],0.75f,0,0,10000,0,4000,8192,10,POSITIONAL);
    PID_Init(&Gimbal_Speed_pid[PITCH],290,3.0f,0,30000,0,10000,250,900,POSITIONAL);
	 #elif (PITCH_SPEED_USE_IMU == 0)&&(PITCH_POSITION_USE_IMU == 1)
	  PID_Init(&Gimbal_Position_pid[PITCH],0.18f,0,0,10000,0,4000,8192,10,POSITIONAL);
    PID_Init(&Gimbal_Speed_pid[PITCH],230,1.5f,0,30000,0,10000,250,800,POSITIONAL);
	 #elif (PITCH_SPEED_USE_IMU == 0)&&(PITCH_POSITION_USE_IMU == 0)
	  PID_Init(&Gimbal_Position_pid[PITCH],0.14f,0,0,10000,0,4000,8192,10,POSITIONAL);
		PID_Init(&Gimbal_Speed_pid[PITCH],350,4.0f,0,30000,0,200,250,1300,POSITIONAL);
   #endif

#endif

//转矩环
#if PITCH_USE_CURRENT == 1	
	PID_Init(&Gimbal_TCurrent_pid[PITCH],0.8f,0,0,28500,0,30000,10000,4000,POSITIONAL);
#endif
		
//PITCH自稳
#if PITCH_STABLE == 1		
//	PID_Init(&Pitch_Stable_pid,0.45f,0.0005f,0,620,0,80,8192,200,POSITIONAL);
	PID_Init(&Pitch_Stable_pid,1.0f,0,0,620,0,620,8192,320,POSITIONAL);
#endif

	//YAW云台
#if YAW_USE_IMUEX == 1 //外挂陀螺仪
	PID_Init(&Gimbal_Position_pid[YAW],0.8f,0,0,10000,0,20,8192,2000,POSITIONAL);
  PID_Init(&Gimbal_Speed_pid[YAW],400,3.0f,0,30000,0,10000,250,2000,POSITIONAL);
//	PID_Init(&Gimbal_TCurrent_pid[YAW],0.8f,0,0,30000,0,20,8192,1000,POSITIONAL);
		
#elif YAW_USE_IMUEX == 0	//A板陀螺仪	
	PID_Init(&Gimbal_Position_pid[YAW],0.017f,0,0,10000,0,20,8192,2000,POSITIONAL);
	PID_Init(&Gimbal_Speed_pid[YAW],28000,5.5f,0,30000,0,10000,35,2000,POSITIONAL);
//	PID_Init(&Gimbal_TCurrent_pid[YAW],0.8f,0,0,30000,0,20,8192,1000,POSITIONAL);
#endif

//转矩环，暂时没用
#if YAW_USE_CURRENT == 1	
	PID_Init(&Gimbal_TCurrent_pid[YAW],0.8f,0,0,28500,0,30000,10000,4000,POSITIONAL);
#endif
	
//拨弹
PID_Init(&Gimbal_Speed_pid[TRIGGER],12,0.01f,0.5f,10000,0,10000,10000,800,POSITIONAL);
PID_Init(&Gimbal_Position_pid[TRIGGER],6,0,0,10000,0,0,8192,2000,POSITIONAL);
 
}

