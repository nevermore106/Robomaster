/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2022/2/7
  * @brief   云台进程
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
#include "tim.h"
Motor_t GM6020[2] = {0}; //YAW 0 PITCH 1 
Motor_t GM2006 = {0}; 
Stable_t Stable = {0};
Filter_t Kalman[2] = {0};
s16 Gravity_Moment = - 400;
s16 pitc = 40;
s16 yawc = -35;

AutoLimit_t AutoLimit[2] = {{YAW_POSITION_MAX,YAW_POSITION_MIN,8192,0},{PITCH_POSITION_MAX,PITCH_POSITION_MIN,8192,0}};

void Gimbal_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
#if (YAW_USE_IMUEX == 1) && (PITCH_POSITION_USE_IMU == 1)
	kalman_fiflter_set(Imuex.Angle.Yaw,Imuex.Angle.Pitch);
	
#elif (YAW_USE_IMUEX == 1) && (PITCH_POSITION_USE_IMU == 0)
	kalman_fiflter_set(Imuex.Angle.Yaw,GM6020[PITCH].Position.Real);
	
#elif (YAW_USE_IMUEX == 0) && (PITCH_POSITION_USE_IMU == 1)
	kalman_fiflter_set(imu.yaw,Imuex.Angle.Pitch);
	
#elif (YAW_USE_IMUEX == 0) && (PITCH_POSITION_USE_IMU == 0)
	kalman_fiflter_set(imu.yaw,GM6020[PITCH].Position.Real);
#endif

  while(1)
  {
		osDelayUntil(&xLastWakeTime,2);//500HZ
//陀螺仪数据获取
		Imu_Update();//板载陀螺仪，当板子立着放的时候不好用（其实是算法不行)
		
		#if YAW_USE_IMUEX == 1		
		  imuex_attitude_update();//外接陀螺仪			
    #endif	
//视觉对云台的控制			
		Vision_Control_Gimbal();
/**********************************Yaw轴控制**********************************/
		//限定输入值范围
		GM6020[YAW].Position.Set = Func_ValueRannge(GM6020[YAW].Position.Set,8192,0);
		
		YAW_PID_WithIMU();
		
	//YAW转矩环
	  #if YAW_USE_CURRENT == 1	
      PID_Calc(&Gimbal_TCurrent_pid[YAW],(-GM6020[YAW].TCurrent),Gimbal_Speed_pid[YAW].Output);//转矩环PID
		  GM6020[YAW].Output = Gimbal_TCurrent_pid[YAW].Output;
	  #elif YAW_USE_CURRENT == 0
      GM6020[YAW].Output = Gimbal_Speed_pid[YAW].Output;		
    #endif
/*********************************Pitch轴控制********************************/
//限制输入范围
		#if PITCH_POSITION_USE_IMU == 1
     GM6020[PITCH].Position.Set = Func_Limit(GM6020[PITCH].Position.Set,5000,3000);//无补偿，测试用
    #elif PITCH_POSITION_USE_IMU == 0
     GM6020[PITCH].Position.Set = Func_Limit(GM6020[PITCH].Position.Set,PITCH_POSITION_MAX,PITCH_POSITION_MIN);//无补偿，测试用
    #endif	
		
//对编码器数值进行滤波
    Gimbal_FirstOrder_KalmanFilter();//主要用于观测误差，补帧效果不好(也可能是没调好QR)
		
//水平垂稳  
	  #if PITCH_STABLE == 1		
		  Pitch_Stable();	
    #endif
		
    //ADRC控制
    #if PITCH_USE_ADRC == 1
      PITCH_ADRC_Control();	
		//PID控制	
    #elif PITCH_USE_ADRC == 0			
      PITCH_PID_Control();
    #endif
		
//PITCH转矩环和重力矩补偿		
		#if PITCH_USE_CURRENT == 1	
  		PID_Calc(&Gimbal_TCurrent_pid[PITCH],(-GM6020[PITCH].TCurrent),Gimbal_Speed_pid[PITCH].Output);//转矩环
  		Gimbal_TCurrent_pid[PITCH].Output += Gravity_Compensate(); 
      GM6020[PITCH].Output = Gimbal_TCurrent_pid[PITCH].Output;
    #elif PITCH_USE_CURRENT == 0		
      Gimbal_Speed_pid[PITCH].Output += Gravity_Compensate();
			GM6020[PITCH].Output = Gimbal_Speed_pid[PITCH].Output;
		#endif
/**********************************射击控制**********************************/		
//射击控制
		Shooting_Control();
		
//拨弹控制		
    Feed_PID_Control();
		
//发送电流值，当遥控器右侧拨杆处于中位时，云台无输出
		(RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(GM6020[YAW].Output,GM6020[PITCH].Output,Gimbal_Speed_pid[TRIGGER].Output);
//测试用
//    (RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(GM6020[YAW].Output,0,Gimbal_Speed_pid[TRIGGER].Output);//无pitch
//		(RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(0,GM6020[PITCH].Output,Gimbal_Speed_pid[TRIGGER].Output);//无yaw
//  	(RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(0,0,Gimbal_Speed_pid[TRIGGER].Output);//无pitch无yaw
//     Gimbal_Output(0,0,0);//无输出
	}
}


/**
  * @brief  射击任务
  * @param  None
  * @retval 状态值
  */
Shooting_t Shooting = {0};

uint16_t motor_pos[80] = {0};   
uint16_t reversal_time_count = 0;
void Shooting_Control(void)
{
//记录历史数据
	motor_pos[79] = GM2006.Position.Convert;
	for(uint8_t i = 0;i < 79;i++)
	{
		motor_pos[i] = motor_pos[i+1];
	} 
    
//摩擦轮有一个缓启动函数，遥控任务里控制开启标志位
	if(Shooting.Friction == 1)
	{
//		switch(InfantryJudge.Shoot17mm1SpeedLimit)
//		{
//			case 15:
//				Friction_ON(1182);
//			  Shooting.Speed = 15000;
//			break;
//			case 18:
//				Friction_ON(1250);
//			  Shooting.Speed = 18000;
//			break;
//			case 30:
				Friction_ON(1440);
			  Shooting.Speed = 29000;
//			break;
//			default:
//				Friction_ON(1182);
//			  Shooting.Speed = 15000;
//			break;
//		}
		//3代步兵：1500-25.8m/s, 1680-28m/s (max：30.6m/s), 1630-25.2~29.9m/s
	}
	else
	{
		Friction_OFF();
		Shooting.Speed = 0;
	} 
    
//判断热量是否允许射击
//	if(InfantryJudge.Shoot17mm1CoolingLimit - InfantryJudge.Real17mm1Heating >= 30 && Shooting.Friction == 1)
	if(Shooting.Friction == 1)
    {
		Shooting.Allow = OK;
    }
	else
	{
		Shooting.Allow = NO;
		Shooting.PosSetCount = 0;
	}
	
	if((Shooting.PosSetCount != 0) && (Shooting.Allow == OK))//发射子弹
	{
		if(reversal_time_count == 0)
		{
			Shooting.PosSetCount--;
			GM2006.Position.Set -= Shooting.PosRatio;   //正反转
		}
		if(motor_pos[79] == motor_pos[0] && (Shooting.PosSetCount < Shooting.Period * 0.47))//出现了不动的情况认为卡子弹了
		{
      GM2006.Position.Step = GM2006.Position.Convert;
      GM2006.Position.Set  = GM2006.Position.Convert;
      reversal_time_count  = Shooting.Period * 0.47;
		}		
	}
	
	if(reversal_time_count != 0)//卡子弹的话就反转
	{
		reversal_time_count--;
		GM2006.Position.Set += Shooting.PosRatio; //正反转
	}	
}


/**
  * @brief  YAW轴控制
  * @param  None
  * @retval 状态值
  */
void YAW_PID_WithIMU(void)
{
//    Gimbal_Dynamic_Limit(&GM6020[YAW].Position.Set,GM6020[YAW].Position.Real,Imuex.Angle.Yaw,&AutoLimit[YAW]);//通过陀螺仪与编码协同计算是否超出设置量程，因为用了小陀螺，故取消
		Func_CircleRamp(GM6020[YAW].Position.Set,&GM6020[YAW].Position.Step,YAW_POSITION_RAMP_RATIO);//斜坡启动函数
	  #if YAW_USE_IMUEX == 1
      PID_Calc(&Gimbal_Position_pid[YAW],Imuex.Angle.Yaw,GM6020[YAW].Position.Step);//位置环PID
		  PID_Calc(&Gimbal_Speed_pid[YAW],Imuex.Speed.Yaw,Gimbal_Position_pid[YAW].Output);//速度环PID
		
    #elif YAW_USE_IMUEX == 0		
		  PID_Calc(&Gimbal_Position_pid[YAW],imu.yaw,GM6020[YAW].Position.Step);//位置环PID
		  PID_Calc(&Gimbal_Speed_pid[YAW],imu.wz,Gimbal_Position_pid[YAW].Output);//速度环PID
    #endif
	
}

void YAW_PID_WithoutIMU(void)
{
		Func_CircleRamp(GM6020[YAW].Position.Set,&GM6020[YAW].Position.Step,YAW_POSITION_RAMP_RATIO);//斜坡启动函数
		PID_Calc(&Gimbal_Position_pid[YAW],GM6020[YAW].Position.Real,GM6020[YAW].Position.Step);//位置环PID
		PID_Calc(&Gimbal_Speed_pid[YAW],GM6020[YAW].Speed.Real,Gimbal_Position_pid[YAW].Output);//速度环PID
}

/**
  * @brief  PITCH轴控制
  * @param  None
  * @retval 状态值
  */
void PITCH_PID_Control(void)
{
//		Gimbal_Dynamic_Limit(&GM6020[PITCH].Position.Set,GM6020[PITCH].Position.Real,Imuex.Angle.Pitch,&AutoLimit[PITCH]);//因反馈采用陀螺仪，需要动态限制范围		
	 Func_CircleRamp(GM6020[PITCH].Position.Set,&GM6020[PITCH].Position.Step,PITCH_POSITION_RAMP_RATIO);

	 #if PITCH_POSITION_USE_IMU == 1
    PID_Calc(&Gimbal_Position_pid[PITCH],Imuex.Angle.Pitch,GM6020[PITCH].Position.Step);
   #elif PITCH_POSITION_USE_IMU == 0
    PID_Calc(&Gimbal_Position_pid[PITCH],GM6020[PITCH].Position.Real,GM6020[PITCH].Position.Step);
   #endif
   
	 #if PITCH_SPEED_USE_IMU == 1
    PID_Calc(&Gimbal_Speed_pid[PITCH],Imuex.Speed.Pitch,Gimbal_Position_pid[PITCH].Output);
   #elif PITCH_SPEED_USE_IMU == 0
    PID_Calc(&Gimbal_Speed_pid[PITCH],GM6020[PITCH].Speed.Real,Gimbal_Position_pid[PITCH].Output);
   #endif
	
}

void PITCH_ADRC_Control(void)
{
	 #if PITCH_POSITION_USE_IMU == 1
    ADRC_Control(&Gimbal_adrc[PITCH],Imuex.Angle.Pitch,GM6020[PITCH].Position.Set);//位置环ADRC
   #elif PITCH_POSITION_USE_IMU == 0
    ADRC_Control(&Gimbal_adrc[PITCH],GM6020[PITCH].Position.Real,GM6020[PITCH].Position.Set);
//	  ADRC_Control(&Gimbal_adrc[PITCH],kalman_position_watch,GM6020[PITCH].Position.Set);
   #endif
			
	 #if PITCH_SPEED_USE_IMU == 1
    PID_Calc(&Gimbal_Speed_pid[PITCH],Imuex.Speed.Pitch,Gimbal_adrc[PITCH].u);
   #elif PITCH_SPEED_USE_IMU == 0
    PID_Calc(&Gimbal_Speed_pid[PITCH],GM6020[PITCH].Speed.Real,Gimbal_adrc[PITCH].u);
   #endif
	
}

/**
  * @brief  拨弹控制
  * @param  None
  * @retval 状态值
  */
void Feed_PID_Control(void)
{
		GM2006.Position.Set = Func_ValueRannge(GM2006.Position.Set,8192,0);//限制输入范围
		//Func_CircleRamp(GM2006.Position.Set,&GM2006.Position.Step,TRIGGER_POSITION_RAMP_RATIO);//斜坡启动函数
		PID_Calc(&Gimbal_Position_pid[TRIGGER],GM2006.Position.Convert,GM2006.Position.Set);//位置环PID
		PID_Calc(&Gimbal_Speed_pid[TRIGGER],GM2006.Speed.Real,Gimbal_Position_pid[TRIGGER].Output);//速度环PID	
}

/**
  * @brief  水平垂稳
  * @param  None
  * @retval 状态值
  */
void Pitch_Stable(void)
{
//	  Stable.Error = (Imuex.Angle.Pitch - 4096);
//    if(Command.RotateFlag == 1)
//		PID_Calc(&Pitch_Stable_pid,Stable.Error,0);//垂稳补偿PID
//		else
//		PID_Clear(&Pitch_Stable_pid);
		
//		GM6020[PITCH].Position.Set +=  imu.pit;	

	  Kalman[PITCH].Imu.Step = imu.pit;
		Kalman[PITCH].Imu.Real = FirstOrder_KalmanFilter_Cacl(&Gimbal_Position_Kalman[PITCH],Kalman[PITCH].Imu.Step,0);
	  GM6020[PITCH].Position.Set += (int)Kalman[PITCH].Imu.Real;
    Kalman[PITCH].Imu.Error = GM6020[PITCH].Position.Set - Kalman[PITCH].Imu.Real;
}

/**
  * @brief  重力矩补偿
  * @param  None
  * @retval 状态值
  */
s16 Gravity_Compensate(void)
{	
   s16 gravity_moment = Gravity_Moment;
	 float gravity_theta = 0;
	 float gravity_costheta = 0;
	 float gravity_Compensate = 0;
	
	 #if PITCH_POSITION_USE_IMU == 1
	  gravity_theta = (Imuex.Angle.Pitch - PITCH_POSITION_INIT) * NORMAL_PI / 4096.0f;
   #elif PITCH_POSITION_USE_IMU == 0
    gravity_theta = (GM6020[PITCH].Position.Real - PITCH_POSITION_INIT) * NORMAL_PI / 4096.0f;
   #endif
	
	 gravity_costheta = cos(gravity_theta);
	 gravity_Compensate = gravity_moment * gravity_costheta;
	
	return gravity_Compensate;
}

/**
  * @brief  一维卡尔曼滤波器
  * @param  input:6020编码器输入值
  * @retval 最优估计值
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void Gimbal_FirstOrder_KalmanFilter(void)
{
    Kalman[PITCH].Position.Step = GM6020[PITCH].Position.Real;
		Kalman[PITCH].Position.Real = FirstOrder_KalmanFilter_Cacl(&Gimbal_Position_Kalman[PITCH],Kalman[PITCH].Position.Step,0);
    Kalman[PITCH].Position.Error = GM6020[PITCH].Position.Set - Kalman[PITCH].Position.Real;
		
		Kalman[PITCH].Speed.Step = GM6020[PITCH].Speed.Real;
		Kalman[PITCH].Speed.Real = FirstOrder_KalmanFilter_Cacl(&Gimbal_Speed_Kalman[PITCH],Kalman[PITCH].Speed.Step,0);
		Kalman[PITCH].Speed.Error = GM6020[PITCH].Speed.Set - Kalman[PITCH].Speed.Real;
}

/**
  * @brief  云台动态限幅
  * @param  input:陀螺仪输入值
  * @param  encoder:编码器反馈数值
  * @param  AutoLimit_t:动态限幅结构体 
  * @retval 状态值
  */
void Gimbal_Dynamic_Limit(s16 *input,u16 encoder,u16 imu,AutoLimit_t *autolimit)//这里主要就是过零处理，自己理清逻辑即可
{
	autolimit->Dynamic_Min = Func_ValueRannge(imu - encoder + autolimit->Min,8192,0);
	autolimit->Dynamic_Max = Func_ValueRannge(imu + autolimit->Max - encoder,8192,0);
	if(autolimit->Dynamic_Max > autolimit->Dynamic_Min)
		*input = (s16)Func_Ramp_Limit(*input,autolimit->Dynamic_Max,autolimit->Dynamic_Min,0); //这个功能函数 编码器数值区分大小
	else 
		*input = (s16)Func_Ramp_Limit(*input,autolimit->Dynamic_Max,autolimit->Dynamic_Min,1);
}


//处理接收到的miniPC数据
 void Vision_Control_Gimbal(void)
{
	//将当前姿态存入卡尔曼滤波器，以备使用
	//因上位机处理的图像都是几十毫秒以前的数据，需要将以前的姿态数据拿来使用
	//这样会更精准一些
#if YAW_USE_IMUEX == 1 
	KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1] =  Imuex.Angle.Yaw;
#elif YAW_USE_IMUEX == 0 
	KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1] =  imu.yaw;
#endif
	
#if PITCH_POSITION_USE_IMU == 1 
	KF.Pitch_angle[KALMAN_FILTER_PAST_TIME- 1 ] = Imuex.Angle.Pitch;
#elif PITCH_POSITION_USE_IMU == 0 
	KF.Pitch_angle[KALMAN_FILTER_PAST_TIME- 1 ] = GM6020[PITCH].Position.Real;
#endif
	for(u8 i = 0;i < KALMAN_FILTER_PAST_TIME - 1;i++)//对姿态数据进行一次迭代
	{
		KF.Yaw_angle[i]   = KF.Yaw_angle[i+1];
		KF.Pitch_angle[i] = KF.Pitch_angle[i+1];
		//KF.Speed[i]       = KF.Speed[i+1];		
	}

	kalman_filter_calc(&KF);//卡尔曼滤波器进行及结算
	if(Command.AimAssitFlag == AUTO_ATTACK )//判定是否跟踪（就是是否开启了自动瞄准），装甲片和大符使用的是同一套控制机制，只不过大符没有x向预测补偿
	{
//打符补偿
//		GM6020[YAW].Position.Set   = Yaw_Predict(KF.filtered_value[1],KF.filtered_value[0],Shooting.Speed);
		GM6020[YAW].Position.Set   = KF.filtered_value[0]+ yawc ;//减小右转，增大左转
		GM6020[PITCH].Position.Set = Bullet_Amend(KF.filtered_value[1],Shooting.Speed,TX1_Data.Distance) + pitc;	//减小数值抬头，增大数值低�-380�
//打装甲板补偿
		
	}
	else
	{
		KF.Kalman_Filter_Yaw        = KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1];
		KF.Kalman_Filter_Pitch      = KF.Pitch_angle[KALMAN_FILTER_PAST_TIME - 1];
		KF.Kalman_Filter_Speed      = 0;
		KF.Kalman_Filter_CenterAngle= KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1];
	}
}




s16 Bullet_Amend(float angle,int bullet_Speed,u16 distance)//速度单位是mm/s，对pitch轴子弹进行一个抛物线逆解算
{
//忽略空气阻力，解算子弹补偿
	float theta = 0,sintheta = 0,temp = 0;
	s16 pitch_angle;
	
	if(bullet_Speed == 0)
		return angle;
	
	theta = (PITCH_POSITION_INIT - angle) * NORMAL_PI / 4096.0f;
	sintheta = sin(theta);
	temp = (9800 * distance * (1 - sintheta * sintheta) / (bullet_Speed * bullet_Speed) + sintheta);
	pitch_angle = PITCH_POSITION_INIT - 0.5 * (asin(temp) + theta) * 4096 / NORMAL_PI;
	return pitch_angle;
}



//TX1_Data_t数据中的角度值都是0-8191
s16 Yaw_Predict(TX1_Data_t *data,float angle,int bullet_Speed)
{ 
	//yaw轴速度预测，主要用于角度补偿，补偿的参量为子弹发射延迟与子弹飞行时间，实测发射延迟为100-120ms
	s16 yaw_predict;
	if(bullet_Speed == 0)
		return angle;
	yaw_predict = angle - atan2(KF.filtered_value[2] * (0.12f + data->Distance / bullet_Speed),data->Distance) * 4096 / NORMAL_PI;
	return yaw_predict;
}

//遥控器右侧拨杆处于中位时云台失能，停止云台控制，对数据复位
void Gimbal_Disable(void)
{
	#if YAW_USE_IMUEX == 1
		GM6020[YAW].Position.Set   = Imuex.Angle.Yaw;
		GM6020[YAW].Position.Step  = Imuex.Angle.Yaw;
	#elif YAW_USE_IMUEX == 0
		GM6020[YAW].Position.Set   = imu.yaw;
		GM6020[YAW].Position.Step  = imu.yaw;
	#endif
		
	#if PITCH_POSITION_USE_IMU == 1                    
		GM6020[PITCH].Position.Set  = Imuex.Angle.Pitch;
		GM6020[PITCH].Position.Step = Imuex.Angle.Pitch;
	#elif PITCH_POSITION_USE_IMU == 0
		GM6020[PITCH].Position.Set  = GM6020[PITCH].Position.Real;
		GM6020[PITCH].Position.Step = GM6020[PITCH].Position.Real;
	#endif
		PID_Clear(&Gimbal_Position_pid[PITCH]);
		PID_Clear(&Gimbal_Speed_pid[PITCH]);
		PID_Clear(&Pitch_Stable_pid);
		
		Command.Mouse_x = imu.yaw;
		Command.Mouse_y = PITCH_POSITION_INIT;
}



//u8 data[5] = {0xA5,0,0};
//#define TwoPI 6.2831853071f
//u8 Signalflag = 0;
//static s16 temp = 0;
//static u32 time = 0;
//static u8 count = 0;
//static s16 test;
//const u16 period[64] = {1000,667,500,400,333,286,250,222,200,182,167,154,143,133,125,118,111,105,100,95,91,87,83,80,77,74,71,69,67,65,63,61,59,57,56,54,53,51,50,49,48,47,45,42,38,36,33,31,29,28,26,25,20,17,14,13,11,10,9,8,5,4,3,2};
//s16 Signal_Generator(void)
//{
//	s16 output;
//	time+=1;
//	output = 500 * sin((double)(TwoPI * time) /period[count]);
//	if(time%(period[count]*20)==0)
//	{

//		count++;
//		time = 0;
//		if(count == 64)
//		{
//			Signalflag = 2;
//			count = 0;
//		}
//	}
//	return output;
//}	


//		if(RC_Ctl.rc.s1 == 2)
//			Signalflag = 1;
//		if(Signalflag == 1)
//		{
//			test = Signal_Generator();
//			temp = GM6020[PITCH].Position.Real - 2900;

//			data[0] = temp >> 8;
//			data[1] = temp;
//			//data[0] = test  >> 8;
//			//data[1] = test;
//			while(HAL_UART_Transmit_DMA(&huart3,data,2));
//		}
//		




