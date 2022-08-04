/**
  ******************************************************************************
  * @file    chassis_task.c
  * @author  北京工业大学-张曦梁
  * @version V2.0
  * @date    2021/10/3
  * @brief   底盘进程
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
Motor_t M3508[4]={0};
Motor_t Chassis={0};
M3508_Ramp_t M3508_Ramp={0};
SpinTop_t SpinTop={0};
//s16 last_aver = 0;
//s16 now_aver = 0;
//s16 angle_error=0;

int16_t Yaw_Position_Move = 0;

void Chassis_Task(void const * argument)
{ 
	u8 pidCount = 0;
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
		osDelayUntil(&xLastWakeTime,4);//250HZ
		Imu_Temperature_Compensate();//温度闭环,不用太高频，就没放云台任务里

//计算旋转速度
		if(Command.RotateFlag == 0)
			Chassis_Follow_PD_Calc(&Chassis_Rotate_pid,GM6020[YAW].Position.Real,Func_ValueRannge(YAW_POSITION_INIT + Yaw_Position_Move,8192,0));//转向闭环计算

		else
			Chassis_Rotate_pid.Output = SpinTop.Speed ;//3800-42W
  
//计算陀螺分解速度
//将云台坐标系转换为底盘坐标系，对四个轮子进行速度调整，以达到小陀螺的全向移动
//因为电机执行需要时间，而此时机器人还在旋转，所以需要对牵引力的方向提前做一个补偿，暂且认为牵引力与速度有关
		SpinTop.Angle.Transmit  = (GM6020[YAW].Position.Real - YAW_POSITION_INIT + 8192) % 8192 / 22.7555556f;
		SpinTop.Vx.Real  = (M3508[0].Speed.Real - M3508[1].Speed.Real + M3508[2].Speed.Real - M3508[3].Speed.Real)/4;
		SpinTop.Vy.Real  = (M3508[0].Speed.Real + M3508[1].Speed.Real - M3508[2].Speed.Real - M3508[3].Speed.Real)/4;
		SpinTop.Angle.Set = (int)(atan2(Command.Vx,Command.Vy)   * 57.2957795f + (int)SpinTop.Angle.Transmit) % 360;
		SpinTop.Angle.Real = (int) atan2(SpinTop.Vx.Real,SpinTop.Vy.Real) * 57.2957795f;
    
		if(Command.RotateFlag == 1)
		{
			if(SpinTop.Speed == ROTATE_NORMAL_SPEED)
			  SpinTop.Angle.TractionCompensate = 11 + Func_Limit(sqrt(SpinTop.Vx.Real*SpinTop.Vx.Real + SpinTop.Vy.Real*SpinTop.Vy.Real) * 0.0135f,33,0);	
//			else if(SpinTop.Speed == ROTATE_LOW_SPEED)
//				SpinTop.Angle.TractionCompensate = 6 + Func_Limit(sqrt(SpinTop.Vx.Real*SpinTop.Vx.Real + SpinTop.Vy.Real*SpinTop.Vy.Real) * 0.01f,32,0);
			else if(SpinTop.Speed == ROTATE_HIGH_SPEED)
				SpinTop.Angle.TractionCompensate = 33 + Func_Limit(sqrt(SpinTop.Vx.Real*SpinTop.Vx.Real + SpinTop.Vy.Real*SpinTop.Vy.Real) * 0.018f,37,0);
			SpinTop.Angle.Transmit += SpinTop.Angle.TractionCompensate;
//			now_aver = weightAverageFilter(SpinTop.Vx.Real);
//			angle_error = now_aver - last_aver;
//			last_aver = now_aver;
		}

		if(SpinTop.Angle.Transmit > 360)
			SpinTop.Angle.Transmit -= 360;
		SpinTop.Angle.Transmit *= 0.0174532f;
		SpinTop.Vx.Set =   Command.Vx * cos(SpinTop.Angle.Transmit) + Command.Vy * sin(SpinTop.Angle.Transmit);
		SpinTop.Vy.Set =  -Command.Vx * sin(SpinTop.Angle.Transmit) + Command.Vy * cos(SpinTop.Angle.Transmit);
//		SpinTop.Vx.LastReal = SpinTop.Vx.Real;
//    SpinTop.Vy.LastReal =	SpinTop.Vy.Real;
//计算各轮速度，逆向运动学分解
		M3508[0].Speed.Set = ( SpinTop.Vx.Set + SpinTop.Vy.Set + Chassis_Rotate_pid.Output) * Command.SpeedZoom * Command.SpeedMinish;
		M3508[1].Speed.Set = (-SpinTop.Vx.Set + SpinTop.Vy.Set + Chassis_Rotate_pid.Output) * Command.SpeedZoom * Command.SpeedMinish;
		M3508[2].Speed.Set = ( SpinTop.Vx.Set - SpinTop.Vy.Set + Chassis_Rotate_pid.Output) * Command.SpeedZoom * Command.SpeedMinish;
		M3508[3].Speed.Set = (-SpinTop.Vx.Set - SpinTop.Vy.Set + Chassis_Rotate_pid.Output) * Command.SpeedZoom * Command.SpeedMinish;

		Power_Control();//功率限制函数
		
		Direction_Keep();//矢量方向保持 同时功率限制作用在这个函数中
		
		M3508Ramp_Self_Deal();//增量自处理函数
		
//电机闭环输出计算	
		for(pidCount = 0;pidCount < 4;pidCount++)
		{
			Func_Ramp(M3508[pidCount].Speed.Set,&M3508[pidCount].Speed.Step,M3508_Ramp.Increment[pidCount]);
			PID_Calc(&M3508_Speed_pid[pidCount],M3508[pidCount].Speed.Real,M3508[pidCount].Speed.Step);//转向电机带斜坡启动
		}
//输出电流	
		Chassis_Power_On() ? M3508_Output(0,0,0,0) : M3508_Output(M3508_Speed_pid[0].Output,M3508_Speed_pid[1].Output,M3508_Speed_pid[2].Output,M3508_Speed_pid[3].Output);
//		M3508_Output(0,0,0,0);
  }
}


/**
  * @brief  判断底盘电源是否开启输出
  * @param  None 
  * @retval 开关量
  */
s8 Chassis_Power_On(void)
{
	if((PowerLimit.Cut) || (RC_Ctl.rc.s2 == 3) || (Command.GimbalFollowOFF == 1))
		return 1;
	else
		return 0;
}


/**
  * @brief  扭腰控制
  * @param  None
  * @retval 扭腰要输出的位置值
  */
//s16 Twist_Control(void)
//{
//	static u8 flag = 0;
//	if(Command.RotateFlag == 1)
//	{
//		if(flag == 0)
//		{
//			Chassis.Position.Set = TWIST_RANGE;
//			flag = 1;
//		}
//		if(YAW_POSITION_INIT - GM6020[YAW].Position.Real > TWIST_RANGE * 0.9)
//			Chassis.Position.Set = TWIST_RANGE;
//		else if(GM6020[YAW].Position.Real - YAW_POSITION_INIT > TWIST_RANGE * 0.9)
//			Chassis.Position.Set = -TWIST_RANGE;
//	}
//	else if(Command.RotateFlag == 0) 
//	{
//		Chassis.Position.Set = 0;
//		flag = 0;
//	}
//	Func_Ramp(Chassis.Position.Set,&Chassis.Position.Step,TWIST_RAMP_RATIO);
//	return Chassis.Position.Step;
//}




//3508加速度处理函数，通过四个轮子的当前速度与目标速度解算加速度值，同时根据当前转速大小调整加速度
void M3508Ramp_Self_Deal(void)
{
	s16   max_mis  = 0;
	float max_ramp[4];
	u8    count     = 0;
	
	
	for(count = 0;count < 4;count++)
	{
		M3508_Ramp.Mis[count] = Func_Abs(M3508[count].Speed.Set - M3508[count].Speed.Real);//计算单个轮误差绝对值
		max_ramp[count] =  Func_Abs(M3508[count].Speed.Set - M3508[count].Speed.Step)/80 + M3508_RAMP_RATIO_MIN;//计算单个轮预期斜坡增量
	}
	
	for(count = 0;count < 4;count++)//计算最大误差
	{
		if(max_mis < M3508_Ramp.Mis[count])
			max_mis = M3508_Ramp.Mis[count];
	}
	
	for(count = 0;count < 4;count++)//计算最终斜坡
		{
			M3508_Ramp.Increment[count] = max_ramp[count] * M3508_Ramp.Mis[count] / max_mis;
//			M3508_Ramp.Increment[count] = Func_Limit(M3508_Ramp.Increment[count],M3508_RAMP_RATIO_MAX,M3508_RAMP_RATIO_MIN);
		}
}

void Imu_Temperature_Compensate(void)
{
	if((s16)imu.temp < 45)
		TIM3->CCR2 = 999;
	else
	  TIM3->CCR2 = 0;
}

/*
//计算旋转速度
		if(Command.RotateFlag == 0)
			PID_Calc(&Chassis_Rotate_pid,GM6020[YAW].Position.Real,YAW_POSITION_INIT);//转向闭环计算0-
		else
			Chassis_Rotate_pid.Output = 4000;

//计算陀螺分解速度
		SpinTop.Angle  = (GM6020[YAW].Position.Real - YAW_POSITION_INIT + 8192) % 8192 / 22.7555556f;
		SpinTop.RVx    = (M3508[0].Speed.Real - M3508[1].Speed.Real + M3508[2].Speed.Real - M3508[3].Speed.Real) / 150;
		SpinTop.RVy    = (M3508[0].Speed.Real + M3508[1].Speed.Real - M3508[2].Speed.Real - M3508[3].Speed.Real) / 150;
		SpinTop.SAngle = ((int)(atan2(Command.Vx,Command.Vy)   * 57.2957795f + 270) % 360 + (int)SpinTop.Angle) % 360;
		SpinTop.RAngle =  (int)(atan2(SpinTop.RVx,SpinTop.RVy) * 57.2957795f + 270) % 360;

		if(Command.RotateFlag == 0)
			SpinTop.AngleCompensate = 0;
		else
		{			
			PID_Calc(&Spin_Top_pid,SpinTop.SAngle,SpinTop.RAngle);		
			SpinTop.AngleCompensate = Spin_Top_pid.Output;
		}
		SpinTop.Angle = SpinTop.Angle + SpinTop.AngleCompensate;
		if(SpinTop.Angle > 360)
			SpinTop.Angle = (SpinTop.Angle - 360) * 0.0174532f;
		else
			SpinTop.Angle *= 0.0174532f;
		SpinTop.Vx =  Command.Vx * cos(SpinTop.Angle) + Command.Vy * sin(SpinTop.Angle);
		SpinTop.Vy = -Command.Vx * sin(SpinTop.Angle) + Command.Vy * cos(SpinTop.Angle);
*/

