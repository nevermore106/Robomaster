/**
  ******************************************************************************
  * @file    powerlimit.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/6
  * @brief   功率限制
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

PowerLimit_t PowerLimit;
/**
  * @brief  功率限制处理
	*         在正常情况下功率一直给定在75W，默认平地移动可以达到最大转速
	*         按下Shift则解除功率限制，在爬坡的时候使用。
  * @param  None
  * @retval None
  */
void Power_Control(void)
{
	u32 sum_speed = 0;
	u8 count = 0;
	Power_Deal();//防止裁判数据失真处理

	if(PowerLimit.Flag == 0)
	{	
		Power_PID_Calc(&Power_Limit_pid,PowerLimit.Real_Power[2],75);//反馈的功率超过75W之后PI控制器开始工作
		PowerLimit.MaxSpeed = CHASSIS_MAX_SPEED * (1 + Power_Limit_pid.Output);
		PowerLimit.MaxSpeed = Func_Limit(PowerLimit.MaxSpeed,CHASSIS_MAX_SPEED,0);//超过75W之后限制最大轮速，以此降低功率值
	}
	else if(PowerLimit.RemainPower[2] >= 30 && PowerLimit.Restore == 0)//手动解除功率限制，在超过30J能量时，不限制最大速度
	{
		PowerLimit.MaxSpeed=CHASSIS_MAX_SPEED;
	}
	else//低于30J能量时，对最大轮速进行限制，速度与实时功率有关
	{
		for(count = 0;count < 4;count++)
			sum_speed += Func_Abs(M3508[count].Speed.Real);
		PowerLimit.MaxSpeed = sum_speed * 17.5f / PowerLimit.Real_Power[2];
		PowerLimit.MaxSpeed = Func_Limit(PowerLimit.MaxSpeed,CHASSIS_MAX_SPEED,0);
		PowerLimit.Restore = 1;
	}
	
			for(count = 0;count < 4;count++)
			sum_speed += Func_Abs(M3508[count].Speed.Real);
	if(PowerLimit.RemainPower[2] <= 10)//当能量槽只有10J以下时，强制切断输出，以防扣血
	{
		PowerLimit.Cut = ON;
		Command.SpeedZoom = 0;
	}
	else if(PowerLimit.RemainPower[2] >= 40 && RC_Ctl.rc.s2 != 3)//能量槽恢复40J之后，恢复底盘动力
	{
		PowerLimit.Cut = OFF;
		PowerLimit.Restore = 0;
		if(RC_Ctl.rc.s2 == 1)
			Command.SpeedZoom = 1;
	}
}


/**
  * @brief  裁判系统能量数据处理
  * @param  None
  * @retval None
  */
void Power_Deal(void)
{
	if(InfantryJudge.ErrorFlag != ERROR)//保证频率为50Hz，对数据进行更新
	{
		PowerLimit.Real_Power[2]  = InfantryJudge.RealPower;
		PowerLimit.RemainPower[2] = InfantryJudge.RemainPower;
	}
	else//数据丢失之后，预测当前功率 
	{
		PowerLimit.Real_Power[2]  = (2*PowerLimit.Real_Power[1]  - PowerLimit.Real_Power[0])  * 1.3f;//数据帧丢失，通过微分关系预测此时功率，为了保证预测功率小于实际功率，取安全系数1.3
		PowerLimit.RemainPower[2] = (2*PowerLimit.RemainPower[1] - PowerLimit.RemainPower[0]) / 1.3f;
		InfantryJudge.ErrorFlag   = NORMAL;
	}
	//能量数据迭代
	PowerLimit.Real_Power[1]  = PowerLimit.Real_Power[2];
	PowerLimit.Real_Power[0]  = PowerLimit.Real_Power[1];
	
	PowerLimit.RemainPower[1] = PowerLimit.RemainPower[2];
	PowerLimit.RemainPower[0] = PowerLimit.RemainPower[1];
}




/**
  * @brief  这个函数是用来防止某个轮子的速度超过最大的给定速度
	*         为了使运动轨迹保持和操控的方向一样
  *         如果有一个轮子超过了最大速度值，等比例缩放四个轮子的速度
  * @param  None
  * @retval 直接比例缩放了给定速度值
  */
void Direction_Keep(void)
{ 
	if(Command.RotateFlag == 1)
	 {
		PowerLimit.MaxSpeed = CHASSIS_MAX_SPEED - SpinTop.Speed;
	  for(u8 i = 0;i < 4;i++)
   	 {
	    M3508[i].Speed.Set -= SpinTop.Speed;
  	 }
	 }
	else
	  PowerLimit.MaxSpeed = CHASSIS_MAX_SPEED;;
	
	for(PowerLimit.ZoomCount = 0;PowerLimit.ZoomCount < 4;PowerLimit.ZoomCount++)
	{
		if(Func_Abs(M3508[PowerLimit.ZoomCount].Speed.Set) > PowerLimit.MaxSpeed)//判断某个轮子是否超过最大轮速
		{
			PowerLimit.ZoomTemp = Func_Abs(M3508[PowerLimit.ZoomCount].Speed.Set) / PowerLimit.MaxSpeed;//超过了最大轮速的多少
			if(PowerLimit.ZoomTemp > PowerLimit.Zoom)
				PowerLimit.Zoom = PowerLimit.ZoomTemp;//记录超过最大轮速的轮子
		}
	}
	if(PowerLimit.Zoom != 1)//如果超过最大轮速了		
	{
		for(PowerLimit.ZoomCount = 0;PowerLimit.ZoomCount < 4;PowerLimit.ZoomCount++)
		{
			M3508[PowerLimit.ZoomCount].Speed.Set = M3508[PowerLimit.ZoomCount].Speed.Set / PowerLimit.Zoom;//对每个轮子的设定速度进行等比例缩放
		}
		PowerLimit.ZoomTemp = PowerLimit.Zoom = 1;
	}
	
	if(Command.RotateFlag == 1)
	 {
	  for(u8 i = 0;i < 4;i++)
   	 {
	    M3508[i].Speed.Set += SpinTop.Speed;
  	 }
	 }
}


/**
  * @brief  功率PID计算，因为和电机的不一样，故单独写了一个
  * @param  输入、反馈
  * @retval None
  */
void Power_PID_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float p = 0,//设定计算的PID值
				i = 0,
				d = 0;
	
	pid->E[2]=Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//当前误差	
	pid->Intergral =pid->Intergral+ (pid->Ki) * (pid->E[2]);//积分值
	pid->Intergral=Func_Limit(pid->Intergral,0,-(pid->I_max_out));//积分限幅
	p = pid->Kp * (pid->E[2]);
	i = pid->Intergral;
	d = pid->Kd * (pid->E[2]-pid->E[1]);
	pid->D_last=d;
	pid->Output=p+i+d;		
//是否超出最大输出
		pid->Output=Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));
	/*迭代误差*/
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

