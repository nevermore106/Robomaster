/**
  ******************************************************************************
  * @file    powerlimit.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/6
  * @brief   ��������
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

PowerLimit_t PowerLimit;
/**
  * @brief  �������ƴ���
	*         ����������¹���һֱ������75W��Ĭ��ƽ���ƶ����Դﵽ���ת��
	*         ����Shift�����������ƣ������µ�ʱ��ʹ�á�
  * @param  None
  * @retval None
  */
void Power_Control(void)
{
	u32 sum_speed = 0;
	u8 count = 0;
	Power_Deal();//��ֹ��������ʧ�洦��

	if(PowerLimit.Flag == 0)
	{	
		Power_PID_Calc(&Power_Limit_pid,PowerLimit.Real_Power[2],75);//�����Ĺ��ʳ���75W֮��PI��������ʼ����
		PowerLimit.MaxSpeed = CHASSIS_MAX_SPEED * (1 + Power_Limit_pid.Output);
		PowerLimit.MaxSpeed = Func_Limit(PowerLimit.MaxSpeed,CHASSIS_MAX_SPEED,0);//����75W֮������������٣��Դ˽��͹���ֵ
	}
	else if(PowerLimit.RemainPower[2] >= 30 && PowerLimit.Restore == 0)//�ֶ�����������ƣ��ڳ���30J����ʱ������������ٶ�
	{
		PowerLimit.MaxSpeed=CHASSIS_MAX_SPEED;
	}
	else//����30J����ʱ����������ٽ������ƣ��ٶ���ʵʱ�����й�
	{
		for(count = 0;count < 4;count++)
			sum_speed += Func_Abs(M3508[count].Speed.Real);
		PowerLimit.MaxSpeed = sum_speed * 17.5f / PowerLimit.Real_Power[2];
		PowerLimit.MaxSpeed = Func_Limit(PowerLimit.MaxSpeed,CHASSIS_MAX_SPEED,0);
		PowerLimit.Restore = 1;
	}
	
			for(count = 0;count < 4;count++)
			sum_speed += Func_Abs(M3508[count].Speed.Real);
	if(PowerLimit.RemainPower[2] <= 10)//��������ֻ��10J����ʱ��ǿ���ж�������Է���Ѫ
	{
		PowerLimit.Cut = ON;
		Command.SpeedZoom = 0;
	}
	else if(PowerLimit.RemainPower[2] >= 40 && RC_Ctl.rc.s2 != 3)//�����ۻָ�40J֮�󣬻ָ����̶���
	{
		PowerLimit.Cut = OFF;
		PowerLimit.Restore = 0;
		if(RC_Ctl.rc.s2 == 1)
			Command.SpeedZoom = 1;
	}
}


/**
  * @brief  ����ϵͳ�������ݴ���
  * @param  None
  * @retval None
  */
void Power_Deal(void)
{
	if(InfantryJudge.ErrorFlag != ERROR)//��֤Ƶ��Ϊ50Hz�������ݽ��и���
	{
		PowerLimit.Real_Power[2]  = InfantryJudge.RealPower;
		PowerLimit.RemainPower[2] = InfantryJudge.RemainPower;
	}
	else//���ݶ�ʧ֮��Ԥ�⵱ǰ���� 
	{
		PowerLimit.Real_Power[2]  = (2*PowerLimit.Real_Power[1]  - PowerLimit.Real_Power[0])  * 1.3f;//����֡��ʧ��ͨ��΢�ֹ�ϵԤ���ʱ���ʣ�Ϊ�˱�֤Ԥ�⹦��С��ʵ�ʹ��ʣ�ȡ��ȫϵ��1.3
		PowerLimit.RemainPower[2] = (2*PowerLimit.RemainPower[1] - PowerLimit.RemainPower[0]) / 1.3f;
		InfantryJudge.ErrorFlag   = NORMAL;
	}
	//�������ݵ���
	PowerLimit.Real_Power[1]  = PowerLimit.Real_Power[2];
	PowerLimit.Real_Power[0]  = PowerLimit.Real_Power[1];
	
	PowerLimit.RemainPower[1] = PowerLimit.RemainPower[2];
	PowerLimit.RemainPower[0] = PowerLimit.RemainPower[1];
}




/**
  * @brief  ���������������ֹĳ�����ӵ��ٶȳ������ĸ����ٶ�
	*         Ϊ��ʹ�˶��켣���ֺͲٿصķ���һ��
  *         �����һ�����ӳ���������ٶ�ֵ���ȱ��������ĸ����ӵ��ٶ�
  * @param  None
  * @retval ֱ�ӱ��������˸����ٶ�ֵ
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
		if(Func_Abs(M3508[PowerLimit.ZoomCount].Speed.Set) > PowerLimit.MaxSpeed)//�ж�ĳ�������Ƿ񳬹��������
		{
			PowerLimit.ZoomTemp = Func_Abs(M3508[PowerLimit.ZoomCount].Speed.Set) / PowerLimit.MaxSpeed;//������������ٵĶ���
			if(PowerLimit.ZoomTemp > PowerLimit.Zoom)
				PowerLimit.Zoom = PowerLimit.ZoomTemp;//��¼����������ٵ�����
		}
	}
	if(PowerLimit.Zoom != 1)//����������������		
	{
		for(PowerLimit.ZoomCount = 0;PowerLimit.ZoomCount < 4;PowerLimit.ZoomCount++)
		{
			M3508[PowerLimit.ZoomCount].Speed.Set = M3508[PowerLimit.ZoomCount].Speed.Set / PowerLimit.Zoom;//��ÿ�����ӵ��趨�ٶȽ��еȱ�������
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
  * @brief  ����PID���㣬��Ϊ�͵���Ĳ�һ�����ʵ���д��һ��
  * @param  ���롢����
  * @retval None
  */
void Power_PID_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float p = 0,//�趨�����PIDֵ
				i = 0,
				d = 0;
	
	pid->E[2]=Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//��ǰ���	
	pid->Intergral =pid->Intergral+ (pid->Ki) * (pid->E[2]);//����ֵ
	pid->Intergral=Func_Limit(pid->Intergral,0,-(pid->I_max_out));//�����޷�
	p = pid->Kp * (pid->E[2]);
	i = pid->Intergral;
	d = pid->Kd * (pid->E[2]-pid->E[1]);
	pid->D_last=d;
	pid->Output=p+i+d;		
//�Ƿ񳬳�������
		pid->Output=Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));
	/*�������*/
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

