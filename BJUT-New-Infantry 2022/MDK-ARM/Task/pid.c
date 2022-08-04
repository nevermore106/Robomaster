/**
  ******************************************************************************
  * @file    pid.c
  * @author  Tinker.Jia
  * @version V1.1
  * @date    2018/11/12
  * @brief   PID���㺯��
  ******************************************************************************
  * @attention
  *	V1.1 ɾ����PID�еĹ��㴦������PID�����������Ż�
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
PidTypeDef M3508_Speed_pid[4]={0};
PidTypeDef Gimbal_Speed_pid[3]={0};
PidTypeDef Gimbal_Position_pid[3]={0};
PidTypeDef Gimbal_TCurrent_pid[3]={0};
PidTypeDef Chassis_Rotate_pid={0};
PidTypeDef Power_Limit_pid={0};
PidTypeDef Spin_Top_pid={0};
PidTypeDef Pitch_Stable_pid={0};
/**
  * @brief  PID�����ĳ�ʼ��
  * @param  ����̫���ˣ�����һЩ��������
  * @retval None
  */
void PID_Init(PidTypeDef * pid,float kp,float ki,float kd,float max_out,float dead_band,float i_band,float max_input,float i_max_out,u8 mode)
{
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	pid->Max_out=max_out;
	pid->I_max_out=i_max_out;
	pid->Dead_band=dead_band;
	pid->Intergral_band=i_band;
	pid->Max_input=max_input;
	//PID���ֵ
	pid->Input=0;
	pid->Output=0;
	pid->Intergral=0;
	pid->Mode=mode;
	//����ʼ��
	pid->E[0]=0;
	pid->E[1]=0;
	pid->E[2]=0;//2���� 1��һ�� 0���ϴ�
	pid->D_last=0;
}


/**
  * @brief  PID����
  * @param  ���롢����
  * @retval None
  */
void PID_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float p = 0,//�趨�����PIDֵ
				i = 0,
				d = 0;
	pid->E[2]=Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//��ǰ���	
	
	if(pid==&Gimbal_Position_pid[0]||pid==&Gimbal_Position_pid[1]||pid==&Gimbal_Position_pid[2])//���㴦��������ôת�������ӻ�����Ҫ�������Ϊ����������
	{
		if(pid->E[2]>4096)
			pid->E[2]=pid->E[2]-8192;
		else if((-pid->E[2])>4096)
			pid->E[2]=8192+pid->E[2];
	}
	if(pid == &Spin_Top_pid)
	{
		if(pid->E[2]>180)
			pid->E[2]=pid->E[2]-360;
		else if((-pid->E[2])>180)
			pid->E[2]=360+pid->E[2];
	}

	if(Func_Abs(pid->E[2]) >=  pid->Dead_band)//��ƫ��ֵ���ڵ�������ֵ���������if
	{
		if(pid->Mode==Positional)
		{	
			if(Func_Abs(pid->E[2]) <= pid->Intergral_band)             //��ƫ��С�ڻ��ַ�Χ��ʱ�򣬽������  ע�⣡ֻ��λ��ʽPID�л��ַ�Χ
//				pid->Intergral =pid->Intergral+ (pid->Ki) * (pid->E[2]);//����ֵ
			 pid->Intergral =pid->Intergral+ ((pid->Ki) * (pid->E[2]) + (pid->Ki) * (pid->E[1]))/2;//���λ���
			else
				{pid->Intergral=pid->Intergral;}                
			pid->Intergral=Func_Limit(pid->Intergral,pid->I_max_out,-(pid->I_max_out));//�����޷�			
  		p = pid->Kp * (pid->E[2]);
			i = pid->Intergral;
			d = pid->Kd * (pid->E[2]-pid->E[1]);
			pid->D_last=d;
			pid->Output=p+i+d;
		}
		else if(pid->Mode==Incremental)
		{
			p=pid->Kp*((pid->E[2]-pid->E[1]));
			i=pid->Ki*pid->E[2];
			d=pid->Kd*(pid->E[2]-pid->E[1]*2+pid->E[0]);
			pid->Output+=p+i+d;
		}
//YAW��PITCH����ǰ������
		if(pid == &Gimbal_Speed_pid[0])
			pid->Output += Yaw_Feedforward_Controler(Gimbal_Position_pid[YAW].Output);
//		if(pid == &Gimbal_Speed_pid[1])
//			pid->Output += Pitch_Feedforward_Controler(Gimbal_Position_pid[PITCH].Output);
//�Ƿ񳬳�������
		pid->Output=Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));

	}
	else
		pid->Output=0;	
	/*�������*/
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

/**
  * @brief  ���̸���PD����
  * @param  ���롢����
  * @retval None
  */
void Chassis_Follow_PD_Calc(PidTypeDef * pid, float real_val, float set_val)
{
	float Abs_Error = 0,
				p = 0,//�趨�����PIDֵ
				d = 0;

	pid->E[2] = Func_Limit(set_val,pid->Max_input,-(pid->Max_input))-real_val;//��ǰ���	
	if(pid->E[2]>4096)
		pid->E[2]=pid->E[2]-8192;
	else if((-pid->E[2])>4096)
		pid->E[2]=8192+pid->E[2];
	Abs_Error = Func_Abs(pid->E[2]);
	if(Abs_Error >  3300)
	{
		pid->Kp = 6;
		pid->Kd = 0;
	}
	else if(Abs_Error >  2200)
	{
		pid->Kp = 5.5;
		pid->Kd = 150;
	}
	else if(Abs_Error >  1300)
	{
		pid->Kp = 5;
		pid->Kd = 200;
	}
	else if(Abs_Error >  550)
	{
		pid->Kp = 5;
		pid->Kd = 220;
	}
	else 
	{
		pid->Kp = 4.5f;			
		pid->Kd = 220;
	}			
						
	p = pid->Kp * (pid->E[2]);
	d = pid->Kd * (pid->E[2]-pid->E[1]);
	pid->Output = p+d;
	pid->Output = Func_Limit(pid->Output,pid->Max_out,-(pid->Max_out));
	
	pid->E[0] = pid->E[1];
	pid->E[1] = pid->E[2];
}

/**
  * @brief  PID�������
  * @param  Ҫ��յĵ�ַ
  * @retval None
  */
void PID_Clear(PidTypeDef * pid)
{
	pid->E[0] = 0;
	pid->E[1] = 0;
	pid->E[2] = 0;
	pid->Intergral = 0;
	pid->Output = 0;
}


//YAW����ǰ����������
//��ʵ����д��һ��ģ�����ʱ��ȽϽ��ȣ���û�кϲ�����ʱ��Ľ�һ�¡�
//ͨ��ϵͳ��ʶ����̨��ģ��Ȼ����䲹��
s16 Yaw_Feedforward_Controler(s16 set)
{
	s16 output;
 /* nΪ���ݺ������ӷ�ĸ��߽״� */
	const uint8_t n = 4;
//����num Ҫע��ǰ�����������ķ��Ӻͷ�ĸ�Ǵ��ݺ����ĵ���
	//const float b[n + 1] = {1.0000, -2.880214963923001, 2.761097200406337, -0.880881688211085};
	const float b[n + 1] = {1.0000, -3.893807793087997, 5.686900214928543, -3.692246693791884,0.899154974898897};
/* ��ĸden */
	//const float a[n + 1] = {-3.644641836338100e-05, 5.655192265084034e-05, 3.644641836338100e-05, -5.655192265084034e-05};
	const float a[n + 1] = {2.614389083511688e-05, -4.583097707295865e-05, -6.103729090941078e-06, 4.584276961172979e-05,-2.002836920540466e-05};
	static float x[n + 1] = {0};
	static float y[n + 1] = {0};
	
	for (uint8_t i = n; i > 0; i--)
	{
		y[i] = y[i - 1];
		x[i] = x[i - 1];
	}
	x[0] = set;
	y[0] = 0;
	
	for (uint8_t i = n; i > 0; i--)
	{
		y[0] = y[0] + b[i] * x[i];
		y[0] = y[0] - a[i] * y[i];
	}	
	output = y[0] + b[0] * x[0];
	output = Func_Limit(output,4000,-4000);
	return output;
}

//PITCH����ǰ����������
s16 Pitch_Feedforward_Controler(s16 set)
{
	s16 output;
	float num;
	float den;
/* nΪ���ݺ������ӷ�ĸ��߽״� */
	const uint8_t n = 3;
//����num Ҫע��ǰ�����������ķ��Ӻͷ�ĸ�Ǵ��ݺ����ĵ���
	const float b[n + 1] = {1, 157.9, 319.3, 3.664};
/* ��ĸden */
	const float a[n + 1] = {0,0.1168, 0.2405, 0};
  if(set <= 3)
		return 0;
	else
	{
	num=b[3]*set*set*set + b[2]*set*set + b[1]*set +b[0];
	den=a[3]*set*set*set + a[2]*set*set + a[1]*set +a[0];
	output = num/den;
	output = Func_Limit(output,4000,-4000);
	return output;
	}
}

//ԭ�ͺ���
//float function(float input)
//{
///* nΪ���ݺ������ӷ�ĸ��߽״� */
//	const uint8_t n = 2;
///* ����num */
//	const float b[n + 1] = {1.0e-03 * 0, 1.0000, -0.9990};
///* ��ĸden */
//	const float a[n + 1] = {1.0000, -1.9990, 0.9990};
// 
//	static float x[n + 1] = {0};
//	static float y[n + 1] = {0};
//	
//	for (uint8_t i = n; i > 0; i--)
//	{
//		y[i] = y[i - 1];
//		x[i] = x[i - 1];
//	}
//	x[0] = input;
//	y[0] = 0;
//	
//	for (uint8_t i = n; i > 0; i--)
//	{
//		y[0] = y[0] + b[i] * x[i];
//		y[0] = y[0] - a[i] * y[i];
//	}	
//	return y[0] + b[0] * x[0];
//}

