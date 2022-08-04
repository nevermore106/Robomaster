/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @author  ������ҵ��ѧ-������
  * @version V1.0
  * @date    2022/2/7
  * @brief   ��̨����
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
//���������ݻ�ȡ
		Imu_Update();//���������ǣ����������ŷŵ�ʱ�򲻺��ã���ʵ���㷨����)
		
		#if YAW_USE_IMUEX == 1		
		  imuex_attitude_update();//���������			
    #endif	
//�Ӿ�����̨�Ŀ���			
		Vision_Control_Gimbal();
/**********************************Yaw�����**********************************/
		//�޶�����ֵ��Χ
		GM6020[YAW].Position.Set = Func_ValueRannge(GM6020[YAW].Position.Set,8192,0);
		
		YAW_PID_WithIMU();
		
	//YAWת�ػ�
	  #if YAW_USE_CURRENT == 1	
      PID_Calc(&Gimbal_TCurrent_pid[YAW],(-GM6020[YAW].TCurrent),Gimbal_Speed_pid[YAW].Output);//ת�ػ�PID
		  GM6020[YAW].Output = Gimbal_TCurrent_pid[YAW].Output;
	  #elif YAW_USE_CURRENT == 0
      GM6020[YAW].Output = Gimbal_Speed_pid[YAW].Output;		
    #endif
/*********************************Pitch�����********************************/
//�������뷶Χ
		#if PITCH_POSITION_USE_IMU == 1
     GM6020[PITCH].Position.Set = Func_Limit(GM6020[PITCH].Position.Set,5000,3000);//�޲�����������
    #elif PITCH_POSITION_USE_IMU == 0
     GM6020[PITCH].Position.Set = Func_Limit(GM6020[PITCH].Position.Set,PITCH_POSITION_MAX,PITCH_POSITION_MIN);//�޲�����������
    #endif	
		
//�Ա�������ֵ�����˲�
    Gimbal_FirstOrder_KalmanFilter();//��Ҫ���ڹ۲�����֡Ч������(Ҳ������û����QR)
		
//ˮƽ����  
	  #if PITCH_STABLE == 1		
		  Pitch_Stable();	
    #endif
		
    //ADRC����
    #if PITCH_USE_ADRC == 1
      PITCH_ADRC_Control();	
		//PID����	
    #elif PITCH_USE_ADRC == 0			
      PITCH_PID_Control();
    #endif
		
//PITCHת�ػ��������ز���		
		#if PITCH_USE_CURRENT == 1	
  		PID_Calc(&Gimbal_TCurrent_pid[PITCH],(-GM6020[PITCH].TCurrent),Gimbal_Speed_pid[PITCH].Output);//ת�ػ�
  		Gimbal_TCurrent_pid[PITCH].Output += Gravity_Compensate(); 
      GM6020[PITCH].Output = Gimbal_TCurrent_pid[PITCH].Output;
    #elif PITCH_USE_CURRENT == 0		
      Gimbal_Speed_pid[PITCH].Output += Gravity_Compensate();
			GM6020[PITCH].Output = Gimbal_Speed_pid[PITCH].Output;
		#endif
/**********************************�������**********************************/		
//�������
		Shooting_Control();
		
//��������		
    Feed_PID_Control();
		
//���͵���ֵ����ң�����Ҳದ�˴�����λʱ����̨�����
		(RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(GM6020[YAW].Output,GM6020[PITCH].Output,Gimbal_Speed_pid[TRIGGER].Output);
//������
//    (RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(GM6020[YAW].Output,0,Gimbal_Speed_pid[TRIGGER].Output);//��pitch
//		(RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(0,GM6020[PITCH].Output,Gimbal_Speed_pid[TRIGGER].Output);//��yaw
//  	(RC_Ctl.rc.s2 == 3) ? Gimbal_Output(0,0,0) : Gimbal_Output(0,0,Gimbal_Speed_pid[TRIGGER].Output);//��pitch��yaw
//     Gimbal_Output(0,0,0);//�����
	}
}


/**
  * @brief  �������
  * @param  None
  * @retval ״ֵ̬
  */
Shooting_t Shooting = {0};

uint16_t motor_pos[80] = {0};   
uint16_t reversal_time_count = 0;
void Shooting_Control(void)
{
//��¼��ʷ����
	motor_pos[79] = GM2006.Position.Convert;
	for(uint8_t i = 0;i < 79;i++)
	{
		motor_pos[i] = motor_pos[i+1];
	} 
    
//Ħ������һ��������������ң����������ƿ�����־λ
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
		//3��������1500-25.8m/s, 1680-28m/s (max��30.6m/s), 1630-25.2~29.9m/s
	}
	else
	{
		Friction_OFF();
		Shooting.Speed = 0;
	} 
    
//�ж������Ƿ��������
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
	
	if((Shooting.PosSetCount != 0) && (Shooting.Allow == OK))//�����ӵ�
	{
		if(reversal_time_count == 0)
		{
			Shooting.PosSetCount--;
			GM2006.Position.Set -= Shooting.PosRatio;   //����ת
		}
		if(motor_pos[79] == motor_pos[0] && (Shooting.PosSetCount < Shooting.Period * 0.47))//�����˲����������Ϊ���ӵ���
		{
      GM2006.Position.Step = GM2006.Position.Convert;
      GM2006.Position.Set  = GM2006.Position.Convert;
      reversal_time_count  = Shooting.Period * 0.47;
		}		
	}
	
	if(reversal_time_count != 0)//���ӵ��Ļ��ͷ�ת
	{
		reversal_time_count--;
		GM2006.Position.Set += Shooting.PosRatio; //����ת
	}	
}


/**
  * @brief  YAW�����
  * @param  None
  * @retval ״ֵ̬
  */
void YAW_PID_WithIMU(void)
{
//    Gimbal_Dynamic_Limit(&GM6020[YAW].Position.Set,GM6020[YAW].Position.Real,Imuex.Angle.Yaw,&AutoLimit[YAW]);//ͨ�������������Эͬ�����Ƿ񳬳��������̣���Ϊ����С���ݣ���ȡ��
		Func_CircleRamp(GM6020[YAW].Position.Set,&GM6020[YAW].Position.Step,YAW_POSITION_RAMP_RATIO);//б����������
	  #if YAW_USE_IMUEX == 1
      PID_Calc(&Gimbal_Position_pid[YAW],Imuex.Angle.Yaw,GM6020[YAW].Position.Step);//λ�û�PID
		  PID_Calc(&Gimbal_Speed_pid[YAW],Imuex.Speed.Yaw,Gimbal_Position_pid[YAW].Output);//�ٶȻ�PID
		
    #elif YAW_USE_IMUEX == 0		
		  PID_Calc(&Gimbal_Position_pid[YAW],imu.yaw,GM6020[YAW].Position.Step);//λ�û�PID
		  PID_Calc(&Gimbal_Speed_pid[YAW],imu.wz,Gimbal_Position_pid[YAW].Output);//�ٶȻ�PID
    #endif
	
}

void YAW_PID_WithoutIMU(void)
{
		Func_CircleRamp(GM6020[YAW].Position.Set,&GM6020[YAW].Position.Step,YAW_POSITION_RAMP_RATIO);//б����������
		PID_Calc(&Gimbal_Position_pid[YAW],GM6020[YAW].Position.Real,GM6020[YAW].Position.Step);//λ�û�PID
		PID_Calc(&Gimbal_Speed_pid[YAW],GM6020[YAW].Speed.Real,Gimbal_Position_pid[YAW].Output);//�ٶȻ�PID
}

/**
  * @brief  PITCH�����
  * @param  None
  * @retval ״ֵ̬
  */
void PITCH_PID_Control(void)
{
//		Gimbal_Dynamic_Limit(&GM6020[PITCH].Position.Set,GM6020[PITCH].Position.Real,Imuex.Angle.Pitch,&AutoLimit[PITCH]);//�������������ǣ���Ҫ��̬���Ʒ�Χ		
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
    ADRC_Control(&Gimbal_adrc[PITCH],Imuex.Angle.Pitch,GM6020[PITCH].Position.Set);//λ�û�ADRC
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
  * @brief  ��������
  * @param  None
  * @retval ״ֵ̬
  */
void Feed_PID_Control(void)
{
		GM2006.Position.Set = Func_ValueRannge(GM2006.Position.Set,8192,0);//�������뷶Χ
		//Func_CircleRamp(GM2006.Position.Set,&GM2006.Position.Step,TRIGGER_POSITION_RAMP_RATIO);//б����������
		PID_Calc(&Gimbal_Position_pid[TRIGGER],GM2006.Position.Convert,GM2006.Position.Set);//λ�û�PID
		PID_Calc(&Gimbal_Speed_pid[TRIGGER],GM2006.Speed.Real,Gimbal_Position_pid[TRIGGER].Output);//�ٶȻ�PID	
}

/**
  * @brief  ˮƽ����
  * @param  None
  * @retval ״ֵ̬
  */
void Pitch_Stable(void)
{
//	  Stable.Error = (Imuex.Angle.Pitch - 4096);
//    if(Command.RotateFlag == 1)
//		PID_Calc(&Pitch_Stable_pid,Stable.Error,0);//���Ȳ���PID
//		else
//		PID_Clear(&Pitch_Stable_pid);
		
//		GM6020[PITCH].Position.Set +=  imu.pit;	

	  Kalman[PITCH].Imu.Step = imu.pit;
		Kalman[PITCH].Imu.Real = FirstOrder_KalmanFilter_Cacl(&Gimbal_Position_Kalman[PITCH],Kalman[PITCH].Imu.Step,0);
	  GM6020[PITCH].Position.Set += (int)Kalman[PITCH].Imu.Real;
    Kalman[PITCH].Imu.Error = GM6020[PITCH].Position.Set - Kalman[PITCH].Imu.Real;
}

/**
  * @brief  �����ز���
  * @param  None
  * @retval ״ֵ̬
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
  * @brief  һά�������˲���
  * @param  input:6020����������ֵ
  * @retval ���Ź���ֵ
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
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
  * @brief  ��̨��̬�޷�
  * @param  input:����������ֵ
  * @param  encoder:������������ֵ
  * @param  AutoLimit_t:��̬�޷��ṹ�� 
  * @retval ״ֵ̬
  */
void Gimbal_Dynamic_Limit(s16 *input,u16 encoder,u16 imu,AutoLimit_t *autolimit)//������Ҫ���ǹ��㴦���Լ������߼�����
{
	autolimit->Dynamic_Min = Func_ValueRannge(imu - encoder + autolimit->Min,8192,0);
	autolimit->Dynamic_Max = Func_ValueRannge(imu + autolimit->Max - encoder,8192,0);
	if(autolimit->Dynamic_Max > autolimit->Dynamic_Min)
		*input = (s16)Func_Ramp_Limit(*input,autolimit->Dynamic_Max,autolimit->Dynamic_Min,0); //������ܺ��� ��������ֵ���ִ�С
	else 
		*input = (s16)Func_Ramp_Limit(*input,autolimit->Dynamic_Max,autolimit->Dynamic_Min,1);
}


//������յ���miniPC����
 void Vision_Control_Gimbal(void)
{
	//����ǰ��̬���뿨�����˲������Ա�ʹ��
	//����λ�������ͼ���Ǽ�ʮ������ǰ�����ݣ���Ҫ����ǰ����̬��������ʹ��
	//���������׼һЩ
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
	for(u8 i = 0;i < KALMAN_FILTER_PAST_TIME - 1;i++)//����̬���ݽ���һ�ε���
	{
		KF.Yaw_angle[i]   = KF.Yaw_angle[i+1];
		KF.Pitch_angle[i] = KF.Pitch_angle[i+1];
		//KF.Speed[i]       = KF.Speed[i+1];		
	}

	kalman_filter_calc(&KF);//�������˲������м�����
	if(Command.AimAssitFlag == AUTO_ATTACK )//�ж��Ƿ���٣������Ƿ������Զ���׼����װ��Ƭ�ʹ��ʹ�õ���ͬһ�׿��ƻ��ƣ�ֻ�������û��x��Ԥ�ⲹ��
	{
//�������
//		GM6020[YAW].Position.Set   = Yaw_Predict(KF.filtered_value[1],KF.filtered_value[0],Shooting.Speed);
		GM6020[YAW].Position.Set   = KF.filtered_value[0]+ yawc ;//��С��ת��������ת
		GM6020[PITCH].Position.Set = Bullet_Amend(KF.filtered_value[1],Shooting.Speed,TX1_Data.Distance) + pitc;	//��С��ֵ̧ͷ��������ֵ���-380�
//��װ�װ岹��
		
	}
	else
	{
		KF.Kalman_Filter_Yaw        = KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1];
		KF.Kalman_Filter_Pitch      = KF.Pitch_angle[KALMAN_FILTER_PAST_TIME - 1];
		KF.Kalman_Filter_Speed      = 0;
		KF.Kalman_Filter_CenterAngle= KF.Yaw_angle[KALMAN_FILTER_PAST_TIME - 1];
	}
}




s16 Bullet_Amend(float angle,int bullet_Speed,u16 distance)//�ٶȵ�λ��mm/s����pitch���ӵ�����һ�������������
{
//���Կ��������������ӵ�����
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



//TX1_Data_t�����еĽǶ�ֵ����0-8191
s16 Yaw_Predict(TX1_Data_t *data,float angle,int bullet_Speed)
{ 
	//yaw���ٶ�Ԥ�⣬��Ҫ���ڽǶȲ����������Ĳ���Ϊ�ӵ������ӳ����ӵ�����ʱ�䣬ʵ�ⷢ���ӳ�Ϊ100-120ms
	s16 yaw_predict;
	if(bullet_Speed == 0)
		return angle;
	yaw_predict = angle - atan2(KF.filtered_value[2] * (0.12f + data->Distance / bullet_Speed),data->Distance) * 4096 / NORMAL_PI;
	return yaw_predict;
}

//ң�����Ҳದ�˴�����λʱ��̨ʧ�ܣ�ֹͣ��̨���ƣ������ݸ�λ
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




