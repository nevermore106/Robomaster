  /**
  ******************************************************************************
  * @file    remote_task.c
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ң�ؽ���
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
#include "tim.h"
Command_t Command={0};

void Remote_Task(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
		osDelayUntil(&xLastWakeTime,2);//500HZ

		switch(RC_Ctl.rc.s2)//ѡ�����ģʽ
		{
			case 1://���˴������Ϸ�ң��������
				Remote_Control();
				break;
			case 2://���˴������·����Կ���
				Computer_control();
				TX1_Mode_Switch();
				break;
			default://���˴�����λ��δ��⵽ң�����źŵ�ʱ��ֹͣһ�������ź�
				Gimbal_Disable();
			  TX1_Mode_Switch();
				break;
		}
  }
}

/**
  * @brief  ң������������
  * @param  None
  * @retval None
  */
void Remote_Control(void)
{
	Command.Vx = (RC_Ctl.rc.ch1 - 1024) * REMOTE_SPEED_ZOOM;//��������λ����Ϊ�ٶȵ���ֵ
	Command.Vy = (RC_Ctl.rc.ch0 - 1024) * REMOTE_SPEED_ZOOM;
	
	GM6020[PITCH].Position.Set = -(RC_Ctl.rc.ch3 - 1024) + PITCH_POSITION_INIT;
	GM6020[YAW].Position.Set -=  ((RC_Ctl.rc.ch2 - 1024) * 0.012f);//��Ϊ��һ���ۼӵĹ��̣����Խ�����

  Automatic_Find_Enemy();//�������Զ�����ʱ��

	switch((RC_Ctl.rc.ch4 == 1684) | (RC_Ctl.rc.ch4 == 364) << 4)//�������༫ֵ
	{
		case 0x01:
			Command.RotateFlag     = 1;
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,60);
		  Command.AimAssitFlag   = MANUAL_ATTACK;
			break;
		case 0x10:
			Command.RotateFlag     = 0;
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,21);//������
		  Command.AimAssitFlag   = AUTO_ATTACK;
			break;
		default:
//			Command.RotateFlag     = 0;
//			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,60);
//		  Command.AimAssitFlag   = MANUAL_ATTACK;
			break;
	}
	
	//��ದ�˿�������
	//��λ�޲��������ϲ���Ħ���֣��ٲ��ر�Ħ����
	//��Ħ���ֿ���������£����²��������ӵ���Ϊ������
	switch(RC_Ctl.rc.s1)
	{
		case 1:
			if(Shooting.Last_s1 == 3)
				Shooting.Friction =! Shooting.Friction;
			Shooting.Last_s1 = 1;
			break;
		case 3:
			Shooting.Last_s1 = 3;
			break;
		case 2:		
			if((Shooting.Friction >= 1) && (Shooting.Last_s1 == 3))//����
			//if((Shooting.Friction == 1) && (Shooting.Status == READY))//����
			{
				//Shooting.Status = SHOOTING;
				Shooting.PosSetCount = Shooting.Period;
			}
			Shooting.Last_s1 = 2;
			break;
		default:
			break;
	} 
}

/**
  * @brief  ���Կ�������
	*         ��β��������ע����д
  * @param  None
  * @retval None
  */
void Computer_control(void)
{
//������Ҽ� ���� ��������
	  Mouse_Control();
//���������������䣬��ס����
	if((RC_Ctl.mouse.press_l == 1) && (Shooting.Friction == 1) && (Shooting.PosSetCount == 0) && (Shooting.PressTime == 0))
	{
		Shooting.PosSetCount = Shooting.Period;
	}
	else if(RC_Ctl.mouse.press_l == 1)
	{
		Shooting.PressTime++;
		if(Shooting.PressTime > 75)
			Shooting.PosSetCount = Shooting.Period * 0.4;
	}
	else
	{
		Shooting.PressTime = 0;
	}
	
//�Ҽ����¿����Զ���׼
	if(RC_Ctl.mouse.press_r== 1)
	{
		Command.AimAssitFlag = AUTO_ATTACK;
	}
	else
	{
		Command.AimAssitFlag = MANUAL_ATTACK;
	}

//WASD�ƶ�����
	switch(Key_Press(Key.W,5) | (Key_Press(Key.S,5) << 4))
	{
		case 0x02:
			Command.Vx =  2500;
			break;
		case 0x20:
			Command.Vx = -2500;
			break;
		default:
			Command.Vx = 0;
			break;
	}
	switch(Key_Press(Key.A,5) | (Key_Press(Key.D,5) << 4))
	{
		case 0x02:
			Command.Vy = -2500;
			break;
		case 0x20:
			Command.Vy =  2500;
			break;
		default:
			Command.Vy = 0;
			break;
	}
	
//��סShiftС����
	switch(Key_Press(Key.Shift,5))
	{
		case 1:
			Command.RotateFlag = 1;
			break;
		case 2:
			Command.RotateFlag = 1;
			break;
		default:
			Command.RotateFlag = 0;
			break;
	}
	
//��סCtrl����
	switch(Key_Press(Key.Ctrl,5))
	{
		case 2:
//			if(Command.RotateFlag == 1)
//				 SpinTop.Speed = ROTATE_LOW_SPEED;
//			else
				 Command.SpeedMinish = 0.5f;
			break;
		default:
			Command.SpeedMinish = 1;//
			break;
	} 

//�����Ҽ��������飬��סQ��ɷ糵ģʽ����Eˢ��UI
	switch(Key_Press(Key.Q,5) | (Key_Press(Key.E,5) << 4))
	{
		case 0x02:
		case 0x12:
				TX1_Data.hitmode  =  0xee;
			break;
		case 0x20:
		case 0x21:
			User.count = 0;
			break;
		default:		
			Command.AimRotateAssitFlag   =   ATTACK_NORMAL_ARMOR;
		  TX1_Data.hitmode   =   0x11;
			break;
	}
	
//F���رյ��̸���
	switch(Key_Press(Key.F,5))
	{
		case 1:
			break;
		case 2:
			Command.GimbalFollowOFF = 1;
			break;
		default:
			Command.GimbalFollowOFF = 0;
			break;
	}	
	
//X��ֵ��������Ħ����
//���������1��ر�Ħ����
//�̰�����Ħ����
	switch(Key_Press(Key.X,500))
	{
		case 2:
			Friction_OFF();
			Shooting.Friction = 0;
			break;
		case 1:
			Shooting.Friction = 1;
			break;
		default:
			break;
	}

//Z�����Ƴ���б45��	
	switch(Key_Press(Key.Z,5))
	{
		case 1:
			if(Yaw_Position_Move == 0)
			 Yaw_Position_Move = 1024;
			else if(Yaw_Position_Move == 1024)
			 Yaw_Position_Move = 0;
			break;
		default:
			break;
	}
	
//G��һ��ת��
	{
		static char rotate_flag = 0;
		static int  rotate_count = 0;
		switch(Key_Press(Key.G,5))
		{
			case 1:
				rotate_flag = 1;
				rotate_count = 0;
				break;
			default:
				if(rotate_count == 1024)
					rotate_flag = 0;
				if(rotate_flag == 1)
				{
					rotate_count++;
					GM6020[YAW].Position.Set += 4; 
				}
				break;
		}
  }
			
//R�����Ƶ��ָǣ�����R�����֣��̰�R�ص���
	switch(Key_Press(Key.R,500))
	{
		case 2:
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,21);
			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,60);
			break;
		default:
			break;
	}
	
//C�� ����
	switch(Key_Press(Key.C,5))
	{
		case 2:
			if(User.voltage > 1900)
				{
					if(Command.RotateFlag == 1)
					{
						SpinTop.Speed = ROTATE_HIGH_SPEED;
						Command.SpeedZoom = 1;
					}
					else
					{
						SpinTop.Speed = ROTATE_NORMAL_SPEED;
						Command.SpeedZoom = 2;
					}
				}
		break;
		default:
			Command.SpeedZoom = 1;
		  SpinTop.Speed = ROTATE_NORMAL_SPEED;
			break;
	}

//V�� ����С��һ�� ww
	switch(Key_Press(Key.V,5))
	{
		case 1:
			Command.TriggerAnti = 1;
			break;
		case 2:
			break;
		default:
			break;
	}
//    Automatic_Find_Enemy();//�������Զ�����ʱ��
}

void Automatic_Find_Enemy(void)
{
	GM6020[YAW].Position.Set += GM6020[YAW].Rotate;
  GM6020[YAW].Rotate = 0;
}

void Mouse_Control(void)
{
	RC_Ctl.mouse.x = Func_Limit(RC_Ctl.mouse.x,400,-400);
	RC_Ctl.mouse.y = Func_Limit(RC_Ctl.mouse.y,400,-400);
	if(RC_Ctl.mouse.x>0)
		Command.Mouse_x -= RC_Ctl.mouse.x/35+0.2f;

  else if(RC_Ctl.mouse.x<0)
	  Command.Mouse_x -= RC_Ctl.mouse.x/35-0.2f;
	
	GM6020[YAW].Position.Set = Command.Mouse_x;

	if(RC_Ctl.mouse.y>0)
	  Command.Mouse_y += RC_Ctl.mouse.y/35+0.2f;
	
	else if(RC_Ctl.mouse.y<0)
	  Command.Mouse_y += RC_Ctl.mouse.y/35-0.2f;
	
  Command.Mouse_y = Func_Limit(Command.Mouse_y,PITCH_POSITION_MAX,PITCH_POSITION_MIN);
	GM6020[PITCH].Position.Set = Command.Mouse_y;
}
