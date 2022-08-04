  /**
  ******************************************************************************
  * @file    remote_task.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   遥控进程
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

		switch(RC_Ctl.rc.s2)//选择控制模式
		{
			case 1://拨杆处于最上方遥控器控制
				Remote_Control();
				break;
			case 2://拨杆处于最下方电脑控制
				Computer_control();
				TX1_Mode_Switch();
				break;
			default://拨杆处于中位和未检测到遥控器信号的时候，停止一切输入信号
				Gimbal_Disable();
			  TX1_Mode_Switch();
				break;
		}
  }
}

/**
  * @brief  遥控器控制任务
  * @param  None
  * @retval None
  */
void Remote_Control(void)
{
	Command.Vx = (RC_Ctl.rc.ch1 - 1024) * REMOTE_SPEED_ZOOM;//将拨杆中位定义为速度的零值
	Command.Vy = (RC_Ctl.rc.ch0 - 1024) * REMOTE_SPEED_ZOOM;
	
	GM6020[PITCH].Position.Set = -(RC_Ctl.rc.ch3 - 1024) + PITCH_POSITION_INIT;
	GM6020[YAW].Position.Set -=  ((RC_Ctl.rc.ch2 - 1024) * 0.012f);//因为是一个累加的过程，所以降低数

  Automatic_Find_Enemy();//仅开启自动索敌时用

	switch((RC_Ctl.rc.ch4 == 1684) | (RC_Ctl.rc.ch4 == 364) << 4)//滑轮两侧极值
	{
		case 0x01:
			Command.RotateFlag     = 1;
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,60);
		  Command.AimAssitFlag   = MANUAL_ATTACK;
			break;
		case 0x10:
			Command.RotateFlag     = 0;
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,21);//开弹仓
		  Command.AimAssitFlag   = AUTO_ATTACK;
			break;
		default:
//			Command.RotateFlag     = 0;
//			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,60);
//		  Command.AimAssitFlag   = MANUAL_ATTACK;
			break;
	}
	
	//左侧拨杆控制任务
	//中位无操作，向上拨打开摩擦轮，再拨关闭摩擦轮
	//在摩擦轮开启的情况下，向下拨，发射子弹，为单发。
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
			if((Shooting.Friction >= 1) && (Shooting.Last_s1 == 3))//单发
			//if((Shooting.Friction == 1) && (Shooting.Status == READY))//连发
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
  * @brief  电脑控制任务
	*         如何操作下面的注释有写
  * @param  None
  * @retval None
  */
void Computer_control(void)
{
//鼠标左右键 操作 滑动操作
	  Mouse_Control();
//左键射击，单按点射，按住连发
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
	
//右键按下开启自动瞄准
	if(RC_Ctl.mouse.press_r== 1)
	{
		Command.AimAssitFlag = AUTO_ATTACK;
	}
	else
	{
		Command.AimAssitFlag = MANUAL_ATTACK;
	}

//WASD移动控制
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
	
//按住Shift小陀螺
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
	
//按住Ctrl减速
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

//按下右键开启自瞄，按住Q变成风车模式，按E刷新UI
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
	
//F键关闭底盘跟随
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
	
//X键值操作开启摩擦轮
//任意键长按1秒关闭摩擦轮
//短按开启摩擦轮
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

//Z键控制车体斜45度	
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
	
//G键一键转身
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
			
//R键控制弹仓盖，长按R开弹仓，短按R关弹仓
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
	
//C键 加速
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

//V键 拨轮小退一下 ww
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
//    Automatic_Find_Enemy();//仅开启自动索敌时用
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
