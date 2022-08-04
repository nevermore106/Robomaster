/**
  ******************************************************************************
  * @file    user_task.c
  * @author  ������ҵ��ѧ-������
  * @version V2.0
  * @date    2021/9/15
  * @brief   �û�����
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
# include <stdio.h>
# include <stdlib.h>
User_t User={0};
u8 mark_count[6]={0};
char dynamic_charcpxt[9];
char static_charcpxt[24];
Referee_Send_Struct Referee_Send={0};
Graph_Data_t Static_Data;
Graph_Data_t Dynamic_Data;
Graph_Data_t G[15] = {0};
int chassis_choose_int = CHASSIS_CHOOSE;

void User_Task(void const * argument)
{ 
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	Static_Charcpxt_Init();
  Staticdata_Position_Init();
	Dynamicdata_Position_Init();
	while(1)
	{
		osDelayUntil(&xLastWakeTime,40);//25HZ
		User.count ++;
		
		Capdata_Deal();
		
		for(u8 i=0;i<4;i++)
		{
			if(InfantryJudge.InjureMark[i] >0)
			{
				InfantryJudge.InjureMark[i] -- ;
				User.armcolor[i] = UI_Color_Purplish_red;
			}
			else
			{
				User.armcolor[i] = UI_Color_Green;
			}
	  }
		
		User.yaw_angle[0] = (GM6020[YAW].Position.Real-YAW_POSITION_INIT)/22.755555f * 0.017444f;
	  User.yaw_angle[1] = ((GM6020[YAW].Position.Real-YAW_POSITION_INIT)/22.755555f + 90) * 0.017444f;
	  User.yaw_angle[2] = ((GM6020[YAW].Position.Real-YAW_POSITION_INIT)/22.755555f + 180) * 0.017444f;
	  User.yaw_angle[3] = ((GM6020[YAW].Position.Real-YAW_POSITION_INIT)/22.755555f + 270) * 0.017444f;
		User.yaw_x = 100*sin(User.yaw_angle[0]);
		User.yaw_y = 100*cos(User.yaw_angle[0]);//����ָʾ������
		
		if(User.count < 10)//���д�뾲̬ͼ�η�ֹ��֡
		{		
			mark_count[0]++;
			Draw_Init();
			Draw_Five_Picture(&G[9],&G[10],&G[11],&G[12],&G[13]);
		}
		
		else if(User.count < 20)//�����Ӷ�̬ͼ��
		{
			mark_count[1]++;
			Draw_Two_Picture(&G[7],&G[8]);
		}
		
		else if(User.count < 30)
		{
			mark_count[2]++;
		  Draw_Seven_Picture(&G[0],&G[1],&G[2],&G[3],&G[4],&G[5],&G[6]);
		}
		
    else if(User.count < 40)//���д�뾲̬�ַ�����ֹ��֡
		{
			mark_count[3]++;
			Referee_Staticdata_Send(UI_Graph_ADD,0x30,static_charcpxt,strlen(static_charcpxt));	
		}		
		
    else if(User.count < 50)//�����Ӷ�̬�ַ�
		{
			mark_count[4]++;
			Referee_Dynamicdata_Send(UI_Graph_ADD,0x33,dynamic_charcpxt,strlen(dynamic_charcpxt));	
		}
		
		else 
		{   
			mark_count[5]++;
			User.res = User.count%4;
       if(User.res == 0)
			  {
					if(Shooting.Friction == 1)//Ħ���ֱ�ʶ
						Circle_Draw(&G[4],"071",UI_Graph_Change,8,UI_Color_Green,5,130,867,8);			
					else
						Circle_Draw(&G[4],"071",UI_Graph_Change,8,UI_Color_Purplish_red,5,130,867,8);
					
					if(Command.RotateFlag == 1)//С���ݱ�ʶ
						Circle_Draw(&G[5],"072",UI_Graph_Change,8,UI_Color_Green,5,130,789,8);		
					else
						Circle_Draw(&G[5],"072",UI_Graph_Change,8,UI_Color_Purplish_red,5,130,789,8);
					
					if(TX1_Data.hitmode == 0xee)//�糵��ʶ
						Circle_Draw(&G[6],"073",UI_Graph_Change,8,UI_Color_Green,5,130,711,8);		
					else
						Circle_Draw(&G[6],"073",UI_Graph_Change,8,UI_Color_Purplish_red,5,130,711,8);
				
					if(User.voltage > 1900)//���ݱ�ʶ
						Circle_Draw(&G[7],"074",UI_Graph_Change,8,UI_Color_Green,5,130,633,8);		
					else
						Circle_Draw(&G[7],"074",UI_Graph_Change,8,UI_Color_Purplish_red,5,130,633,8);
//						Line_Draw(&G[8],"061",UI_Graph_Change,8,UI_Color_Main,5,1700,620,(uint32_t)(1700 + User.yaw_x) ,(uint32_t)(620 + User.yaw_y));
					Line_Draw(&G[9],"062",UI_Graph_Change,7,User.capcolor,15,810,300,(uint32_t)(805 + 300 * (User.percent_of_voltage - 1800)/400),300);
					Draw_Five_Picture(&G[4],&G[5],&G[6],&G[7],&G[9]);
				}
			if(User.res == 1||User.res == 2)
			  {
					Rectangle_Draw(&G[10],"051",UI_Graph_Change,5,User.armcolor[0],5,(uint32_t)(1680+100*sin(User.yaw_angle[0])),(uint32_t)(610+100*cos(User.yaw_angle[0])),(uint32_t)(1720+100*sin(User.yaw_angle[0])),(uint32_t)(630+100*cos(User.yaw_angle[0])));
//			    Circle_Draw(&G[10],"051",UI_Graph_Change,5,User.armcolor[0],5,(uint32_t)(1700+100*sin(User.yaw_angle[0])),(uint32_t)(620+100*cos(User.yaw_angle[0])),20);
					Circle_Draw(&G[11],"052",UI_Graph_Change,5,User.armcolor[3],5,(uint32_t)(1700+100*sin(User.yaw_angle[1])),(uint32_t)(620+100*cos(User.yaw_angle[1])),20);
					Circle_Draw(&G[12],"053",UI_Graph_Change,5,User.armcolor[2],5,(uint32_t)(1700+100*sin(User.yaw_angle[2])),(uint32_t)(620+100*cos(User.yaw_angle[2])),20);
					Circle_Draw(&G[13],"054",UI_Graph_Change,5,User.armcolor[1],5,(uint32_t)(1700+100*sin(User.yaw_angle[3])),(uint32_t)(620+100*cos(User.yaw_angle[3])),20);
					Line_Draw(&G[14],"055",UI_Graph_Change,5,UI_Color_Green,7,1400,500,1400,600);
					Draw_Five_Picture(&G[10],&G[11],&G[12],&G[13],&G[14]);
			  }
			if(User.res == 3)
			  {
					Dynamic_Charcpxt_Cacl();
			    Referee_Dynamicdata_Send(UI_Graph_Change,0x33,dynamic_charcpxt,strlen(dynamic_charcpxt));	
		  	}
			
    }
		if(User.count > 30000)			
				User.count =60;
	}
}


void Draw_Init(void)
{
		Line_Draw(&G[0],"091",UI_Graph_ADD,9,UI_Color_Cyan,3,960 - 25,496     ,960 - 6 ,496     );
		Line_Draw(&G[1],"092",UI_Graph_ADD,9,UI_Color_Cyan,3,960     ,496 - 25,960     ,496 - 6 );
		Line_Draw(&G[2],"093",UI_Graph_ADD,9,UI_Color_Cyan,3,960 +  6,496     ,960 + 25,496     );
		Line_Draw(&G[3],"094",UI_Graph_ADD,9,UI_Color_Cyan,3,960     ,496 + 6 ,960     ,496 + 25);
		Circle_Draw(&G[4],"071",UI_Graph_ADD,8,UI_Color_Purplish_red,5,130,867,8);
		Circle_Draw(&G[5],"072",UI_Graph_ADD,8,UI_Color_Purplish_red,5,130,789,8);
		Circle_Draw(&G[6],"073",UI_Graph_ADD,8,UI_Color_Purplish_red,5,130,711,8);
		Circle_Draw(&G[7],"074",UI_Graph_ADD,8,UI_Color_Purplish_red,5,130,633,8);
		Line_Draw(&G[8],"061",UI_Graph_ADD,8,UI_Color_Main,5,1700,620,1700,680);//YAWָʾ
		Line_Draw(&G[9],"062",UI_Graph_ADD,7,UI_Color_Green,15,810,300,1110,300);//����ָʾ
		Rectangle_Draw(&G[10],"051",UI_Graph_ADD,5,User.armcolor[0],5,1680,710,1720,730);//����װ�װ�ָʾ
	//	Circle_Draw(&G[10],"051",UI_Graph_ADD,5,UI_Color_Green,5,1700,720,15);
		Circle_Draw(&G[11],"052",UI_Graph_ADD,5,UI_Color_Green,5,1800,620,20);
		Circle_Draw(&G[12],"053",UI_Graph_ADD,5,UI_Color_Green,5,1700,520,20);
		Circle_Draw(&G[13],"054",UI_Graph_ADD,5,UI_Color_Green,5,1600,620,20);
		Line_Draw(&G[14],"055",UI_Graph_ADD,5,UI_Color_Green,7,1400,500,1400,600);//PITCHָʾ
}


void Static_Charcpxt_Init(void)
{
		static_charcpxt[0] = 'f';
		static_charcpxt[1] = 'r';
		static_charcpxt[2] = 'i';
		static_charcpxt[3] = '\n';	
		static_charcpxt[4] = '\n';
		static_charcpxt[5] = 'r';
		static_charcpxt[6] = 'o';
	  static_charcpxt[7] = 't';
		static_charcpxt[8] = 'e';
		static_charcpxt[9] = '\n';
		static_charcpxt[10] = '\n';
	  static_charcpxt[11] = 'a';
	  static_charcpxt[12] = 'i';
	  static_charcpxt[13] = 'm';
	  static_charcpxt[14] = '\n';
	  static_charcpxt[15] = '\n';
	  static_charcpxt[16] = 'c';
	  static_charcpxt[17] = 'a';
	  static_charcpxt[18] = 'p';
	  static_charcpxt[19] = '\n';
  	static_charcpxt[20] = '\n';
	  static_charcpxt[21] = 'p';
	  static_charcpxt[22] = 'i';
	  static_charcpxt[23] = 't';
	
}

void Dynamic_Charcpxt_Cacl(void)
{
		dynamic_charcpxt[0] = '0' + (User.percent_of_voltage-1800)/400;	
		dynamic_charcpxt[1] = '0' + (User.percent_of_voltage-1800)/40%10;
		dynamic_charcpxt[2] = '0' + (User.percent_of_voltage-1800)/4%10;
		dynamic_charcpxt[3] = '\n';
		dynamic_charcpxt[4] = '\n';
	  dynamic_charcpxt[5] = '0' + GM6020[PITCH].Position.Real/1000%10;
		dynamic_charcpxt[6] = '0' + GM6020[PITCH].Position.Real/100%10;
		dynamic_charcpxt[7] = '0' + GM6020[PITCH].Position.Real/10%10;
		dynamic_charcpxt[8] = '0' + GM6020[PITCH].Position.Real%10;
}

void Staticdata_Position_Init(void)
{
		Static_Data.graphic_tpye = UI_Graph_Char;//���Ͻ�������ʾ
		Static_Data.layer        = 1;
		Static_Data.start_angle  = 26;
		Static_Data.end_angle    = 2;
		Static_Data.width        = 3;
		Static_Data.start_x      = 200;//��Ļ����
		Static_Data.start_y      = 880;
}

void Dynamicdata_Position_Init(void)
{
		Dynamic_Data.graphic_tpye = UI_Graph_Char;//���Ͻ�������ʾ
		Dynamic_Data.layer        = 2;
		Dynamic_Data.start_angle  = 26;
		Dynamic_Data.end_angle    = 2;
		Dynamic_Data.width        = 3;
		Dynamic_Data.start_x      = 350;//��Ļ����
		Dynamic_Data.start_y      = 645;
}


/**
  * @brief  ����ֱ��
  * @param  None
  * @retval ״ֵ̬
  */
        
void Line_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
    int i;
    for(i=0;i<3;i++)
      image->graphic_name[2-i]=imagename[i];
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/**
  * @brief  ���ƾ���
  * @param  None
  * @retval ״ֵ̬
  */
        
void Rectangle_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
    int i;
    for(i=0;i<3;i++)
       image->graphic_name[2-i]=imagename[i];
    image->graphic_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/**
  * @brief  ����Բ��
  * @param  None
  * @retval ״ֵ̬
  */
        
void Circle_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
    int i;
    for(i=0;i<3;i++)
      image->graphic_name[2-i]=imagename[i];
    image->graphic_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}

/**
  * @brief  ����Բ��
  * @param  None
  * @retval ״ֵ̬
  */
        
void Arc_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length)
{
		int i;  
		for(i=0;i<3;i++)
			image->graphic_name[2-i]=imagename[i];
		image->graphic_tpye = UI_Graph_Arc;
		image->operate_tpye = Graph_Operate;
		image->layer = Graph_Layer;
		image->color = Graph_Color;
		image->width = Graph_Width;
		image->start_x = Start_x;
		image->start_y = Start_y;
		image->start_angle = Graph_StartAngle;
		image->end_angle = Graph_EndAngle;
		image->end_x = x_Length;
		image->end_y = y_Length;
}

/**
  * @brief  ����ϵͳ���ݷ��ͣ�����������Ը��������ݽ��в�֣�Ȼ����
  * @param  a\b\c ��λ������Ҫ��ʾ������
  * @retval None
  */
void Referee_Staticdata_Send(u8 UI_Graph_Operate,char index,char * data,uint8_t data_len)//data_len = 21
{	
		const static uint8_t char_data_len = 51;
		/*********���¹���֡ͷ********/
		Referee_Send.Seq++;
		JudgeSendBuff[0] = 0xA5;//֡ͷ
		JudgeSendBuff[1] = char_data_len;//���ݳ��ȵͰ�λ
		JudgeSendBuff[2] = 0x00;//���ݳ��ȸ߰�λ
		JudgeSendBuff[3] = Referee_Send.Seq;
		Append_CRC8_Check_Sum(JudgeSendBuff,5);//֡ͷCRCУ��
		/*********���Ϲ���֡ͷ********/
		
		JudgeSendBuff[5] = 0x01;//�����˼佻������������ 0x301
		JudgeSendBuff[6] = 0x03;//�����˼佻������������ 0x301
		
		/*********�������ݴ����￪ʼ********/
		JudgeSendBuff[7] = 0x10;//Э��涨����,����һ��ͼ�ε�������
		JudgeSendBuff[8] = 0x01;//Э��涨����
		
		JudgeSendBuff[9]  = InfantryJudge.RobotID;//��ǰ�����˵�ID
		JudgeSendBuff[10] = InfantryJudge.RobotID >> 8;
		
		JudgeSendBuff[11] = (InfantryJudge.RobotID + 256);//�����ֿ��Ƶ�ID = ������ID + 256
		JudgeSendBuff[12] = (InfantryJudge.RobotID >> 8) + 1;
			
		Static_Data.graphic_name[0] = index;//�����ͼ����Ҫ���ַ���
		Static_Data.graphic_name[1] = 0x31;
		Static_Data.graphic_name[2] = 0x32;

		Static_Data.operate_tpye = UI_Graph_Operate;

		memcpy(&JudgeSendBuff[13], &Static_Data.graphic_name[0],11);
		
		//�ַ������ݴ��⿪ʼ
		memcpy(&JudgeSendBuff[28], data,data_len);
		for(uint8_t i = 52;i < 58;i++)
			JudgeSendBuff[i] = 0;
		Append_CRC16_Check_Sum(JudgeSendBuff,char_data_len + 9);//CRCУ��
		HAL_UART_Transmit_IT(&huart6,JudgeSendBuff,char_data_len + 9);//����
}

void Referee_Dynamicdata_Send(u8 UI_Graph_Operate,char index,char * data,uint8_t data_len)//data_len = 21
{	
		const static uint8_t char_data_len = 51;
		/*********���¹���֡ͷ********/
		Referee_Send.Seq++;
		JudgeSendBuff[0] = 0xA5;//֡ͷ
		JudgeSendBuff[1] = char_data_len;//���ݳ��ȵͰ�λ
		JudgeSendBuff[2] = 0x00;//���ݳ��ȸ߰�λ
		JudgeSendBuff[3] = Referee_Send.Seq;
		Append_CRC8_Check_Sum(JudgeSendBuff,5);//֡ͷCRCУ��
		/*********���Ϲ���֡ͷ********/
		
		JudgeSendBuff[5] = 0x01;//�����˼佻������������ 0x301
		JudgeSendBuff[6] = 0x03;//�����˼佻������������ 0x301
		
		/*********�������ݴ����￪ʼ********/
		JudgeSendBuff[7] = 0x10;//Э��涨����,����һ��ͼ�ε�������
		JudgeSendBuff[8] = 0x01;//Э��涨����
		
		JudgeSendBuff[9]  = InfantryJudge.RobotID;//��ǰ�����˵�ID
		JudgeSendBuff[10] = InfantryJudge.RobotID >> 8;
		
		JudgeSendBuff[11] = (InfantryJudge.RobotID + 256);//�����ֿ��Ƶ�ID = ������ID + 256
		JudgeSendBuff[12] = (InfantryJudge.RobotID >> 8) + 1;
			
		Dynamic_Data.graphic_name[0] = index;//�����ͼ����Ҫ���ַ���
		Dynamic_Data.graphic_name[1] = 0x34;
		Dynamic_Data.graphic_name[2] = 0x35;

		Dynamic_Data.operate_tpye = UI_Graph_Operate;

		memcpy(&JudgeSendBuff[13], &Dynamic_Data.graphic_name[0],11);
		
		//�ַ������ݴ��⿪ʼ
		memcpy(&JudgeSendBuff[28], data,data_len);
		for(uint8_t i = 37;i < 58;i++)
			JudgeSendBuff[i] = 0;
		Append_CRC16_Check_Sum(JudgeSendBuff,char_data_len + 9);//CRCУ��
		HAL_UART_Transmit_IT(&huart6,JudgeSendBuff,char_data_len + 9);//����
}


void Draw_One_Picture(Graph_Data_t *image1)
{
		const static uint8_t char_data_len = 21;
		/*********���¹���֡ͷ********/
		Referee_Send.Seq++;
		JudgeSendBuff[0] = 0xA5;//֡ͷ
		JudgeSendBuff[1] = char_data_len;//���ݳ��ȵͰ�λ
		JudgeSendBuff[2] = 0x00;//���ݳ��ȸ߰�λ
		JudgeSendBuff[3] = Referee_Send.Seq;
		Append_CRC8_Check_Sum(JudgeSendBuff,5);//֡ͷCRCУ��
		/*********���Ϲ���֡ͷ********/
		
		JudgeSendBuff[5] = 0x01;//�����˼佻������������ 0x301
		JudgeSendBuff[6] = 0x03;//�����˼佻������������ 0x301
		
		/*********�������ݴ����￪ʼ********/
		JudgeSendBuff[7] = 0x01;//Э��涨����,����һ��ͼ�ε�������
		JudgeSendBuff[8] = 0x01;//Э��涨����
		
		JudgeSendBuff[9]  = InfantryJudge.RobotID;//��ǰ�����˵�ID
		JudgeSendBuff[10] = InfantryJudge.RobotID >> 8;
		
		JudgeSendBuff[11] = (InfantryJudge.RobotID + 256);//�����ֿ��Ƶ�ID = ������ID + 256
		JudgeSendBuff[12] = (InfantryJudge.RobotID >> 8) + 1;
		
		
		memcpy(&JudgeSendBuff[13], &image1->graphic_name[0],15);
		
		Append_CRC16_Check_Sum(JudgeSendBuff,char_data_len + 9);//CRCУ��
		HAL_UART_Transmit_IT(&huart6,JudgeSendBuff,char_data_len + 9);//����
}

void Draw_Two_Picture(Graph_Data_t *image1,Graph_Data_t *image2)
{
		const static uint8_t char_data_len = 36;
		/*********���¹���֡ͷ********/
		Referee_Send.Seq++;
		JudgeSendBuff[0] = 0xA5;//֡ͷ
		JudgeSendBuff[1] = char_data_len;//���ݳ��ȵͰ�λ
		JudgeSendBuff[2] = 0x00;//���ݳ��ȸ߰�λ
		JudgeSendBuff[3] = Referee_Send.Seq;
		Append_CRC8_Check_Sum(JudgeSendBuff,5);//֡ͷCRCУ��
		/*********���Ϲ���֡ͷ********/
		
		JudgeSendBuff[5] = 0x01;//�����˼佻������������ 0x301
		JudgeSendBuff[6] = 0x03;//�����˼佻������������ 0x301
		
		/*********�������ݴ����￪ʼ********/
		JudgeSendBuff[7] = 0x02;//Э��涨����,����һ��ͼ�ε�������
		JudgeSendBuff[8] = 0x01;//Э��涨����
		
		JudgeSendBuff[9]  = InfantryJudge.RobotID;//��ǰ�����˵�ID
		JudgeSendBuff[10] = InfantryJudge.RobotID >> 8;
		
		JudgeSendBuff[11] = (InfantryJudge.RobotID + 256);//�����ֿ��Ƶ�ID = ������ID + 256
		JudgeSendBuff[12] = (InfantryJudge.RobotID >> 8) + 1;
		
		
		memcpy(&JudgeSendBuff[13], &image1->graphic_name[0],15);
		memcpy(&JudgeSendBuff[28], &image2->graphic_name[0],15);
		
		Append_CRC16_Check_Sum(JudgeSendBuff,char_data_len + 9);//CRCУ��
		HAL_UART_Transmit_IT(&huart6,JudgeSendBuff,char_data_len + 9);//����
}

void Draw_Five_Picture(Graph_Data_t *image1,Graph_Data_t *image2,Graph_Data_t *image3,Graph_Data_t *image4,Graph_Data_t *image5)
{
		const static uint8_t char_data_len = 81;
		/*********���¹���֡ͷ********/
		Referee_Send.Seq++;
		JudgeSendBuff[0] = 0xA5;//֡ͷ
		JudgeSendBuff[1] = char_data_len;//���ݳ��ȵͰ�λ
		JudgeSendBuff[2] = 0x00;//���ݳ��ȸ߰�λ
		JudgeSendBuff[3] = Referee_Send.Seq;
		Append_CRC8_Check_Sum(JudgeSendBuff,5);//֡ͷCRCУ��
		/*********���Ϲ���֡ͷ********/
		
		JudgeSendBuff[5] = 0x01;//�����˼佻������������ 0x301
		JudgeSendBuff[6] = 0x03;//�����˼佻������������ 0x301
		
		/*********�������ݴ����￪ʼ********/
		JudgeSendBuff[7] = 0x03;//Э��涨����,����һ��ͼ�ε�������
		JudgeSendBuff[8] = 0x01;//Э��涨����
		
		JudgeSendBuff[9]  = InfantryJudge.RobotID;//��ǰ�����˵�ID
		JudgeSendBuff[10] = InfantryJudge.RobotID >> 8;
		
		JudgeSendBuff[11] = (InfantryJudge.RobotID + 256);//�����ֿ��Ƶ�ID = ������ID + 256
		JudgeSendBuff[12] = (InfantryJudge.RobotID >> 8) + 1;
		
		
		memcpy(&JudgeSendBuff[13], &image1->graphic_name[0],15);
		memcpy(&JudgeSendBuff[28], &image2->graphic_name[0],15);
		memcpy(&JudgeSendBuff[43], &image3->graphic_name[0],15);
		memcpy(&JudgeSendBuff[58], &image4->graphic_name[0],15);
		memcpy(&JudgeSendBuff[73], &image5->graphic_name[0],15);
		
		Append_CRC16_Check_Sum(JudgeSendBuff,char_data_len + 9);//CRCУ��
		HAL_UART_Transmit_IT(&huart6,JudgeSendBuff,char_data_len + 9);//����
}

void Draw_Seven_Picture(Graph_Data_t *image1,Graph_Data_t *image2,Graph_Data_t *image3,Graph_Data_t *image4,Graph_Data_t *image5,Graph_Data_t *image6,Graph_Data_t *image7)
{
		const static uint8_t char_data_len = 111;
		/*********���¹���֡ͷ********/
		Referee_Send.Seq++;
		JudgeSendBuff[0] = 0xA5;//֡ͷ
		JudgeSendBuff[1] = char_data_len;//���ݳ��ȵͰ�λ
		JudgeSendBuff[2] = 0x00;//���ݳ��ȸ߰�λ
		JudgeSendBuff[3] = Referee_Send.Seq;
		Append_CRC8_Check_Sum(JudgeSendBuff,5);//֡ͷCRCУ��
		/*********���Ϲ���֡ͷ********/
		
		JudgeSendBuff[5] = 0x01;//�����˼佻������������ 0x301
		JudgeSendBuff[6] = 0x03;//�����˼佻������������ 0x301
		
		/*********�������ݴ����￪ʼ********/
		JudgeSendBuff[7] = 0x04;//Э��涨����,����һ��ͼ�ε�������
		JudgeSendBuff[8] = 0x01;//Э��涨����
		
		JudgeSendBuff[9]  = InfantryJudge.RobotID;//��ǰ�����˵�ID
		JudgeSendBuff[10] = InfantryJudge.RobotID >> 8;
		
		JudgeSendBuff[11] = (InfantryJudge.RobotID + 256);//�����ֿ��Ƶ�ID = ������ID + 256
		JudgeSendBuff[12] = (InfantryJudge.RobotID >> 8) + 1;
		
		
		memcpy(&JudgeSendBuff[13], &image1->graphic_name[0],15);
		memcpy(&JudgeSendBuff[28], &image2->graphic_name[0],15);
		memcpy(&JudgeSendBuff[43], &image3->graphic_name[0],15);
		memcpy(&JudgeSendBuff[58], &image4->graphic_name[0],15);
		memcpy(&JudgeSendBuff[73], &image5->graphic_name[0],15);
		memcpy(&JudgeSendBuff[88], &image6->graphic_name[0],15);
		memcpy(&JudgeSendBuff[103], &image7->graphic_name[0],15);
		
		Append_CRC16_Check_Sum(JudgeSendBuff,char_data_len + 9);//CRCУ��
		HAL_UART_Transmit_IT(&huart6,JudgeSendBuff,char_data_len + 9);//����
}

void Capdata_Deal(void)
{
	  User.voltage = (int)(PowerData[1]*100);
		User.power = (int)(PowerData[3]);
		if(User.voltage > 2200)
			User.percent_of_voltage = 2200;
		else if(User.voltage < 1800)
			User.percent_of_voltage = 1800;
		else
			User.percent_of_voltage = User.voltage;//���ݵ�ѹ����

		if(User.voltage < 1900)
			User.capcolor  = UI_Color_Purplish_red;
		else if (User.voltage < 2100)
			User.capcolor  = UI_Color_Orange;
		else
			User.capcolor  = UI_Color_Green;
}


