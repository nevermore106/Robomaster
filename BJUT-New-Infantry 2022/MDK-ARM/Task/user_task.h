/**
  ******************************************************************************
  * @file    user_task.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2021/9/15
  * @brief   用户进程
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "universal.h"
/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8


typedef struct
{
	u16 DataLength;                       //数据长度
	u8 Seq;                               //包序号
	float DataA;                          //自定义数据ABC
	float DataB;                           
	float DataC;
}Referee_Send_Struct;


typedef __packed struct
{ 
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11;
	
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11;              //图形数据
} Graph_Data_t;

typedef struct
{
		s16   power;//功率
		s16   voltage;//电容电压
	  s16   percent_of_voltage;//电容百分比
	  s16    count;//UI计数
	  float  yaw_angle[4];
	  s16  yaw_x;
	  s16  yaw_y;
	  u32  capcolor;
    u32  armcolor[4];
	  u8 res;
} User_t;

void User_Task(void const * argument);
void Referee_Staticdata_Send(u8 UI_Graph_Operate,char index,char * data,uint8_t data_len);
void Referee_Dynamicdata_Send(u8 UI_Graph_Operate,char index,char * data,uint8_t data_len);
void Referee_Picture_Init(void);

void Draw_One_Picture(Graph_Data_t *image1);
void Draw_Two_Picture(Graph_Data_t *image1,Graph_Data_t *image2);
void Draw_Five_Picture(Graph_Data_t *image1,Graph_Data_t *image2,Graph_Data_t *image3,Graph_Data_t *image4,Graph_Data_t *image5);
void Draw_Seven_Picture(Graph_Data_t *image1,Graph_Data_t *image2,Graph_Data_t *image3,Graph_Data_t *image4,Graph_Data_t *image5,Graph_Data_t *image6,Graph_Data_t *image7);
void Draw_Init(void);
void Capdata_Deal(void);

void Static_Charcpxt_Init(void);
void Dynamic_Charcpxt_Cacl(void);
void Staticdata_Position_Init(void);
void Dynamicdata_Position_Init(void);
void Line_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
void Rectangle_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
void Circle_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius);
void Arc_Draw(Graph_Data_t *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length);
extern User_t User;

