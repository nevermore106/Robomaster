/**
  ******************************************************************************
  * @file    bsp_TX1.h
  * @author  ±±¾©¹¤Òµ´óÑ§-¼ÖÍ©
  * @version V1.0
  * @date    2019/5/9
  * @brief   TX1½ÓÊÕµÄÍ·ÎÄ¼ş
  ******************************************************************************
  * @attention
	* 
	*
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "universal.h"

//ÉÏÏÂÎ»»úÍ¨ĞÅ¶¨Òå
#define ATTACK_RED_ARMOR        0x00
#define ATTACK_BLUE_ARMOR       0xff
#define WINDMILL_CLOCKWISE      0x77
#define WINDMILL_ANTI_CLOCKWISE 0x66
#define ATTACK_WINDMILL         0x44


typedef struct
{
	u8 mode;//ÉÏÎ»»ú¹¤×÷Ä£Ê½£¬ÓÉÏÂÎ»»ú¸ø¶
	u8 hitmode;
	s16 Yaw_Angle;//YawÖá½Ç¶È£¬Îª¿¨¶ûÂüÂË²¨Ç°½Ç¶È
	s16 Pitch_Angle;//PitchÖá½Ç¶È£¬Í¬ÉÏ
	u16 Distance;//Ä¿±ê¾àÀëÖµ
	s16 Speed;//Ä¿±êÒÆ¶¯ËÙ¶È£¬Í¬ÎªÂË²¨Ç°Êı¾İ
	s16 Center_Angle;//ÖĞĞÄÆ«½Ç
	s16 Yaw_Offset;//YawÖĞĞÄµã×ø±êÆ«²î
	u16 ArmorLost;
	s16 Wrongdata;
}TX1_Data_t;



void TX1_DataReceive(void);
void TX1_DataTransmit(void);
void TX1_Mode_Switch(void);

extern u8 TX1_Receive_Data[30];
extern u8 TX1_Transmit_Data[22];
extern TX1_Data_t TX1_Data;
extern s16 pitcom;

