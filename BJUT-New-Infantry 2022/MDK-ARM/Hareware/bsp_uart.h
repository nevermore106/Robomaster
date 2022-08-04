/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2018/11/8
  * @brief   ͷ�ļ�
  ******************************************************************************
  * @attention
 
 
  ******************************************************************************
  */

#include "usart.h"

typedef struct
{
	struct
	{
		uint16_t ch0;//ң���������ͨ��
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint16_t ch4;
		uint8_t s1;//���Ҳದ��
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;//���x���ƶ��ٶ�
		int16_t y;//���y���ƶ��ٶ�
		int16_t z;//�����˵�����z���ƶ��ٶȣ����Ǿ������ĸ���Ҳ��֪����û�����
		uint8_t press_l;//������
		uint8_t press_r;//����Ҽ�
	}mouse;
	struct
	{
		uint8_t v_l;//���̸߰�λ����
		uint8_t v_h;//�Ͱ�λ
	}key;
}RC_Ctl_t;

typedef struct//���ṹ����������Լ��̳������̰��Ĳ���
{
	u16 Q[2];
	u16 W[2];
	u16 E[2];
	u16 R[2];
	u16 A[2];
	u16 S[2];
	u16 D[2];
	u16 F[2];
	u16 G[2];
	u16 Z[2];
	u16 X[2];
	u16 C[2];
	u16 V[2];
	u16 B[2];
	u16 Shift[2];
	u16 Ctrl[2];
}Key_t; //���̿��ƴ���


void Key_Deal(void);
void Remote_DateReceive(void);
void UART_Start(void);
void UART_RxIdleCallback(UART_HandleTypeDef *huart);
u8 Key_Press(u16 key[],u16 SetTime);

extern Key_t Key;
extern RC_Ctl_t RC_Ctl;
extern u8 Sbus_Receive_Data[18];
