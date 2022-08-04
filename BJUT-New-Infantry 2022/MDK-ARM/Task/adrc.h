/**
  ******************************************************************************
  * @file    adrc.c
  * @author  ������
  * @version V1.0
  * @date    2021/10/24
  * @brief   ADRC���㺯��
  ******************************************************************************
  * @attention
  *	
  *
  ******************************************************************************
  */
	
#include "universal.h"
typedef struct
{
/*******���Ź��ɹ���*******/
float x1;//����΢����״̬��
float x2;//����΢����״̬��΢����
float r;//ʱ��߶�
float h;//ADRCϵͳ����ʱ��
u16 N0;//����΢���ڽ���ٶȳ���h0=N*h

float h0;
float fh;//����΢�ּ��ٶȸ�����
float fst;//����΢�ּ��ٶȸ�����

/*******����״̬�۲���*******/
/******��֪���y������u******/
float z1;
float z2;
float z3;//���ݿ��ƶ���������������ȡ���Ŷ���Ϣ
float e;//ϵͳ״̬���
float y;//ϵͳ�����
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
float b;


/**********ϵͳ״̬������*********/
float e0;//״̬��������
float e1;//״̬ƫ��
float e2;//״̬΢����
float u0;//���������ϵͳ���
float u;//�Ŷ�����������
float b0;//�Ŷ�����

/*********��һ�������ʽ*********/
float beta_0;//����
float beta_1;//��������ϲ���
float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0)
/*********�ڶ��������ʽ*********/
float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
float alpha2;//0<alpha1<1<alpha2
float zeta;//���Զε����䳤��
/*********�����������ʽ*********/
float h1;//u0=-fhan(e1,e2,r,h1)
u16 N1;//����΢���ڽ���ٶȳ���h0=N*h
/*********�����������ʽ*********/
float c;//u0=-fhan(e1,c*e2*e2,r,h1)

}AdrcTypeDef;



void ADRC_Init(AdrcTypeDef *fhan_Input1,AdrcTypeDef *fhan_Input2);
void Fhan_ADRC(AdrcTypeDef *fhan_Input,float expect_ADRC);
void ADRC_Control(AdrcTypeDef *fhan_Input,float feedback_ADRC,float expect_ADRC);

extern AdrcTypeDef Gimbal_adrc[2];

