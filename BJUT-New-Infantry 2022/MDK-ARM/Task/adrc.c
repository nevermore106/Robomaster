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

/* Includes ------------------------------------------------------------------*/
#include "Header.h"

AdrcTypeDef Gimbal_adrc[2];//YAW0 PITCH1
const float ADRC_Unit[3][16]=
{
/*TD����΢����   �Ľ�����TD,h0=N*h      ����״̬�۲���ESO           �Ŷ�����           ���������*/
/*  r      h      N               beta_01    beta_02    beta_03     b0         beta_0    beta_1    beta_2      N1   C   alpha1  alpha2  zeta   b*/
 {300000 ,0.002 , 15,              500,      83333,     2000000,  0.001f,    0.02f,    2.3f,     0.001f,    5,   5,   0.8f,   1.5f,   50,   0},
 {300000 ,0.002 , 10,              500,      83333,     2000000,  0.001f,    0.02f,    1.0f,     0.00014f,   5,   5,   0.8f,   1.5f,   50,   1},
 {300000 ,0.002 , 20,              500,      83000,     10000,    0.001f,    0.02f,    3.0f,     0.002f,     5,   5,   0.8f,   1.5f,   50,   0},
};


s16 Sign_ADRC(float Input)
{
    s16 output=0;
    if(Input>1E-6f) 
			output=1;
    else if(Input<-1E-6f) 
			output=-1;
    else 
			output=0;
    return output;
}

s16 Fsg_ADRC(float x,float d)
{
  s16 output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}

void ADRC_Init(AdrcTypeDef *fhan_Input1,AdrcTypeDef *fhan_Input2)
{
  fhan_Input1->r=ADRC_Unit[0][0];
  fhan_Input1->h=ADRC_Unit[0][1];
  fhan_Input1->N0=(u16)(ADRC_Unit[0][2]);
  fhan_Input1->beta_01=ADRC_Unit[0][3];
  fhan_Input1->beta_02=ADRC_Unit[0][4];
  fhan_Input1->beta_03=ADRC_Unit[0][5];
  fhan_Input1->b0=ADRC_Unit[0][6];
  fhan_Input1->beta_0=ADRC_Unit[0][7];
  fhan_Input1->beta_1=ADRC_Unit[0][8];
  fhan_Input1->beta_2=ADRC_Unit[0][9];
  fhan_Input1->N1=(u16)(ADRC_Unit[0][10]);
  fhan_Input1->c=ADRC_Unit[0][11];

  fhan_Input1->alpha1=ADRC_Unit[0][12];
  fhan_Input1->alpha2=ADRC_Unit[0][13];
  fhan_Input1->zeta=ADRC_Unit[0][14];
  fhan_Input1->b=ADRC_Unit[0][15];

  fhan_Input2->r=ADRC_Unit[1][0];
  fhan_Input2->h=ADRC_Unit[1][1];
  fhan_Input2->N0=(u16)(ADRC_Unit[1][2]);
  fhan_Input2->beta_01=ADRC_Unit[1][3];
  fhan_Input2->beta_02=ADRC_Unit[1][4];
  fhan_Input2->beta_03=ADRC_Unit[1][5];
  fhan_Input2->b0=ADRC_Unit[1][6];
  fhan_Input2->beta_0=ADRC_Unit[1][7];
  fhan_Input2->beta_1=ADRC_Unit[1][8];
  fhan_Input2->beta_2=ADRC_Unit[1][9];
  fhan_Input2->N1=(u16)(ADRC_Unit[1][10]);
  fhan_Input2->c=ADRC_Unit[1][11];

  fhan_Input2->alpha1=ADRC_Unit[1][12];
  fhan_Input2->alpha2=ADRC_Unit[1][13];
  fhan_Input2->zeta=ADRC_Unit[1][14];
  fhan_Input2->b=ADRC_Unit[1][15];
}

//ADRC����΢�ָ�����TD,�Ľ����㷨fhan
void Fhan_ADRC(AdrcTypeDef *fhan_Input,float expect_ADRC)//����ADRC���ɹ���
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;//ADRC״̬���������
  x1_delta=fhan_Input->x1-expect_ADRC;//��x1-v(k)����x1�õ���ɢ���¹�ʽ
  fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//��h0����h,�������΢�ָ�������������
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
  a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
  y=x1_delta+a0;//y=x1+a0
  a1=sqrt(d*(d+8*Func_Abs(y)));//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
                  -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//�õ�����΢�ּ��ٶȸ�����
  fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//��������״̬׷����x1
  fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//��������״̬׷����΢��x2
}

//ԭ�㸽���������Զε������ݴκ���
float Fal_ADRC(float e,float alpha,float zeta)
{
    s16 s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(Func_Abs(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}




/***************����״̬�۲���********************/
//״̬�۲�������beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(AdrcTypeDef *fhan_Input)
{
  fhan_Input->e=fhan_Input->z1-fhan_Input->y;//״̬���

  fhan_Input->fe=Fal_ADRC(fhan_Input->e,0.5,fhan_Input->h);//�����Ժ���,��ȡ����״̬�뵱ǰ���
  fhan_Input->fe1=Fal_ADRC(fhan_Input->e,0.25,fhan_Input->h);

  /*************����״̬������**********/
  fhan_Input->z1+=fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
  fhan_Input->z2+=fhan_Input->h*(fhan_Input->z3
                                 -fhan_Input->beta_02*fhan_Input->fe
                                   +fhan_Input->b*fhan_Input->u);
 //ESO����״̬���ٶ��ź�,�����Ŷ�����
  fhan_Input->z3+=fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
}

void Nolinear_Conbination_ADRC(AdrcTypeDef *fhan_Input)
{
  float temp_e2=0;
  temp_e2=Func_Limit(fhan_Input->e2,5000,-5000);
  fhan_Input->u0=fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
                +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);

}


void ADRC_Control(AdrcTypeDef *fhan_Input,float feedback_ADRC,float expect_ADRC)
{
//	   if(expect_ADRC - feedback_ADRC>4096)
//			expect_ADRC=expect_ADRC-8192;
//		 else if( - expect_ADRC + feedback_ADRC>4096)
//			expect_ADRC=8192+expect_ADRC;
    /*�Կ��ſ�������һ��*/
      /*****
      ���Ź��ɹ���,����Ϊ����,
      ��TD����΢�����õ�:
      ���������ź�x1,���������ź�΢��x2
      ******/
      Fhan_ADRC(fhan_Input,expect_ADRC);

    /*�Կ��ſ������ڶ���*/
      /************ϵͳ���ֵΪ������,״̬����,ESO����״̬�۲�������*********/
      fhan_Input->y=feedback_ADRC;
      /*****
      �õ������źŵ�����״̬:
      1״̬�ź�z1;
      2״̬�ٶ��ź�z2;
      3״̬���ٶ��ź�z3?
      z1,z2��TD�õ����ź�x1,x2����,���������Ժ���ӳ�䣬�ٳ�beta
      �õ�δ�����Ŷ�������u
      *********/
      ESO_ADRC(fhan_Input);//���ٶȻ�Ư�ƣ�����û��z3
    /*�Կ��ſ�����������*/
      /********״̬����***/
      fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;//״̬������
      fhan_Input->e1=fhan_Input->x1-fhan_Input->z1;//״̬ƫ����
      fhan_Input->e2=fhan_Input->x2-fhan_Input->z2;//״̬΢����
      /********�������*******/
     /*
      fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
                    +fhan_Input->beta_1*fhan_Input->e1
                    +fhan_Input->beta_2*fhan_Input->e2;
     */
      Nolinear_Conbination_ADRC(fhan_Input);
      /**********�Ŷ�����*******/
//      fhan_Input->u=fhan_Input->u0
//                   -fhan_Input->z3/fhan_Input->b0;
      //������Ư�ƣ�δ�����Ŷ�����
      fhan_Input->u=Func_Limit(fhan_Input->u0,300,-300);
}


