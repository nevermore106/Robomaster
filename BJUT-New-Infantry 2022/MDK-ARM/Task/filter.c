/**
  ******************************************************************************
  * @file    filter.c
  * @author  ������ҵ��ѧ-������
  * @version V1.0
  * @date    2022/2/5
  * @brief   �˲�������
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Header.h"
FirstOrder_Kalman_t Gimbal_Position_Kalman[2] = {0};
FirstOrder_Kalman_t Gimbal_Speed_Kalman[2] = {0};
FirstOrder_Kalman_t Imu_Kalman = {0};
s16 tttt[2];

/**
  * @brief  һά�������˲���
  * @param  input:6020����������ֵ
  * @retval ���Ź���ֵ
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void FirstOrder_KalmanFilter_Init(FirstOrder_Kalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
	  p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

float FirstOrder_KalmanFilter_Cacl(FirstOrder_Kalman_t* p,float data,float u)
{
    p->X_mid =p->A*p->X_last+p->B*u;               //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(data-p->X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;							  //���Ԥ����x(k|k)
}

/**
  * @brief  ��Ȩ���ƾ�ֵ�˲���
  * @param  input:����������ֵ
  * @retval �˲�����ֵ
  */

s8 weight[4] = {1,2,3,4};
s16 value_buf[4] = {0};
s16 weightAverageFilter(s16 Value)
{
	   tttt[0]++;
    u8 i = 0; 
	  s32 sum = 0;
    for(i = 0;i < 3;i++)
     {
			 tttt[1]++;
			 value_buf[i] = value_buf[i+1];
     }
		value_buf[3] = Value;
    for(i = 0;i < 4;i++)
     {
       sum += value_buf[i] * weight[i];
     }
    return sum/10;
}

