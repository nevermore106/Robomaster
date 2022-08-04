/**
  ******************************************************************************
  * @file    filter.c
  * @author  ������ҵ��ѧ-������
  * @version V1.0
  * @date    2022/2/5
  * @brief   �˲�������ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "universal.h"
typedef struct 
{
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
	  float B;
	  float U;
    float Q;
    float R;
    float H;
}FirstOrder_Kalman_t;

void FirstOrder_KalmanFilter_Init(FirstOrder_Kalman_t *p,float T_Q,float T_R);
float FirstOrder_KalmanFilter_Cacl(FirstOrder_Kalman_t* p,float data,float u);
s16 weightAverageFilter(s16 Value);

extern FirstOrder_Kalman_t Gimbal_Position_Kalman[2];
extern FirstOrder_Kalman_t Gimbal_Speed_Kalman[2];
extern FirstOrder_Kalman_t Imu_Kalman;
extern s8 weight[4];
extern s16 value_buf[4];
