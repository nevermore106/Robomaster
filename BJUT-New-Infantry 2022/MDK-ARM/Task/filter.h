/**
  ******************************************************************************
  * @file    filter.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2022/2/5
  * @brief   滤波器进程头文件
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
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
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
