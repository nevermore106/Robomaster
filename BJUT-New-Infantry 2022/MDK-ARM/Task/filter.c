/**
  ******************************************************************************
  * @file    filter.c
  * @author  北京工业大学-张曦梁
  * @version V1.0
  * @date    2022/2/5
  * @brief   滤波器进程
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
  * @brief  一维卡尔曼滤波器
  * @param  input:6020编码器输入值
  * @retval 最优估计值
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
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
    p->X_mid =p->A*p->X_last+p->B*u;               //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(data-p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;							  //输出预测结果x(k|k)
}

/**
  * @brief  加权递推均值滤波器
  * @param  input:编码器输入值
  * @retval 滤波后数值
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

