/**
  ******************************************************************************
  * @file    KalmanFilter.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/5/9
  * @brief   卡尔曼滤波器
  ******************************************************************************
  * @attention
 
 
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "universal.h"
#include "Config.h"



/*
 *@Function ：mat_init    (mat,行,列,数据)
 *@Function ：mat_add     (A,B,结果)
 *@Function ：mat_sub     (A,B,结果)
 *@Function ：mat_mult    (A,B,结果)
 *@Function ：mat_trans   (原始矩阵,转置矩阵)
 *@Function ：mat_inv     (原始矩阵,逆矩阵)
 */
#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32


/*
 *@param raw_value:原始值（2019.5.9我现在还不太清楚这个值是干嘛的，感觉没什么实际作用）
 *@param filtered_value：滤波器返回值
 *@param xhat:后验状态矩阵（就是需要用的融合模型和传感器的估计值）
 *@param xhatminus：先验状态矩阵（通过模型计算出的预测值）
 *@param z：观测值
 *@param A：状态转移矩阵（模型矩阵）
 *@param H：状态量到观测量的转换矩阵（就是状态量和观测量的关系）
 *@param AT：A的转置
 *@param HT：H的转置
 *@param Q：过程噪声方差矩阵
 *@param P：后验状态方差矩阵 
 *@param R：测量方差矩阵
 *@param Pminus:先验状态方差矩阵
 *@param K：卡尔曼增益矩阵
*/
typedef struct
{
  float raw_value;
  float filtered_value[4];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
	s16 Yaw_angle[KALMAN_FILTER_PAST_TIME],Pitch_angle[KALMAN_FILTER_PAST_TIME],Speed[KALMAN_FILTER_PAST_TIME],ArmorAngle;
  float Kalman_Filter_Yaw,Kalman_Filter_Pitch,Kalman_Filter_Speed,Kalman_Filter_CenterAngle;
} kalman_filter_t;


/*
 *@breif : 第一次状态更新的状态值，其实也是对矩阵的一个初始化
 *@param raw_value:原始值（2019.5.9我现在还不太清楚这个值是干嘛的，感觉没什么实际作用）
**********************************************************************
********************直接初始化为0的矩阵*******************************
**********************************************************************
 *@param Pminus_data    ：先验状态方差数据
 *@param z_data         ：观测值
 *@param K_data         ：卡尔曼增益数据
 *@param xhatminus_data ：先验估计值

**********************************************************************
*********************需要自己赋值的矩阵*******************************
**********************************************************************
 *@param xhat_data      : 后验估计值（第一次的观测值）
 *@param P_data         ：后验状态方差数据（第一次的观测误差）
 *@param A_data         ：状态转移矩阵数据（传递模型函数）
 *@param H_data         ：状态量到观测量的转换矩阵（观测量到状态量的转换关系）
 *@param Q_data         ：过程噪声数据（过程噪声置方差）
 *@param R_data         ：测量噪声数据（观测噪声方差）
 *@param AT_data        ：转置
 *@param HT_data        ：转置
 *@param filtered_value ：第一次滤波器返回值（第一次的后验估计值）
*/
typedef struct
{
  float raw_value;
  float filtered_value[4];
  float xhat_data[4], xhatminus_data[4], z_data[4],Pminus_data[16], K_data[16];
  float P_data[16];
  float AT_data[16], HT_data[16];
  float A_data[16];
  float H_data[16];
  float Q_data[16];
  float R_data[16];
} kalman_filter_init_t;

void kalman_fiflter_set(float yaw_angle, float pitch_angle);
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F);

extern kalman_filter_t KF;
