/**
  ******************************************************************************
  * @file    KalmanFilter.c
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/5/9
  * @brief   卡尔曼滤波器
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "KalmanFilter.h"
#include "universal.h"

kalman_filter_t KF;
kalman_filter_init_t KF_Init;

/**
  * @brief  对矩阵赋值                                                     矩阵 X(t)  0  △t * V(x)	
  * @param  yaw_angle   ：两个角度值作为第一次的后验估计值                       0   Y(t)  0
	* @param  pitch_angle ：                                                       0    0   V(x)
  * @retval None
  */
void kalman_fiflter_set(float yaw_angle, float pitch_angle)
{
	KF_Init.A_data[0] = 1;//状态转移矩阵初始化 X(t)
	KF_Init.A_data[5] = 1;//Y(t)
	KF_Init.A_data[10] = 1;//Vx 
	KF_Init.A_data[15] = 1;//ArmorAngle 
	
	KF_Init.H_data[0] = 1;//状态量到观测量转移矩阵
	KF_Init.H_data[5] = 1;
	KF_Init.H_data[10] = 1;
	KF_Init.H_data[15] = 1;
	
//	KF_Init.Q_data[0] = 200;//过程噪声协方差矩阵 不动装甲板滤波
//	KF_Init.Q_data[5] = 200;
//	KF_Init.Q_data[10] = 200;
//	KF_Init.Q_data[15] = 200;

	KF_Init.Q_data[0] = 400;//过程噪声协方差矩阵  大符滤波
	KF_Init.Q_data[5] = 400;
	KF_Init.Q_data[10] = 400;
	KF_Init.Q_data[15] = 400;
	
	KF_Init.R_data[0] = 40000;//观测噪声协方差矩阵 大符滤波
	KF_Init.R_data[5] = 40000;
	KF_Init.R_data[10] = 400000;	
	KF_Init.R_data[15] = 400000;
	
//	KF_Init.R_data[0] = 1700;//观测噪声协方差矩阵 不动装甲板滤波
//	KF_Init.R_data[5] = 1700;
//	KF_Init.R_data[10] = 1700;	
//	KF_Init.R_data[15] = 1700;
	
//	KF_Init.P_data[0] = 400;//第一次后验估计方差
//	KF_Init.P_data[5] = 400;
//	KF_Init.P_data[10] = 400;
//	KF_Init.P_data[15] = 400;
	
	KF_Init.P_data[0] = 400;//第一次后验估计方差
	KF_Init.P_data[5] = 400;
	KF_Init.P_data[10] = 400;
	KF_Init.P_data[15] = 400;
	
	KF_Init.xhat_data[0] = 0;//第一次后验估计值
	KF_Init.xhat_data[1] = 0;
	KF_Init.xhat_data[2] = 0;
	KF_Init.xhat_data[3] = 0;
	for(u8 i = 0;i < KALMAN_FILTER_PAST_TIME;i++)
	{
		KF.Yaw_angle[i]   = yaw_angle;
		KF.Pitch_angle[i] = pitch_angle;
	}
	kalman_filter_init(&KF,&KF_Init);
}
/**
  * @brief  对卡尔曼滤波器初始化，对mat矩阵赋值，第一次观测准备
						这里的调用，要先对结构体初始化，再初始化矩阵，不然矩阵会为0
  * @param  kalman_filter_t      ：要初始化的滤波器
	* @param  kalman_filter_init_t ：初始化的数值
  * @retval None
  */
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
	//对这12个矩阵初始化，参数意义写在.h文件里
  mat_init(&F->xhat     ,4,1,(float *)I->xhat_data);
	mat_init(&F->xhatminus,4,1,(float *)I->xhatminus_data);
	mat_init(&F->z        ,4,1,(float *)I->z_data);
	mat_init(&F->Q        ,4,4,(float *)I->Q_data);
	mat_init(&F->R        ,4,4,(float *)I->R_data);
	mat_init(&F->P        ,4,4,(float *)I->P_data);
	mat_init(&F->Pminus   ,4,4,(float *)I->Pminus_data);
	mat_init(&F->K				,4,4,(float *)I->K_data);	
  mat_init(&F->A				,4,4,(float *)I->A_data);
	mat_init(&F->H				,4,4,(float *)I->H_data);
	mat_init(&F->AT				,4,4,(float *)I->AT_data);
	mat_init(&F->HT				,4,4,(float *)I->HT_data);
	//对两个需要计算的矩阵求逆
  mat_trans(&F->A, &F->AT); 
  mat_trans(&F->H, &F->HT);
}

/**
  * @brief  滤波器迭代更新，q这是一个两状态变量的滤波器，多变量的需要更改参数
  * @param  kalman_filter_t      ：要更新的滤波器
	* @param  signal1              ：信号值1
	* @param  signal2              ：信号值2
  * @retval filtered_value       ：后验估计值
  */
float *kalman_filter_calc(kalman_filter_t *F)
{
  float TEMP_data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float TEMP_data41[4] = {0,0,0,0};
	float identity[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};//这个是单位矩阵，后边的数值，我是真没想好怎么改，自己手动改一下吧
  mat TEMP,TEMP41,IDENTITY;
	
	mat_init(&IDENTITY,4,4,(float *)identity);
  mat_init(&TEMP,4,4,(float *)TEMP_data);
  mat_init(&TEMP41,4,1,(float *)TEMP_data41);

  F->z.pData[0] = F->Kalman_Filter_Yaw;
  F->z.pData[1] = F->Kalman_Filter_Pitch;
  F->z.pData[2] = F->Kalman_Filter_Speed;	
  F->z.pData[3] = F->Kalman_Filter_CenterAngle;		
//1. xhat'(k)= A xhat(k-1)   
//状态先验估计矩阵(xhatminus) = 状态空间矩阵(A)*后验估计值(xhat)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);
//2. P'(k) = A P(k-1) AT + Q  
//先验估计方差矩阵(Pminus) = 状态空间矩阵(A)*后验估计方差矩阵(P)*状态转移矩阵转置(AT)+过程噪声矩阵(Q)
  mat_mult(&F->A, &F->P, &F->Pminus); 
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

//3. K(k) = P'(k) HT / (H P'(k) HT + R)  
//增益矩阵(K) = 先验估计方差矩阵(Pminus)*状态转移矩阵转置(HT)/(状态转移矩阵(H)*先验估计方差矩阵(Pminus)*状态转移矩阵转置(HT)+测量噪声矩阵(R))
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
//状态后验估计矩阵(xhat) = 状态先验估计矩阵(xhatminus)+增益矩阵(K)*(观测矩阵(z)-状态转移矩阵(H)*状态先验估计矩阵(xhatminus))
  mat_mult(&F->H, &F->xhatminus, &TEMP41);
  mat_sub(&F->z, &TEMP41, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP41);
  mat_add(&F->xhatminus, &TEMP41, &F->xhat);

//5. P(k) = (1-K(k)H)P'(k)
//后验估计方差矩阵(P) = (1 - 增益矩阵(K)*状态转移矩阵(H))*先验估计方差矩阵(Pminus)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&IDENTITY, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  F->filtered_value[2] = F->xhat.pData[2];
  F->filtered_value[3] = F->xhat.pData[3];	
  return F->filtered_value;
}

//1. xhat'(k)= A xhat(k-1)   
//状态先验估计矩阵(xhatminus) = 状态空间矩阵(A)*后验估计值(xhat)

//2. P'(k) = A P(k-1) AT + Q  
//先验估计方差矩阵(Pminus) = 状态空间矩阵(A)*后验估计方差矩阵(P)*状态转移矩阵转置(AT)+过程噪声矩阵(Q)

//3. K(k) = P'(k) HT / (H P'(k) HT + R)  
//增益矩阵(K) = 先验估计方差矩阵(Pminus)*状态转移矩阵转置(HT)/(状态转移矩阵(H)*先验估计方差矩阵(Pminus)*状态转移矩阵转置(HT)+测量噪声矩阵(R))

//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
//状态后验估计矩阵(xhat) = 状态先验估计矩阵(xhatminus)+增益矩阵(K)*(观测矩阵(z)-状态转移矩阵(H)*状态先验估计矩阵(xhatminus))

//5. P(k) = (1-K(k)H)P'(k)
//后验估计方差矩阵(P) = (1 - 增益矩阵(K)*状态转移矩阵(H))*先验估计方差矩阵(Pminus)
