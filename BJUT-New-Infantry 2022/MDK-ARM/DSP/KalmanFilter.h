/**
  ******************************************************************************
  * @file    KalmanFilter.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/5/9
  * @brief   �������˲���
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
 *@Function ��mat_init    (mat,��,��,����)
 *@Function ��mat_add     (A,B,���)
 *@Function ��mat_sub     (A,B,���)
 *@Function ��mat_mult    (A,B,���)
 *@Function ��mat_trans   (ԭʼ����,ת�þ���)
 *@Function ��mat_inv     (ԭʼ����,�����)
 */
#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32


/*
 *@param raw_value:ԭʼֵ��2019.5.9�����ڻ���̫������ֵ�Ǹ���ģ��о�ûʲôʵ�����ã�
 *@param filtered_value���˲�������ֵ
 *@param xhat:����״̬���󣨾�����Ҫ�õ��ں�ģ�ͺʹ������Ĺ���ֵ��
 *@param xhatminus������״̬����ͨ��ģ�ͼ������Ԥ��ֵ��
 *@param z���۲�ֵ
 *@param A��״̬ת�ƾ���ģ�;���
 *@param H��״̬�����۲�����ת�����󣨾���״̬���͹۲����Ĺ�ϵ��
 *@param AT��A��ת��
 *@param HT��H��ת��
 *@param Q�����������������
 *@param P������״̬������� 
 *@param R�������������
 *@param Pminus:����״̬�������
 *@param K���������������
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
 *@breif : ��һ��״̬���µ�״ֵ̬����ʵҲ�ǶԾ����һ����ʼ��
 *@param raw_value:ԭʼֵ��2019.5.9�����ڻ���̫������ֵ�Ǹ���ģ��о�ûʲôʵ�����ã�
**********************************************************************
********************ֱ�ӳ�ʼ��Ϊ0�ľ���*******************************
**********************************************************************
 *@param Pminus_data    ������״̬��������
 *@param z_data         ���۲�ֵ
 *@param K_data         ����������������
 *@param xhatminus_data ���������ֵ

**********************************************************************
*********************��Ҫ�Լ���ֵ�ľ���*******************************
**********************************************************************
 *@param xhat_data      : �������ֵ����һ�εĹ۲�ֵ��
 *@param P_data         ������״̬�������ݣ���һ�εĹ۲���
 *@param A_data         ��״̬ת�ƾ������ݣ�����ģ�ͺ�����
 *@param H_data         ��״̬�����۲�����ת�����󣨹۲�����״̬����ת����ϵ��
 *@param Q_data         �������������ݣ����������÷��
 *@param R_data         �������������ݣ��۲��������
 *@param AT_data        ��ת��
 *@param HT_data        ��ת��
 *@param filtered_value ����һ���˲�������ֵ����һ�εĺ������ֵ��
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
