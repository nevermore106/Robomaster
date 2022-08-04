/**
  ******************************************************************************
  * @file    Config.h
  * @author  ������ҵ��ѧ-��ͩ
  * @version V1.0
  * @date    2019/3/1
  * @brief   ���������ļ�
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */


#define GAME_MODE                        1                  // 1Ϊ����ģʽ 0Ϊ����ģʽ�����������Ƿ���ģ����ߺ��Ƿ�λ
#define YAW_USE_IMUEX                    0                  // 0 YAW��ʹ��A�������� 1 TAW��ʹ�����������
#define YAW_USE_CURRENT                  0                  // 0 YAW��ʹ��ת�ػ� 1 YAWʹ��ת�ػ�
#define PITCH_POSITION_USE_IMU           0                  // 0 PITCHλ�÷������Ա����� 1 PITCH��������IMU , ��ʱû��
#define PITCH_SPEED_USE_IMU              0                  // 0 PITCH�ٶȷ������Ա����� 1 PITCH��������IMU
#define PITCH_STABLE                     0                  // 0 PITCH�᲻���� 1 PITCH�ᴹ��
#define PITCH_USE_ADRC                   1                  // 0 PITCHʹ�ô���PID 1 PITCHʹ��ADRC
#define PITCH_USE_CURRENT                1                  // 0 PITCH��ʹ��ת�ػ� 1 PITCHʹ��ת�ػ�
/************************************************************************************************************/
/**************************************************���Ʋ���**************************************************/
/************************************************************************************************************/
#define REMOTE_SPEED_ZOOM                12                 //ң�������˷Ŵ�ϵ��
//#define MOUSE_SENSITIVITY_YAW            0.2f              //���������
//#define MOUSE_SENSITIVITY_PITCH          0.2f
/************************************************************************************************************/
/**************************************************���̲���**************************************************/
/************************************************************************************************************/
#define CHASSIS_MAX_SPEED                7000               //����ƶ��ٶ�
#define M3508_RAMP_RATIO_MAX             40                //3508����б�£��ɿ����Ż�Ϊ�����ߣ�
#define M3508_RAMP_RATIO_MIN             10
#define CHASSIS_RAMP_RATIO               32                 //����б��
#define TWIST_RAMP_RATIO                 50                 //Ť��б��
#define TWIST_RANGE                      1000               //Ť���ķ�Χ
#define ROTATE_NORMAL_SPEED              2000               //С������ת�ٶ�
#define ROTATE_LOW_SPEED                 1100               //С������ת�ٶ�
#define ROTATE_HIGH_SPEED                3500               //С������ת�ٶ�

/************************************************************************************************************/
/**************************************************��̨����**************************************************/
/************************************************************************************************************/
#define KALMAN_FILTER_PAST_TIME          15                 //��̨ʹ����ǰ�����ݣ����п������˲�
//YAW����
#define YAW_POSITION_RAMP_RATIO          13                 //YAWб��
#define YAW_POSITION_INIT                2767                //YAW�����ʼλ��ֵ
#define YAW_POSITION_MAX                 YAW_POSITION_INIT + 1200 //��ʱδʹ��
#define YAW_POSITION_MIN								 YAW_POSITION_INIT - 1200 //��ʱδʹ��

//PITCH����
#define PITCH_POSITION_RAMP_RATIO        7                 //PITCH
#define PITCH_POSITION_MAX               7280
#define PITCH_POSITION_MIN               6380

#if PITCH_POSITION_USE_IMU == 1
#define PITCH_POSITION_INIT              4096               //PITCH�������λ��ֵ
#elif PITCH_POSITION_USE_IMU == 0
#define PITCH_POSITION_INIT              6964               //PITCH�������λ��ֵ
#endif
//TRIGGER����
#define TRIGGER_POSITION_RAMP_RATIO      34                 //б��
//����ѡ��
#define CHASSIS_CHOOSE 1     															  //0Ϊ����ѡ��������  1Ϊ����ѡ��Ѫ������






