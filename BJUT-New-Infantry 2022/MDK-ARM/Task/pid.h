#include "universal.h"

#define Incremental 0
#define Positional  1



typedef struct
{
    //PID ������
    float Kp;
    float Ki;
    float Kd;
		//������ ����
		float Max_out;  //������
		float Dead_band;//PIDƫ������
		float Intergral_band;//������
		float Max_input;//�������
    //PID���ֵ
		float Input;//����
    float Output;//���
		float I_max_out;//�����������
    //���
    float E[3];//2���� 1��һ�� 0���ϴ�
		float D_last;
		float Intergral;//����ֵ
	  u8    Mode;//ģʽѡ��λ��PID������PID
} PidTypeDef;



void PID_Init(PidTypeDef * pid,float kp,float ki,float kd,float max_out,float dead_band,float i_band,float max_input,float i_max_out,u8 mode);
void PID_Calc(PidTypeDef * pid, float real_val, float set_val);
void Chassis_Follow_PD_Calc(PidTypeDef * pid, float real_val, float set_val);
void PID_Clear(PidTypeDef * pid);
s16 Yaw_Feedforward_Controler(s16 set);
s16 Pitch_Feedforward_Controler(s16 set);

extern PidTypeDef M3508_Speed_pid[4];
extern PidTypeDef Gimbal_Speed_pid[3];
extern PidTypeDef Gimbal_Position_pid[3];
extern PidTypeDef Gimbal_TCurrent_pid[3];
extern PidTypeDef Chassis_Rotate_pid;
extern PidTypeDef Power_Limit_pid;
extern PidTypeDef Spin_Top_pid;
extern PidTypeDef Pitch_Stable_pid;

