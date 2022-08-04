#include "universal.h"


void M3508_Output(s16 cm1_iq,s16 cm2_iq,s16 cm3_iq,s16 cm4_iq);
void Gimbal_Output(s16 gm1_iq,s16 gm2_iq,s16 gm3_iq);
void Motor_Data_Receive(void);
void Motor_Data_Deal(Motor_t *Receive,u8 Data[]);
void GM2006_Position_Deal(Motor_t *Receive);
void Cap_Output(uint16_t temPower);

extern float PowerData[4];
