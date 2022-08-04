/**
  ******************************************************************************
  * @file    Config.h
  * @author  北京工业大学-贾桐
  * @version V1.0
  * @date    2019/3/1
  * @brief   参数配置文件
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */


#define GAME_MODE                        1                  // 1为参赛模式 0为常用模式，区别在于是否检测模块掉线后是否复位
#define YAW_USE_IMUEX                    0                  // 0 YAW轴使用A板陀螺仪 1 TAW轴使用外挂陀螺仪
#define YAW_USE_CURRENT                  0                  // 0 YAW不使用转矩环 1 YAW使用转矩环
#define PITCH_POSITION_USE_IMU           0                  // 0 PITCH位置反馈来自编码器 1 PITCH反馈来自IMU , 暂时没用
#define PITCH_SPEED_USE_IMU              0                  // 0 PITCH速度反馈来自编码器 1 PITCH反馈来自IMU
#define PITCH_STABLE                     0                  // 0 PITCH轴不垂稳 1 PITCH轴垂稳
#define PITCH_USE_ADRC                   1                  // 0 PITCH使用串级PID 1 PITCH使用ADRC
#define PITCH_USE_CURRENT                1                  // 0 PITCH不使用转矩环 1 PITCH使用转矩环
/************************************************************************************************************/
/**************************************************控制参数**************************************************/
/************************************************************************************************************/
#define REMOTE_SPEED_ZOOM                12                 //遥控器拨杆放大系数
//#define MOUSE_SENSITIVITY_YAW            0.2f              //鼠标灵敏度
//#define MOUSE_SENSITIVITY_PITCH          0.2f
/************************************************************************************************************/
/**************************************************底盘参数**************************************************/
/************************************************************************************************************/
#define CHASSIS_MAX_SPEED                7000               //最大移动速度
#define M3508_RAMP_RATIO_MAX             40                //3508上升斜坡（可考虑优化为抛物线）
#define M3508_RAMP_RATIO_MIN             10
#define CHASSIS_RAMP_RATIO               32                 //底盘斜坡
#define TWIST_RAMP_RATIO                 50                 //扭腰斜坡
#define TWIST_RANGE                      1000               //扭腰的范围
#define ROTATE_NORMAL_SPEED              2000               //小陀螺旋转速度
#define ROTATE_LOW_SPEED                 1100               //小陀螺旋转速度
#define ROTATE_HIGH_SPEED                3500               //小陀螺旋转速度

/************************************************************************************************************/
/**************************************************云台参数**************************************************/
/************************************************************************************************************/
#define KALMAN_FILTER_PAST_TIME          15                 //云台使用以前的数据，进行卡尔曼滤波
//YAW参数
#define YAW_POSITION_RAMP_RATIO          13                 //YAW斜坡
#define YAW_POSITION_INIT                2767                //YAW电机初始位置值
#define YAW_POSITION_MAX                 YAW_POSITION_INIT + 1200 //暂时未使用
#define YAW_POSITION_MIN								 YAW_POSITION_INIT - 1200 //暂时未使用

//PITCH参数
#define PITCH_POSITION_RAMP_RATIO        7                 //PITCH
#define PITCH_POSITION_MAX               7280
#define PITCH_POSITION_MIN               6380

#if PITCH_POSITION_USE_IMU == 1
#define PITCH_POSITION_INIT              4096               //PITCH电机初试位置值
#elif PITCH_POSITION_USE_IMU == 0
#define PITCH_POSITION_INIT              6964               //PITCH电机初试位置值
#endif
//TRIGGER参数
#define TRIGGER_POSITION_RAMP_RATIO      34                 //斜坡
//底盘选择
#define CHASSIS_CHOOSE 1     															  //0为底盘选择功率优先  1为底盘选择血量优先






