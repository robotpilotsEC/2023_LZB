#ifndef __POWER_LIMIT_H
#define __POWER_LIMIT_H


#include "rp_math.h"



#define  CHAS_MOTOR_MAX_SPEED_TORQUE   15  //1个电机最大速度扭矩15
#define  CHAS_MID_POWER_BUFFER   30
#define  CHAS_MAX_POWER_BUFFER   60

void Chassis_Motor_Power_Limit(float *data,float buffer);

#endif

