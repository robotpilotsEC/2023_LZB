#ifndef __PID_H
#define __PID_H
#include "main.h"
#include "pid_conf.h"

typedef struct __pid_type_t
{
    float Kp;
    float Ki;
    float Kd;
	  float	integral;

	  float max_integral;
    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
	  float last_out;
    float Pout;
    float Iout;
    float Dout;
	
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
		
} pid_type_t;

typedef struct __pid_t{
	pid_type_t	speed;
	pid_type_t	angle;
	float		speed_out;
	float   angle_out;
} pid_t;

float PID_Inc_Calc(pid_type_t *pid, float fdb, float set);
float PID_Plc_Calc(pid_type_t *pid, float fdb, float set);
float PID_Hbt_Calc(pid_type_t *pid, float fdb, float set);
float PID_plc_Calc(pid_type_t *pid, float fdb, float set);

extern void PID_Init(pid_type_t *pid);

#endif
