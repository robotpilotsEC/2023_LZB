#include "pid.h"


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Init(pid_type_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
		pid->integral = 0;
}

// 位置式PID
float PID_Plc_Calc(pid_type_t *pid, float fdb, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

		//float PIout = 0.0f;
		
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
		
		//数据更新
		
    pid->error[0] = set - fdb;
		//arm_sub_f32(&set, &fdb, &pid->error[0], 1);
		pid->integral += pid->error[0];
		//arm_add_f32(&pid->error[0], &pid->integral, &pid->integral, 1);
		LimitMax(pid->integral, pid->max_integral);
		
		//分别计算PID
		//output = kp * Error + ki * Integral + kd * DError
		
    pid->Pout = pid->Kp * pid->error[0];
		//arm_mult_f32(&pid->Kp, &pid->error[0], &pid->Pout, 1);
    pid->Iout = pid->Ki * pid->integral;
		//arm_mult_f32(&pid->Ki, &pid->integral, &pid->Iout, 1);
		LimitMax(pid->Iout, pid->max_iout);
		
		pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
		pid->Dbuf[1] = pid->error[0] - pid->error[1];
		pid->Dbuf[2] = pid->error[1] - pid->error[2];
		//arm_sub_f32(&pid->error[0], &pid->error[1], &pid->Dbuf[1], 1);
		//arm_sub_f32(&pid->error[1], &pid->error[2], &pid->Dbuf[2], 1);
		//arm_sub_f32(&pid->Dbuf[2], &pid->Dbuf[1], &pid->Dbuf[0], 1);
		
		pid->Dout = pid->Kd * pid->Dbuf[1];
		//arm_sub_f32(&pid->Kd, &pid->Dbuf[1], &pid->Dout, 1);
		
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		//arm_add_f32(&pid->Pout, &pid->Iout, &PIout, 1);
		//arm_add_f32(&PIout, &pid->Dout, &pid->out, 1);
		
		LimitMax(pid->out, pid->max_out);
    
		pid->last_out = pid->out;
		
    return pid->out;
}

float PID_plc_Calc(pid_type_t *pid, float fdb, float set)
{
	  if (pid == NULL)
    {
        return 0.0f;
    }

		//float PIout = 0.0f;
		
		pid->max_integral = 5000.0f;
		pid->max_out = 10000.0f;
		
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
		
		//数据更新
		
    pid->error[0] = set - fdb;
		//arm_sub_f32(&set, &fdb, &pid->error[0], 1);
		pid->integral += pid->error[0];
		//arm_add_f32(&pid->error[0], &pid->integral, &pid->integral, 1);
//		LimitMax(pid->integral, pid->max_integral);
		
		//分别计算PID
		//output = kp * Error + ki * Integral + kd * DError
		
    pid->Pout = pid->Kp * pid->error[0];
		//arm_mult_f32(&pid->Kp, &pid->error[0], &pid->Pout, 1);
    pid->Iout = pid->Ki * pid->integral;
		//arm_mult_f32(&pid->Ki, &pid->integral, &pid->Iout, 1);
//		LimitMax(pid->Iout, pid->max_iout);
		
		pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
		pid->Dbuf[1] = pid->error[0] - pid->error[1];
		pid->Dbuf[2] = pid->error[1] - pid->error[2];
		//arm_sub_f32(&pid->error[0], &pid->error[1], &pid->Dbuf[1], 1);
		//arm_sub_f32(&pid->error[1], &pid->error[2], &pid->Dbuf[2], 1);
		//arm_sub_f32(&pid->Dbuf[2], &pid->Dbuf[1], &pid->Dbuf[0], 1);
		
		pid->Dout = pid->Kd * pid->Dbuf[1];
		//arm_sub_f32(&pid->Kd, &pid->Dbuf[1], &pid->Dout, 1);
		
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		//arm_add_f32(&pid->Pout, &pid->Iout, &PIout, 1);
		//arm_add_f32(&PIout, &pid->Dout, &pid->out, 1);
		
		LimitMax(pid->out, pid->max_out);
    
		pid->last_out = pid->out;
		
    return pid->out;
}

// 抗积分饱和式PID
float PID_Hbt_Calc(pid_type_t *pid, float fdb, float set)
{
	  if (pid == NULL)
    {
        return 0.0f;
    }

		//float PIout = 0.0f, temp = 0.f;
		
		pid->max_integral = 5000.0f;
		pid->max_out = 10000.0f;
		
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = fdb;
		
		//数据更新
		
    pid->error[0] = set - fdb;
		//arm_sub_f32(&set, &fdb, &pid->error[0], 1);
		pid->integral += pid->error[0];
		//arm_add_f32(&pid->error[0], &pid->integral, &pid->integral, 1);
		
		//分别计算PID
		//output = kp * Error + ki * Integral + kd * DError
		
    pid->Pout = pid->Kp * pid->error[0];
		//arm_mult_f32(&pid->Kp, &pid->error[0], &pid->Pout, 1);
    pid->Iout = pid->Ki * pid->integral;
		//arm_mult_f32(&pid->Ki, &pid->integral, &pid->Iout, 1);
//		LimitMax(pid->Iout, pid->max_iout);
		
		pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
		pid->Dbuf[1] = pid->error[0] - pid->error[1];
		pid->Dbuf[2] = pid->error[1] - pid->error[2];
		//arm_sub_f32(&pid->error[0], &pid->error[1], &pid->Dbuf[1], 1);
		//arm_sub_f32(&pid->error[1], &pid->error[2], &pid->Dbuf[2], 1);
		//arm_sub_f32(&pid->Dbuf[2], &pid->Dbuf[1], &pid->Dbuf[0], 1);
		
		pid->Dout = pid->Kd * pid->Dbuf[1];
		//arm_sub_f32(&pid->Kd, &pid->Dbuf[1], &pid->Dout, 1);
		
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		//arm_add_f32(&pid->Pout, &pid->Iout, &PIout, 1);
		//arm_add_f32(&PIout, &pid->Dout, &pid->out, 1);
		
		pid->last_out = pid->out;
		
		LimitMax(pid->out, pid->max_out);
    
		if (pid->out > pid->max_out)
		{
			pid->integral += (pid->max_out - pid->last_out) * 1/pid->Kp;
			
//			//temp = pid->max_out - pid->last_out
//			arm_sub_f32(&pid->max_out, &pid->last_out, &temp, 1);
//			//temp *= 1/kp
//			arm_scale_f32(&temp, 4/pid->Kp, &temp, 1);
//			//pid->integral += temp
//			arm_add_f32(&pid->integral, &temp, &pid->integral, 1);
		}
		else if (pid->out < -pid->max_out)
		{
			pid->integral += (pid->max_out - pid->last_out) * 1/pid->Kp;
			
//			//temp = pid->max_out - pid->last_out
//			arm_sub_f32(&pid->max_out, &pid->last_out, &temp, 1);
//			//temp *= 1/kp
//			arm_scale_f32(&temp, 4/pid->Kp, &temp, 1);
//			//pid->integral += temp
//			arm_add_f32(&pid->integral, &temp, &pid->integral, 1);
		}
    return pid->out;
}

// 增量式PID
//float PID_Inc_Calc(pid_type_t *pid, float fdb, float set)
//{
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }
//		
//		float PIout = 0.0f;
//		float PIDout = 0.0f;
//		
//		pid->error[2] = pid->error[1];
//    pid->error[1] = pid->error[0];
//    pid->set = set;
//    pid->fdb = fdb;
//		
//		//数据更新
//		
//    //pid->error[0] = set - fdb;
//		arm_sub_f32(&set, &fdb, &pid->error[0], 1);
//		
//		//pid->Dbuf[0] = pid->error[0] - pid->error[1]; //0 误差增量 1 误差增量的增量 2 上一次误差增量
//		arm_sub_f32(&pid->error[0], &pid->error[1], &pid->Dbuf[0], 1);
//		
//		//pid->Dbuf[1] = pid->Dbuf[0] - pid->Dbuf[2];
//		arm_sub_f32(&pid->Dbuf[0], &pid->Dbuf[2], &pid->Dbuf[1], 1);
//		pid->Dbuf[2] = pid->Dbuf[0];
//		
//		//分别计算PID
//		//output = kp * DError + ki * Error + kd * DDError
//		
//		//pid->Pout = pid->Kp * pid->Dbuf[0];
//		arm_mult_f32(&pid->I_Kp, &pid->Dbuf[0], &pid->Pout, 1);
//		
//		//pid->Iout = pid->Ki * pid->error[0];
//		arm_mult_f32(&pid->I_Ki, &pid->error[0], &pid->Iout, 1);
//		LimitMax(pid->Iout, pid->max_iout);
//		
//		//pid->Dout = pid->Kd * pid->Dbuf[1];
//		arm_mult_f32(&pid->I_Kd, &pid->Dbuf[1], &pid->Dout, 1);
//		
//		//pid->out += pid->Pout + pid->Iout + pid->Dout;
//		arm_add_f32(&pid->Pout, &pid->Iout, &PIout, 1);
//		arm_add_f32(&PIout, &pid->Dout, &PIDout, 1);
//		arm_add_f32(&PIDout, &pid->out, &pid->out, 1);
//		
//		LimitMax(pid->out, pid->max_out);
//    
//		pid->last_out = pid->out;
//		
//    return pid->out;
//}

