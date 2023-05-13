#ifndef __Under_Layer_H
#define __Under_Layer_H

/*流程
1.设备层提供测量数据,保存到device_info_t包中
2.系统目标值设置
3.系统测量值设置
4.系统误差值设置

*/



/*---------------------------Pravited  Include-----------------------------*/
#include "device.h"
#include "stdarg.h"

/*---------------------------Pravited  Define---------------------------*/



/*---------------------------Pravited  Typedef--------------------------*/

/*状态变量*/
typedef struct status
{
	//平均
	float  C_X;       
	float  C_V;		 
	
	//平均
	float  M_X;       
	float  M_V;	
		
	float  Pitch;            
	float  Pitch_1;   

	float  Yaw;
	float  Yaw_1;		
	
	float  Roll;
	float  Roll_1;
	
}	sys_status_t;

/*用户还想更新的数据可在这里添加*/
typedef struct
{
	//----两个轮子各自位移速度加速度
	float  C_X_L;
	float  C_V_L;
	
	float  C_X_R;
	float  C_V_R;

	float  M_X_L;
	float  M_V_L;
	
	float  M_X_R;
	float  M_V_R;
	
	//----两个动量快各自位置位移速度
	uint16_t L_M_Pos; //只用测量值，目标误差不管	
	uint16_t R_M_Pos; //只用测量值，目标误差不管	
	
	//----YAW电机原始角度,规定化后的角度
	uint16_t Y_Raw_Pos;
	uint16_t Y_Standard_Pos;
	
} device_info_t;


typedef struct
{
	sys_status_t sys_tar;
	sys_status_t sys_meas;
	sys_status_t sys_err;
	
} sys_status_pack_info_t;

#endif
