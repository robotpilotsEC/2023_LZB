#ifndef __CHASSIC_H
#define __CHASSIC_H

#include "device.h"
#include "algo.h"
#include "State.h"
#include "Master_Comm.h"

//-----------------------------底盘和动量块信息表--------------------------------
//状态变量测量值 |   变化趋势  
//
//车体位移			 |   往车头走 +
//车体速度			 |   往车头走 +
//动量块位移     |	 往车头走 +
//动量块速度     |   往车头走 +
//Pitch角度      |   往车前倾 -
//Pitch角速度    |   往车前倾 -
//Yaw角度        |   往左转   +
//Yaw角速度      |   往左转   +
//Roll角度			 |  往车头左边倾斜 + 
//Yaw电机        |   电池4096 ，前0，左2080，右 6185

//单个电机测量值 |   变化趋势    |    输出的电流/电压   |   计算得出的扭矩
//
//左9025位移速度 |   往车头走 +  |    往前转 -          |   往前转 -     
//右9025位移速度 |   往车头走 -  |    往前转 +          |   往前转 - 
//左6020位移速度 |   往车头走 +  |    往前转 -          |   往前转 - 
//右6020位移速度 |   往车头走 -  |    往前转 +          |   往前转 - 







/* Exported macro ------------------------------------------------------------*/
/*------------------------以下为其他要用的宏定义------------------------*/

#define STATUS_VAL_NUM  8
#define STRUCT_STATUS_UNNOT_UP_VAL_NUM   4  //status结构体中不要去更新误差的数据个数
#define CHASSIC_MAX_POWER_BUFFER    (judge_info.power_heat_data.chassis_power_buffer)         







/*-------------------------以下为底盘属性-----------------------------------*/\
#define CHASSIC_LENGTH			0.36f						//车头到车尾的距离
#define WHEEL_RADIUS        0.08f          	//轮子半径      m单位//0.1
#define WHEEL_GIRTH					( 2 * VALUE_PI * WHEEL_RADIUS ) //轮子周长 m单位
#define MOMENTUM_RADIUS			0.033f					//6020半径  
#define MOMENTUM_GIRTH      ( 2 * VALUE_PI * MOMENTUM_RADIUS ) //动量块周长
//动量块以中点为位移为0的点，往前最大的位移是车长
#define MOMENTUM_MAX_X_MEAS   (CHASSIC_LENGTH / 2)  

//动量快电机从后到前的电机角度和6590 
#define L_MOMENTUM_B_2_F_ANGLE_SUM	6150
#define R_MOMENTUM_B_2_F_ANGLE_SUM	6370 							


//实际速度转换为发送给电机的速度
//SPEED_CONTROL * 0.01 / 360 * 周长 = SPEED(m/s)
#define KT_SPEED_TO_SPEED_CONTROL(SPEED)   (SPEED / WHEEL_GIRTH * 360.f * 100.f)  


/*-----------------跟测距相关------------*/
//往前倒,前测距1.8cm,后测距25.6cm;  往后倒,前测距24cm,后测距2.1cm;  水平都11.5cm
#define CHAS_F_TOF_MIN_RAW_X				(1.8f)
#define CHAS_F_TOF_MAX_RAW_X				(25.6f)
#define CHAS_B_TOF_MIN_RAW_X				(2.1f)
#define CHAS_B_TOF_MAX_RAW_X				(24.f)
#define CHAS_Horizontal_Height      (11.5f)//11.5
#define STEP_HEIGHT									(23.f)		//台阶高度

//老车：地上最大13.6，飞坡最大12.6，15°坡最大13.6

/*------------------------以下为底盘限制的参数--------------------------------*/
#define H_X_MAX_START_SPEED      2.5f          //整车起步最大的线速度
#define M_X_MAX_START_SPEED      1.7f
#define L_X_MAX_START_SPEED      1.5f          //整车起步最大的线速度
#define Judge_Offline_X_Speed  	 0.9f          //裁判失联时底盘最大速度
#define SMALL_TOP_YAW_SPIN_RATE	  0.01f   //小陀螺yaw角度变化率
#define H_SPEED_CONTROL    			 0.6f
#define L_SPEED_CONTROL          0.2f

#define CHASSIC_MAX_PITCH_ANGLE   (6.f*ANGLE_CONVERSION_RADIAN) //最大的倾角姿态
#define MAX_SPEED_TORQUE    (	abs_(L_C_LQR_Param[P]) * CHASSIC_MAX_PITCH_ANGLE + \
															abs_(L_C_LQR_Param[M_X]) * MOMENTUM_MAX_X_MEAS ) //1个电机最大大概35扭矩
					 										

																				


//--以下皆为测试得出，猜测可以根据当前线速度的大小来线性拟合旋转速率
#define MAX_YAW_ANGLE_ERR			20        
#define SPIN_STATE_X_SPEED    0.15f       //旋转的时候线速度是大概为0.1左右，设置上限0.15  
#define ONLY_SPIN_RATE	 			0.02f			//原地旋转速率
#define SPIN_AS_BRAKING_RATE  0.02f     //刹车过程转弯速率
#define SPIN_AS_FORWARD_RATE	0.01f		//前进并旋转的速率






/* Exported types ------------------------------------------------------------*/




//状态变量在数组的索引号
typedef enum status_index
{
	C_X = 0,
	C_V,
	
	M_X,
	M_V,
	
	//pitch
	P,
	P1,
	
	//yaw
	Y = 6,
	Y1,
	
	STATUS_TOTAL,
	
}status_index_e;



//状态变量基本单位
typedef struct status
{
	//平均
	float  C_X;       
	float  C_V;		 
	
	//平均
	float  M_X;       
	float  M_V;	
		
	float  Pitch;            
	float  Rate_Pitch;   

	float  Yaw;
	float  Rate_Yaw;	
	
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
	uint16_t Y_Pos;
	uint16_t Y_Standard_Pos;
	
}	status_t;
 

//右边系统的轮子位移、速度、动量块速度加负号
typedef struct Sys_Status
{
	status_t meas;
	status_t err;
  status_t tar;
	
} Sys_Status_t;

//左边系统的轮子扭矩加负号
//对轮子：左-右=旋转计算的扭矩,左＋右=除去旋转外计算的扭矩
//对动量块，不算旋转得到的扭矩
typedef struct Sys_Input
{
	float Tx_Torque;
	float Rx_Torque;
	float Err_Torque;

	float Spd_Torque;
	float Balance_Torque;
	float Gravity_Torque;
	float Spin_Torque;
	
	float Current;
	float Voltage;
	
} Sys_Input_t;

typedef struct Gesture_TOF_Info
{
	//cm单位
	float L_Raw_X;
	float L_Vert_X;

	float R_Raw_X;
	float R_Vert_X;
	
}Gesture_TOF_Info_t;

typedef struct
{
	/*检测电机过温次数*/
	uint8_t L_M_Over_I_Time;
	uint8_t R_M_Over_I_Time;
	
	/*刹车瞬间的速度方向*/
	int  brake_over_dir;
	
	/*刹车瞬间开启小陀螺的时候,保证pitch和速度连续较低*/
	uint8_t low_pitch_cnt;
	uint8_t low_speed_cnt;
	
	/*卡墙次数*/
	uint8_t stuck_cnt;
	
	/*开启小陀螺前,先给一个斜坡输入*/
	uint8_t start_spin_flag;
	float   spin_val;
	uint8_t spin_handler_flag; //先知道此时旋转的时候偏向哪边，1偏前(pitch<0)2偏后(pitch>0)
	uint8_t spin_close_cnt; //脱落关闭时满足的次数
	uint8_t spin_err; //陀螺中出现错误
	
	/*真的在地上后进行一段延时来判断下一次的失重*/
	uint32_t bounce_delay_cnt;

	/*动量块失联后的控制方式*/
	uint8_t  momentum_offline;
	
	/*不同时下台阶*/
	uint8_t  roll_in_air_flag; 
	uint8_t  imu_off_ground; 
	 
	uint8_t  torque_compensation_request; //落地扭矩补偿
	uint8_t  first_bounce; //1表示第一次落地了
	
	uint8_t  sideways_request; //斜坡输入
	uint8_t  sideways_over; //侧身结束了，用来修改底盘的朝向
	uint8_t  sideways_dir; //侧身旋转方向
	
}User_typedef_t;


typedef struct Chassic_Info
{
	
	Sys_Status_t 	Status;
	
	Gesture_TOF_Info_t 	TOF_Meas;
	
	//底盘左电机，右电机；动量块左电机，右电机
	Sys_Input_t   C_L_M;
	Sys_Input_t   C_R_M;
	Sys_Input_t   M_L_M;
	Sys_Input_t   M_R_M;
	
	//选用数组是因为在处理悬空时要在state.c文件修改增益，但又不希望State文件包含chassis文件
	float         C_LQR_Gain[STATUS_VAL_NUM];
	float         M_LQR_Gain[STATUS_VAL_NUM];
	
}Chassic_Info_t;

typedef struct Chassic
{
	Chassic_Info_t       info;
	User_typedef_t       users;
	
	void                (*ctrl)(void);		//总控制	
	
}Chassic_t;


__weak void Chassic_Flag_Set(const char flag);
__weak void Chas_Show_UI_Stuck(const char flag);
void Chassic_Stop(void);


//负号依次:(车体的)- -  (动量快的)- -   (角度的)+ +  (yaw)- -
extern float  H_C_LQR_Param[STATUS_VAL_NUM];
extern float  L_C_LQR_Param[STATUS_VAL_NUM];
//符号依次:(车体的)+ +  (动量快的)+ +   (角度的)- -  (yaw)0 0
extern float  H_M_LQR_Param[STATUS_VAL_NUM];
extern float  L_M_LQR_Param[STATUS_VAL_NUM];
extern Chassic_t chassic;

#endif
