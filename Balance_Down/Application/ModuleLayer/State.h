#ifndef __State_H
#define __State_H


#include "stm32f4xx.h"                  // Device header
#include "rp_config.h"
#include "rp_math.h"



#define    NO_LINK_TO_RECEIVE		
//#define    LINK_TO_RECEIVE

//如果下主控接了接收机,那么就要把上主控发的有关遥控器标志,遥控器通道值更新进行关闭
//但是还是要进行通讯



#define    RC_ONLINE             (State.rc_flag.work_state == DEV_ONLINE)
#define    RC_OFFLINE            (State.rc_flag.work_state == DEV_OFFLINE)
#define    FAIL_COMM						 (State.comm_flag.state_flag == COMM_FAIL)
#define    SUCCESS_COMM					 (State.comm_flag.state_flag == COMM_OK)

//---------------------------------------------枚举类型
typedef enum
{
	RC_WORK_STATE_BIT = 0,	//rc_work_state, 0关1开
	
	MAGZ_FLAG_BIT, 					//magazine_state,0关1开
	
	SMALL_TOP_FLAG_BIT,    //小陀螺,0关1开
	
	GIMBAL_HEADER_LOW_BIT,     //头的位置,0:7191(前面);   1:3095(后面)    
	 
	GIMBAL_TURN_BIT,       //换头是否完成,1在换(包括上电时头复位这个过程);0换完了
	
	GIMBAL_AUTO_BIT,       //自瞄0关1开
	
	SYS_RESET_BIT,         //系统复位0关1开

	SIDEWAYS_BIT,         //侧身0关1开
	
	SPEED_UP_BIT,         //加速0关1开
	
	DAFU_MODE_BIT,        //打符模式下，底盘侧身不跟头了
	
	GIMBAL_IMU_STATE_BIT,     //云台陀螺仪状态，0正常1不正常
	
	GIMBAL_HEADER_HIGH_BIT, 	//头的位置,0:左，1:右    
	
}comm_rx_status_bit_e;


typedef enum
{
	COMM_OK = 0, //有数据且正常
	
	COMM_FAIL,//没数据
	
	
	//有数据但是数据有误
	//用于串口1校验检查错误
	FRAME_HEADER_ERR,
	CRC8_ERR,
	CRC16_ERR,
	
}comm_state_flg_e;



typedef enum
{
	CLOSE,
	
	OPEN,
	
	SWITCH_UP,//开机
	SWITCH_DOWN,//关机
	NORMAL_WORK,//正常工作,只有等陀螺仪校正完成才设置该标志位
	
}switch_e;

typedef enum
{
	//is_in_air_flag
	Fly_Out_Of_Ground = 2,//飞出离地
	Bounce_Up_Of_Ground,//弹起离地
	ON_GROUND,
	
	//now_state
	ALINE,//对正
	FALL,//倒下
	SPIN,//旋转
	DAFU, //打符模式
	STUCK,  //卡墙
	SIDEWAYS, //侧身
	
	//chas_motion_mode
	SMALL_TOP_OFF,
	SMALL_TOP_ON,

	//gimb_motion_mode
	GIMB_TURING_OK,//调头完成,也就是锁头
	GIMB_TURING_AROUND,//正在换头
	
	//chas_ws_dir
	ALINE_TO_F,//对正在前
	ALINE_TO_B,//对正在后
	//chas_ad_dir
	ALINE_TO_L,
	ALINE_TO_R,
	
	//chas_ctrl_mode
	TORQUE_CTRL,
	H_SPEED_CTRL,
	L_SPEED_CTRL,
	ZERO_SPEED_CTRL,
	
	//sideways_state
	SIDEWAYS_ON,
	SIDEWAYS_OFF,
	
	//dafu_switch
	
	//gimb_imu_state
	NORMAL,
	UNNORMAL,
	
}chas_flag_list_e;



//-------------------------------------------结构体类型

typedef struct Chas_Flag
{
	chas_flag_list_e  				is_in_air; //是否离地	
	switch_e                  brake_ctrl_switch;  //刹车控制请求
	chas_flag_list_e       		now_state; //当前状态模式
	chas_flag_list_e				  chas_motion_mode; //底盘当前运动模式	
	chas_flag_list_e	        gimb_motion_mode; //云台当前运动模式
	chas_flag_list_e 					chas_ws_dir; //底盘前后方向
	chas_flag_list_e 					chas_ad_dir; //底盘左右方向
	chas_flag_list_e					chas_ctrl_mode; //底盘控制模式
	switch_e          				gimb_auto_mode; //自瞄是否开启          
	switch_e                  sys_reset_switch; //系统是否复位
	chas_flag_list_e          sideways_state; //是否开启侧身
	switch_e									speed_up_switch; //是否加速
	switch_e                  dafu_switch; //是否打符
	chas_flag_list_e          gimb_imu_state;//云台imu状态
	
}Chas_Flag_t;


typedef struct Master_Comm_Flag
{
	comm_state_flg_e  	state_flag;		//master_comm_heart_beat设置
			
}Master_Comm_Flag_t;

typedef struct RC_Flag
{
	dev_work_state_e  	work_state;		
	switch_e         		ch_wise; 			//Link_RC_CH_Is_Reset_Handler函数里修改
	switch_e  					magz_state;		
	
}RC_Flag_t;
	
typedef struct
{
	Chas_Flag_t  				chassic_flag;
	Master_Comm_Flag_t  comm_flag;
	RC_Flag_t       		rc_flag;

	void    (*ctrl)(void);

}State_t;




void Chassic_Flag_Set(const char flag);
void State_Reset(void);


void Link_RC_CH_Is_Reset_Handler(char flag);
bool Link_RC_Judge_Is_Wise_Enable_Handler(void);
void Link_RC_Offline_Handler(void);//在State_Ctrl被调用
void Link_RC_State_UP_Handler(char state);

extern State_t State;

#endif
