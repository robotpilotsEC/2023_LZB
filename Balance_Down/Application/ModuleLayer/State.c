#include "State.h"
extern void Chassic_Stop(void);

//---------------------------------------上层函数
void State_Ctrl(void);//总执行

void Sys_Judge_Is_Reset(void);
void State_Reset(void);
void Chassic_Flag_Set(const char flag);



void Link_RC_CH_Is_Reset_Handler(char flag);
bool Link_RC_Judge_Is_Wise_Enable_Handler(void);
void Link_RC_Offline_Handler(void);//在State_Ctrl被调用
void Link_RC_State_UP_Handler(char state);


State_t State =
{
	//任务中调用
	.ctrl = State_Ctrl,
	
	.chassic_flag.is_in_air = ON_GROUND,
	.chassic_flag.brake_ctrl_switch = CLOSE,
	.chassic_flag.now_state = FALL,
	.chassic_flag.chas_motion_mode  = SMALL_TOP_OFF,
	.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND,
	.chassic_flag.chas_ws_dir = ALINE_TO_F,
	.chassic_flag.chas_ad_dir = ALINE_TO_L,
	.chassic_flag.chas_ctrl_mode = TORQUE_CTRL,
	.chassic_flag.gimb_auto_mode  = CLOSE,
	.chassic_flag.sys_reset_switch = CLOSE,
	.chassic_flag.sideways_state = SIDEWAYS_OFF,
	.chassic_flag.speed_up_switch = CLOSE,
	.chassic_flag.dafu_switch = CLOSE,
	.chassic_flag.gimb_imu_state = NORMAL,
	
	.rc_flag.ch_wise = OPEN,
	.rc_flag.magz_state = OPEN,
};


uint8_t fail_comm_cnt;
void State_Ctrl(void)
{
	//接了接收机,毫无相干
	#ifdef LINK_TO_RECEIVE
	{
		if( RC_OFFLINE )
			Link_RC_Offline_Handler();
		
		if( FAIL_COMM )
			Comm_Fail_Handler();
	}	
	#endif
	
	//要么通讯成功但是遥控器失联,要么通讯失败遥控器失联
	#ifdef NO_LINK_TO_RECEIVE
	{
		Sys_Judge_Is_Reset();
		
		if(FAIL_COMM)
		{
			fail_comm_cnt++;
			if(fail_comm_cnt >= 100)
			{
				fail_comm_cnt = 0;
				State_Reset();
				State.comm_flag.state_flag = COMM_FAIL;
				State.rc_flag.work_state = DEV_OFFLINE;
			}
		}
		else
		{
			if(RC_OFFLINE)
			{
				State_Reset();
				State.rc_flag.work_state = DEV_OFFLINE;
			}
			fail_comm_cnt = 0;
		}
	}	
	#endif

		
}



/*系统是否复位*/
void Sys_Judge_Is_Reset(void)
{
	if(State.chassic_flag.sys_reset_switch == OPEN)
	{
		Chassic_Stop();
		__set_FAULTMASK(1); 
		NVIC_SystemReset();
	}
}

/*重置所有标志位，但是不设置通讯是否成功和遥控器是否开启*/
void State_Reset(void)
{
	//底盘
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.brake_ctrl_switch = CLOSE;
	State.chassic_flag.now_state = FALL;
	State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;
	State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	State.chassic_flag.chas_ws_dir = ALINE_TO_F;
	State.chassic_flag.chas_ad_dir = ALINE_TO_L;
	State.chassic_flag.chas_ctrl_mode = TORQUE_CTRL;
	State.chassic_flag.gimb_auto_mode = CLOSE;
	State.chassic_flag.sideways_state = SIDEWAYS_OFF;
	State.chassic_flag.speed_up_switch = CLOSE;
	State.chassic_flag.dafu_switch = CLOSE;
	//State.chassic_flag.gimb_imu_state = NORMAL;
	
	//遥控器
	State.rc_flag.ch_wise    = OPEN;
	State.rc_flag.magz_state = OPEN;
	
}




















//-------------------------------------底盘的虚函数重写
void Chassic_Flag_Set(const char flag)
{
	chas_flag_list_e chas_flag = (chas_flag_list_e)flag;
	
	if(flag == CLOSE || flag == OPEN) 
		State.chassic_flag.brake_ctrl_switch = (switch_e)flag;
	
	else if(chas_flag == Fly_Out_Of_Ground || chas_flag == Bounce_Up_Of_Ground || \
					chas_flag == ON_GROUND)
		State.chassic_flag.is_in_air = chas_flag;
	
	else if(chas_flag == SMALL_TOP_OFF || chas_flag == SMALL_TOP_ON )
		State.chassic_flag.chas_motion_mode = chas_flag;	
	
	else if(chas_flag == ALINE || chas_flag == FALL || chas_flag == SPIN || \
					chas_flag == STUCK || chas_flag == SIDEWAYS || chas_flag == DAFU)
		State.chassic_flag.now_state = chas_flag;	
	
	else if(chas_flag == GIMB_TURING_OK || chas_flag == GIMB_TURING_AROUND)
		State.chassic_flag.gimb_motion_mode = chas_flag;
	
	else if(chas_flag == ALINE_TO_F || chas_flag == ALINE_TO_B) 
		State.chassic_flag.chas_ws_dir = chas_flag;
	
	else if(chas_flag == ALINE_TO_L || chas_flag == ALINE_TO_R)
		State.chassic_flag.chas_ad_dir = chas_flag;
	
	else if(chas_flag == TORQUE_CTRL || chas_flag == H_SPEED_CTRL || chas_flag == L_SPEED_CTRL || \
					chas_flag == ZERO_SPEED_CTRL)
		State.chassic_flag.chas_ctrl_mode = chas_flag;
	
	else if(chas_flag == SIDEWAYS_ON || chas_flag == SIDEWAYS_OFF)
		State.chassic_flag.sideways_state = chas_flag;

	
}

























//--------------------------------------遥控器的虚函数重写

//接了接收机模式,第一次上电遥控器通道正常
void Link_RC_CH_Is_Reset_Handler(char flag)
{
	State.rc_flag.ch_wise = (switch_e)flag;
}

//接了接收机模式,返回
bool Link_RC_Judge_Is_Wise_Enable_Handler(void)
{
	return State.rc_flag.ch_wise;
}

//接了遥控器后重置遥控器所有标志位和底盘所有标志位
void Link_RC_Offline_Handler(void)
{
	//底盘
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.brake_ctrl_switch = CLOSE;
	State.chassic_flag.chas_ctrl_mode = TORQUE_CTRL;
	//遥控器
	State.rc_flag.work_state = DEV_OFFLINE;
	
	State.rc_flag.ch_wise    = CLOSE;
	
	State.rc_flag.magz_state = OPEN;
	
}
//接了接收机，更新State的标志位
void Link_RC_State_UP_Handler(char state)
{
	State.rc_flag.work_state = (dev_work_state_e)state;
}
