#include "State.h"
extern void Chassic_Stop(void);

//---------------------------------------�ϲ㺯��
void State_Ctrl(void);//��ִ��

void Sys_Judge_Is_Reset(void);
void State_Reset(void);
void Chassic_Flag_Set(const char flag);



void Link_RC_CH_Is_Reset_Handler(char flag);
bool Link_RC_Judge_Is_Wise_Enable_Handler(void);
void Link_RC_Offline_Handler(void);//��State_Ctrl������
void Link_RC_State_UP_Handler(char state);


State_t State =
{
	//�����е���
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
	//���˽��ջ�,�������
	#ifdef LINK_TO_RECEIVE
	{
		if( RC_OFFLINE )
			Link_RC_Offline_Handler();
		
		if( FAIL_COMM )
			Comm_Fail_Handler();
	}	
	#endif
	
	//ҪôͨѶ�ɹ�����ң����ʧ��,ҪôͨѶʧ��ң����ʧ��
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



/*ϵͳ�Ƿ�λ*/
void Sys_Judge_Is_Reset(void)
{
	if(State.chassic_flag.sys_reset_switch == OPEN)
	{
		Chassic_Stop();
		__set_FAULTMASK(1); 
		NVIC_SystemReset();
	}
}

/*�������б�־λ�����ǲ�����ͨѶ�Ƿ�ɹ���ң�����Ƿ���*/
void State_Reset(void)
{
	//����
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
	
	//ң����
	State.rc_flag.ch_wise    = OPEN;
	State.rc_flag.magz_state = OPEN;
	
}




















//-------------------------------------���̵��麯����д
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

























//--------------------------------------ң�������麯����д

//���˽��ջ�ģʽ,��һ���ϵ�ң����ͨ������
void Link_RC_CH_Is_Reset_Handler(char flag)
{
	State.rc_flag.ch_wise = (switch_e)flag;
}

//���˽��ջ�ģʽ,����
bool Link_RC_Judge_Is_Wise_Enable_Handler(void)
{
	return State.rc_flag.ch_wise;
}

//����ң����������ң�������б�־λ�͵������б�־λ
void Link_RC_Offline_Handler(void)
{
	//����
	State.chassic_flag.is_in_air = ON_GROUND;
	State.chassic_flag.brake_ctrl_switch = CLOSE;
	State.chassic_flag.chas_ctrl_mode = TORQUE_CTRL;
	//ң����
	State.rc_flag.work_state = DEV_OFFLINE;
	
	State.rc_flag.ch_wise    = CLOSE;
	
	State.rc_flag.magz_state = OPEN;
	
}
//���˽��ջ�������State�ı�־λ
void Link_RC_State_UP_Handler(char state)
{
	State.rc_flag.work_state = (dev_work_state_e)state;
}
