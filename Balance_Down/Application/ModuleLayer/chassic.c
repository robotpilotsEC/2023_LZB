#include "chassic.h"
//* Private macro -------------------------------------------------------------*/
//* Private function prototypes -----------------------------------------------*/
//----------上层函数
void Chassic_Ctrl(void);
void LQR_Param_UP(float* C_LQR, float* M_LQR);
void Chassic_Status_UP(void);
void Chassic_Sys_Input_Tx(void);
void Chassic_Stop(void);
void Chassic_In_RC_OFFLINE_Handler(void);
void Chassic_Unnormal_State_Handler(void);


//-----------虚函数
__weak void Chassic_Flag_Set(const char flag);
__weak void Chas_Show_UI_Stuck(const char flag);


//----------基础函数
void Chassic_Status_Meas_UP(Sys_Status_t *status);
void Chassic_Status_Tar_UP(Sys_Status_t *status);
void Chassic_Status_Err_UP(Sys_Status_t *status);
void Chassic_Gesture_TOF_Meas_UP(void);
void Chassic_Judge_Is_In_Air(void);


void Chassic_Sys_Input_Cal(void);
void Motor_Rx_Torque_UP(Sys_Input_t *input);
void Cal_Motor_Sys_Input(Sys_Input_t *input);
void Motor_Err_Torque_UP(Sys_Input_t *input);
void Cal_Motor_Iq_Input(Sys_Input_t *input);
void Tx_KT_Motor_Iq_Input(void);
void Tx_KT_Motor_SpeedControl_Input(void);
void Tx_RM_Motor_Iq_Input(void);
void Cal_Angle_Pid_Out(float L_Angle_Tar, float R_Angle_Tar);


//----------辅助函数
float PosMeas_2_XMeas(uint16_t pos_meas, uint16_t mid_pos_meas, MOM_motor_list_e name);
void Chassic_Sim(float WS, float AD, float tar_v);
void Chassic_Yaw_Motor_Angle_Standard(Sys_Status_t *status, 
																			const uint16_t now_angle,
																			const uint16_t bias_angle);
bool Judge_LQR_Is_Replace(float *former, float* now);//相同为1,不同为0
void Momentum_At_Limited_Pos_Protect(void);													
void Chassic_X_Speed_Tar_Dynamic_Changes(float WS);																			
void Chassic_Improve_Fixed_Point_Effects(void);
void Chassic_Brake_Over_Dynamic_Ctrl(void);
void Check_KT_Motor_State(void);
void Chas_Clear_X_Info(void);
void Momen_Offline_Handler(void);
																			
//* Private variables ---------------------------------------------------------*/
Chassic_t chassic = 
{
	.ctrl = Chassic_Ctrl,
	
};

//依次是位移,速度,动量块位移,动量快速度,角度,角速度,yaw,yaw角度

//负号依次:(车体的)- -  (动量快的)- -   (角度的)+ +  (yaw)- -
float  H_C_LQR_Param[STATUS_VAL_NUM] = {-7, -12, -44, -16, 120, 14, -40, -6};
float  L_C_LQR_Param[STATUS_VAL_NUM] = {-8, -16, -44, -16, 110, 22, -40, -6};
//符号依次:(车体的)+ +  (动量快的)+ +   (角度的)- -  (yaw)0 0
//float  L_M_LQR_Param[STATUS_VAL_NUM] = 
float  L_M_LQR_Param[STATUS_VAL_NUM] = {0.2, 1.2, 1.5, 1.0, -0.8, -0.2, 0, 0};
float  H_M_LQR_Param[STATUS_VAL_NUM] = {0.2, 0.6, 1.2, 1.0, -0.7, -0.2, 0, 0};//速度高的


float C_LQR_Reserve[STATUS_VAL_NUM] = {0};
float M_LQR_Reserve[STATUS_VAL_NUM] = {0};

extern RM_motor_t MOM_motor[MOM_MOTOR_LIST];
extern KT_motor_t KT_motor[KT_MOTOR_LIST];

//4个不可修改的全局指针指向4个电机,但是9025不加const,因为在遥控失联要把电机算的总电机数清除
KT_motor_rx_info_t*  Leg_L_info = &KT_motor[LEG_L].KT_motor_info.rx_info;
KT_motor_rx_info_t*	 Leg_R_info = &KT_motor[LEG_R].KT_motor_info.rx_info;

const RM_motor_rx_info_t*  Mom_L_info = &MOM_motor[MOMENTUM_L].rx_info;
const RM_motor_rx_info_t*	 Mom_R_info = &MOM_motor[MOMENTUM_R].rx_info;

//LQR共16个参数的正负号表格,强制把动量块关于yaw方向的增益变为0
const int LQR_Sign_Buff[2][STATUS_VAL_NUM] = \
{
	//先底盘,后动量块
	{-1, -1, //车体位移速度
	 -1, -1, //动量块位移速度
	  1,  1, //车体倾角
	 -1, -1},//yaw
	
	{ 1,  1, //车体位移速度
	  1,  1, //动量块位移速度
	 -1, -1, //车体倾角
	  0,  0} //yaw
};




void Chassic_Ctrl(void)
{
	
	if(State.chassic_flag.speed_up_switch == OPEN)
		LQR_Param_UP(H_C_LQR_Param, H_M_LQR_Param);
	else 
		LQR_Param_UP(L_C_LQR_Param, L_M_LQR_Param);
	
	
	if(RC_ONLINE)
	{
	
		Chassic_Status_UP();
		
		//Chassic_Unnormal_State_Handler();	
		
		Chassic_Sys_Input_Tx();
	}
	
	else
	{
		Chassic_Stop();
		
		Chassic_In_RC_OFFLINE_Handler();
	}
}



//-----------------------------------------系统更新


 


//只需修改上面两个LQR数组就可保存到结构体的两个数组中
//内置符号逻辑设置,防止调参手误疯车
void LQR_Param_UP(float* C_LQR, float* M_LQR)
{
	if(C_LQR == NULL || M_LQR == NULL)
		return;
	
	uint8_t i = 0, j = 0;
	//先判断底盘符号
	for(i = 0; i < 2; i ++)
	{
		for(j = 0; j < STATUS_VAL_NUM; j ++)
		{
			//对于底盘,判断C_LQR数组
			if( i == 0 && LQR_Sign_Buff[i][j] != one(C_LQR[j]) )
			{
				if(LQR_Sign_Buff[i][j] == 0)	
					C_LQR[j] = 0;
					
				else
					C_LQR[j] *= -1;					
			}
			//对于动量块,判断M_LQR数组
			else if( i == 1 && LQR_Sign_Buff[i][j] != one(M_LQR[j]) )
			{
				if(LQR_Sign_Buff[i][j] == 0)	
					M_LQR[j] = 0;
					
				else
					M_LQR[j] *= -1;
			}		
		}
		//一轮结束修改j值
		j = 0;
	}
	i = 0;
	
	memcpy( chassic.info.C_LQR_Gain, C_LQR, STATUS_VAL_NUM * sizeof(C_LQR[0]) );
	memcpy( chassic.info.M_LQR_Gain, M_LQR, STATUS_VAL_NUM * sizeof(M_LQR[0]) );
	

}

void Chassic_Status_UP(void)
{
	
	Chassic_Status_Meas_UP(&chassic.info.Status);
	Chassic_Status_Tar_UP(&chassic.info.Status);
	Chassic_Status_Err_UP(&chassic.info.Status);
	
	Chassic_Gesture_TOF_Meas_UP();
	//Chassic_Improve_Fixed_Point_Effects();
}

//右边系统的轮子电机位移、速度加一个负号、动量块速度加一个负号跟左系统对应好

void Chassic_Status_Meas_UP(Sys_Status_t *status)
{
	if(status == NULL)
		return;

	//用总弧度 / 2pai * (2pai*R)
	status->meas.C_X_L = Leg_L_info->radian_sum * WHEEL_RADIUS;
	status->meas.C_X_R = -Leg_R_info->radian_sum * WHEEL_RADIUS;
	
	//用rpm--->m/s，首先÷60变成1s有N转，再乘以(2*pai*R)
	status->meas.C_V_L = ((float)Leg_L_info->speed) / 360.f * WHEEL_GIRTH;
	status->meas.C_V_R = -((float)Leg_R_info->speed) / 360.f * WHEEL_GIRTH;

	
	//各自的角度位置
	status->meas.L_M_Pos = Mom_L_info->angle_offset;
	status->meas.R_M_Pos = Mom_R_info->angle_offset;	
	
	//各自的位移
	status->meas.M_X_L = PosMeas_2_XMeas(status->meas.L_M_Pos,
																		   MOMENTUM_L_M_POS_OFFSET,
																		   MOMENTUM_L);
	status->meas.M_X_R = PosMeas_2_XMeas(status->meas.R_M_Pos,
																		   MOMENTUM_R_M_POS_OFFSET,
																		   MOMENTUM_R);
	
	//各自的速度
	status->meas.M_V_L = ((float)(Mom_L_info->speed)) / 60.f * MOMENTUM_GIRTH;
	status->meas.M_V_R = -((float)(Mom_R_info->speed)) / 60.f * MOMENTUM_GIRTH;		
	
	//Yaw电机角度
	Chassic_Yaw_Motor_Angle_Standard(status, 
																	 Master.rx_pack.std_frame_rx_data_t.motor_angle,
																	 YAW_MOTOR_F_ANGLE);
	
	
	//轮子平均数据
	status->meas.C_X = (status->meas.C_X_L + status->meas.C_X_R) / 2.f; 
	status->meas.C_V = (status->meas.C_V_L + status->meas.C_V_R) / 2.f; 
	
	//动量块平均数据
	status->meas.M_X = (status->meas.M_X_L + status->meas.M_X_R) / 2.f;
	status->meas.M_V = (status->meas.M_V_L + status->meas.M_V_R) / 2.f;
	
	//转弧度值
	status->meas.Pitch = imu_sensor.info->base_info.pitch * ANGLE_CONVERSION_RADIAN;
	status->meas.Rate_Pitch = imu_sensor.info->base_info.ave_rate_pitch * ANGLE_CONVERSION_RADIAN;

	
	//如果接了接收器,采用自身yaw
	#ifdef LINK_TO_RECEIVE
	{
		status->meas.Yaw = imu_sensor.info->base_info.yaw * ANGLE_CONVERSION_RADIAN;		
	}
	#endif

		
	//用上主控发的电机角度作为yaw参考值
	#ifdef NO_LINK_TO_RECEIVE
	{
		//要么是起立过程
		//要么是对齐中调头过程
		//要么开启打符模式，不过要先侧身好
		//要么是侧身调头
		if( State.chassic_flag.now_state == FALL || \
		   ( State.chassic_flag.now_state == ALINE && \
				 State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND ) || \
				State.chassic_flag.now_state == DAFU || \
			 ( State.chassic_flag.now_state == SIDEWAYS && \
				 State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND ) )
		{
			status->meas.Yaw = imu_sensor.info->base_info.yaw * ANGLE_CONVERSION_RADIAN;	
		}

		//只处于对齐
		//自旋中调头或不调头
		//自旋关闭过程
		else
		{
			//得到0~-360范围
			status->meas.Yaw = -status->meas.Y_Standard_Pos / RM_ANGEL_TO_IMU_ANGLE * ANGLE_CONVERSION_RADIAN;
			//规定化到-180~180
			if(status->meas.Yaw <= -VALUE_PI)
				status->meas.Yaw += VALUE_PI * 2.f;
			
	  }
			
	}
	#endif
	
	status->meas.Rate_Yaw = imu_sensor.info->base_info.ave_rate_yaw * ANGLE_CONVERSION_RADIAN;
	
}


//目标值更新中
//如果接了接收机,进行了速度目标不为0清除位移、失联保持yaw的目标值是测量值
float basi;
void Chassic_Status_Tar_UP(Sys_Status_t *status)
{
	if(status == NULL)
		return;

	float tar_v;
	uint8_t  devices_online = 0;
	
	#ifdef LINK_TO_RECEIVE
	{
		Chassic_Sim(RC_LEFT_CH_UD_VALUE, RC_RIGH_CH_LR_VALUE);
	}
	#endif	
	
	#ifdef NO_LINK_TO_RECEIVE
	{	
		//裁判是否在线
		if(JUDGE_ONLINE && Super_2023.work_state == DEV_ONLINE)
		{
			devices_online = 1;
			
			if(State.chassic_flag.speed_up_switch == OPEN)
				tar_v = H_X_MAX_START_SPEED;
			else 
				tar_v = L_X_MAX_START_SPEED;
		}
		else
		{
			devices_online = 0;
			tar_v = Judge_Offline_X_Speed;
			chassic.info.C_LQR_Gain[Y] = L_C_LQR_Param[Y]*3.f/4.f;
			chassic.info.M_LQR_Gain[C_V] = L_M_LQR_Param[C_V]*1.5f;
		}

		
//		//前期辅助加速，只会在加速的时候才启动
//		if( RC_LEFT_CH_UD_VALUE != 0 && \
//				tar_v == L_X_MAX_START_SPEED && \
//				State.chassic_flag.sideways_state == SIDEWAYS_OFF 
//			)
//		{
//			//如果此时是正对着头且想往前走
//			if( State.chassic_flag.chas_dir == ALINE_TO_F && \
//				  chassic.info.Status.meas.C_V > 0 && \
//					chassic.info.Status.meas.C_V < 0.5f && \
//					RC_LEFT_CH_UD_VALUE > 500 )
//			{
//				tar_v = H_X_MAX_START_SPEED;
//			}
//				
//				
//			//如果此时是背对着头且想往前走
//			if( State.chassic_flag.chas_dir == ALINE_TO_B && \
//				  chassic.info.Status.meas.C_V < 0 && \
//					chassic.info.Status.meas.C_V > -0.5f && \
//					RC_LEFT_CH_UD_VALUE < -500 )			
//			{
//				tar_v = H_X_MAX_START_SPEED;
//			}
//			
//		}
//		//后期辅助加速	
//		if( RC_LEFT_CH_UD_VALUE != 0 && \
//				abs_(status->err.C_V) <= 0.3f && \
//				tar_v == L_X_MAX_START_SPEED && \
//				State.chassic_flag.sideways_state == SIDEWAYS_OFF && \
//				abs_(chassic.info.Status.err.Pitch) <= 5.f*ANGLE_CONVERSION_RADIAN )
//		{
//			tar_v = M_X_MAX_START_SPEED;
//		}o
//		
		/*-------------------------------卡墙---------------------------------------------*/
//		if(judge_info.power_heat_data.chassis_power_buffer<20.f)			 
//		{

//			Chas_Clear_X_Info();
//			Chassic_Flag_Set(STUCK);	
//			
//			if(Super_2023.work_state == DEV_ONLINE)
//			{
//				Chassic_Flag_Set(H_SPEED_CTRL);	
//				
//			}
//			else
//			{
//				Chassic_Flag_Set(L_SPEED_CTRL);	
//			}

//		}	
		
		/*--------------------------------状态机开始工作---------------------------------*/

		
		//底盘倒下
		if(State.chassic_flag.now_state == FALL)
		{
			//初始复位底盘用自己的yaw,等待云台复位成功再跟随
			if(State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND)
			{
				status->tar.Yaw = status->meas.Yaw;
			}
			//云台复位成功
			else
			{		
				Chassic_Flag_Set(ALINE);
			}	
		}
		
		//底盘卡墙
		else if( State.chassic_flag.now_state == STUCK )
		{
			if(abs_(judge_info.power_heat_data.chassis_power_buffer)>40.f)
			{
				Chassic_Flag_Set(TORQUE_CTRL);
				Chassic_Flag_Set(ALINE);
			
				//在前面,目标角度为0
				if(State.chassic_flag.chas_ws_dir == ALINE_TO_F)
				{
					status->tar.Yaw = 0.f;
				}
				//在后面,目标角度为180°,即Π弧度
				else if(State.chassic_flag.chas_ws_dir == ALINE_TO_B)
				{
					status->tar.Yaw = VALUE_PI;
				}
			}

				
		}

		//底盘跟随
		else if(State.chassic_flag.now_state == ALINE)
  	{
			
			/*-------------开启小陀螺,切换底盘状态为SPIN,满足倾角小-------------*/
			if(State.chassic_flag.chas_motion_mode == SMALL_TOP_ON)
			{			
				if(abs_(imu_sensor.info->base_info.pitch) <= 5.f)	chassic.users.low_pitch_cnt++;	
				else  chassic.users.low_pitch_cnt = 0;		
				
				if(chassic.users.low_pitch_cnt >= 10 )
				{
					chassic.users.low_pitch_cnt = 0;
					chassic.users.low_speed_cnt = 0;
					Chassic_Flag_Set(SPIN);
					Chassic_Flag_Set(SIDEWAYS_OFF);
				}
				else
				{
					chassic.info.C_LQR_Gain[P] = L_C_LQR_Param[P] * 3;
				}

			}
			else
			{
				chassic.users.low_pitch_cnt = 0;
				chassic.users.low_speed_cnt = 0;
			}
			
			/*-------------开启侧身,底盘的倾角应该要小才能进入------------------*/
			if(State.chassic_flag.sideways_state == SIDEWAYS_ON)
			{
				if( abs_(status->err.Pitch) <= 5.f && abs_(status->meas.C_V) <= 0.6f )
				{
					Chassic_Flag_Set(SIDEWAYS);
					chassic.users.sideways_request = 1;//请求侧身
				}
				else
				{
					
					/*动量块尽快帮助刹车*/
					if(chassic.users.momentum_offline == 0)
					{
						status->tar.M_X_L = status->tar.M_X_R = -one(chassic.info.Status.meas.C_V)*MOMENTUM_MAX_X_MEAS;
					
						chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = \
						chassic.info.M_LQR_Gain[P] = 0;
						chassic.info.M_LQR_Gain[M_X] = 3*L_M_LQR_Param[M_X];
					}			
				}
				
				
				
			}
			else
			{
				chassic.users.sideways_request = 0;//没有请求侧身
				chassic.users.sideways_over = 0;//没有侧身好
				chassic.users.sideways_dir = 0; //侧身方向为0
			}
			
			/*-----------------------------转弯保护--------------------------------*/
			if( RC_LEFT_CH_UD_VALUE != 0 && State.chassic_flag.sideways_state == SIDEWAYS_OFF )
			{
				if( (abs_(status->err.Yaw) >= 6.f*ANGLE_CONVERSION_RADIAN) && (abs_(status->meas.C_V) >= tar_v*3.f/4.f) )
				{
					if(chassic.users.momentum_offline == 0)
					{
						status->tar.M_X_L = status->tar.M_X_R = -one(chassic.info.Status.meas.C_V)*MOMENTUM_MAX_X_MEAS;

						chassic.info.C_LQR_Gain[M_X] = 0;
						chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = \
						chassic.info.M_LQR_Gain[P] = 0;
						chassic.info.M_LQR_Gain[M_X] = 2*L_M_LQR_Param[M_X];
					}		
					if( abs_(status->err.Pitch) >= 6.f*ANGLE_CONVERSION_RADIAN )
						chassic.info.C_LQR_Gain[P] = L_C_LQR_Param[P] * 3;
					else if( abs_(status->err.Pitch) >= 3.f*ANGLE_CONVERSION_RADIAN )
						chassic.info.C_LQR_Gain[P] = L_C_LQR_Param[P] * 4;
				}
				
				else
					status->tar.M_X_L = status->tar.M_X_R = 0;
			}


			
			/*----------------------------如果调头了-----------------------------------*/
			if(State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND)
			{
				status->tar.Yaw = status->meas.Yaw;
			}
			
			/*----------------------------如果并没有调头------------------------------*/
			else
			{			
				//弹仓打开，速度较小
				if(State.rc_flag.magz_state == OPEN)
				{		
					if(State.chassic_flag.chas_ws_dir == ALINE_TO_F)
					{
						status->tar.Yaw = 0.f;	
						Chassic_Sim(RC_LEFT_CH_UD_VALUE / 3.f, 0, tar_v);				
					}
					
					else if(State.chassic_flag.chas_ws_dir == ALINE_TO_B)
					{
						status->tar.Yaw = VALUE_PI;
						Chassic_Sim( -RC_LEFT_CH_UD_VALUE / 3.f, 0, tar_v);
					}							
				}				
				//弹仓关闭，速度较大
				else
				{
					if(State.chassic_flag.chas_ws_dir == ALINE_TO_F)
					{
						status->tar.Yaw = 0.f;
						Chassic_Sim(RC_LEFT_CH_UD_VALUE, 0, tar_v);					
					}						
					else if(State.chassic_flag.chas_ws_dir == ALINE_TO_B)
					{
						status->tar.Yaw = VALUE_PI;
						Chassic_Sim( -RC_LEFT_CH_UD_VALUE, 0, tar_v);					
					}						
				}
			}
				
		}
		
		//底盘侧身
		else if(State.chassic_flag.now_state == SIDEWAYS)
		{
		
			/*给定斜坡输入，保证是以操作手的视角逆时针转*/
			if(State.chassic_flag.chas_ws_dir == ALINE_TO_F && chassic.users.sideways_dir == 0)
			{
				chassic.users.sideways_dir = ALINE_TO_F;			
			}		
			else if(State.chassic_flag.chas_ws_dir == ALINE_TO_B  && chassic.users.sideways_dir == 0)
			{
				chassic.users.sideways_dir = ALINE_TO_B;		
			}
			
			if(chassic.users.sideways_dir == ALINE_TO_F)
			{
				if(devices_online)
					status->tar.Yaw += 0.008f;
				else
					status->tar.Yaw += 0.005f;
				
				if(status->tar.Yaw > VALUE_PI/2.f)
				{
					status->tar.Yaw = VALUE_PI/2.f;
				}
			}
			else if(chassic.users.sideways_dir == ALINE_TO_B)
			{							
				if(status->tar.Yaw == VALUE_PI)
				{	
					status->tar.Yaw *= -1;
				}
				if(devices_online)
					status->tar.Yaw += 0.008f;
				else
					status->tar.Yaw += 0.005f;
				
				if(status->tar.Yaw > -VALUE_PI/2.f)
				{
					status->tar.Yaw = -VALUE_PI/2.f;
				}
			}
			
			/*------------------如果开了侧身了并且是打符模式了------------------------*/
			if( State.chassic_flag.dafu_switch == OPEN && chassic.users.sideways_over == 1 )
			{		
				Chassic_Flag_Set(DAFU);
			}

			/*完成了侧身*/
			if( (State.chassic_flag.chas_ad_dir == ALINE_TO_L && \
				   status->meas.Y_Standard_Pos >=1900 && \
				   status->meas.Y_Standard_Pos <= 2150)	|| 
					(State.chassic_flag.chas_ad_dir == ALINE_TO_R && \
				   status->meas.Y_Standard_Pos >=6000 && \
				   status->meas.Y_Standard_Pos <= 6250)	)
			{
				status->tar.M_X_L = status->tar.M_X_R = 0;
				chassic.users.sideways_over = 1;
			}
			else/*如果没完全侧身好，就清空位移计算*/
			{
				Chas_Clear_X_Info();
								
			}				
	

//			/*降低左右移动时产生的倾角*/
//			if(chassic.users.sideways_over == 1)
//			{
//				chassic.info.M_LQR_Gain[C_V] = 1.5f*L_M_LQR_Param[C_V];
//				chassic.info.C_LQR_Gain[P] = L_C_LQR_Param[P]*1.5f;
//			}
//			
			
			/*如果侧身中调头*/
			if(State.chassic_flag.gimb_motion_mode == GIMB_TURING_AROUND)
			{
				status->tar.Yaw = status->meas.Yaw;				  
			}
			else
			{
					
				if(devices_online)
				{
					if(State.chassic_flag.chas_ad_dir == ALINE_TO_R)
					{
						status->tar.Yaw = VALUE_PI/2.f;
						/*降低左右移动速度*/
						if(devices_online)
							Chassic_Sim(-RC_LEFT_CH_LR_VALUE/1.5f, 0, tar_v);
					}
						
					else if(State.chassic_flag.chas_ad_dir == ALINE_TO_L)
					{
						status->tar.Yaw = -VALUE_PI/2.f;	
						/*降低左右移动速度*/
						if(devices_online)
							Chassic_Sim(RC_LEFT_CH_LR_VALUE/1.5f, 0, tar_v);						
					}
				}
		
			}
			
			/*如果要关闭侧身前进了，需要等侧身完90°再前进*/	
			if(State.chassic_flag.sideways_state == SIDEWAYS_OFF)
			{
				if( abs_(chassic.info.Status.err.Yaw) <= 5.f )
				{
					Chassic_Flag_Set(ALINE);
				}
				else
				{
					Chas_Clear_X_Info();
				}
			}
		}
  	//底盘旋转
		else if(State.chassic_flag.now_state == SPIN)
		{
			//开启自旋,不允许前进后退,动量块在一前一后
			//允许云台自主移动
			//允许调头
			if(State.chassic_flag.chas_motion_mode == SMALL_TOP_ON)
			{
				//动量快的位置目标值改变
			 if(chassic.users.momentum_offline == 0)
			 {
				status->tar.M_X_L = -MOMENTUM_MAX_X_MEAS;				
				status->tar.M_X_R = MOMENTUM_MAX_X_MEAS;
		
				//提高动量块响应
				chassic.info.M_LQR_Gain[M_X] = 5 * L_M_LQR_Param[M_X];
			 }
				
				
				//关闭不必要的参数
				chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = 0;
				chassic.info.C_LQR_Gain[M_X] = chassic.info.C_LQR_Gain[M_V] = 0;
			 
				//清除位移测量
				Chas_Clear_X_Info();
	
				//模拟小陀螺前的斜坡输入
				if(chassic.users.start_spin_flag == 0)
				{			
					if(devices_online)
					{
						if(chassic.users.spin_val < VALUE_PI/2.f)
						{
							chassic.info.C_LQR_Gain[P] = L_C_LQR_Param[P] * 2;
							chassic.users.spin_val += 0.005f;
							status->tar.Yaw = status->meas.Yaw + chassic.users.spin_val;
							
							if(chassic.users.spin_handler_flag == 0)
							{
								if(status->meas.Pitch > 0)
									chassic.users.spin_handler_flag = 1;
								else
									chassic.users.spin_handler_flag = 2;
							}					
							
						}				
						else
						{
							chassic.users.start_spin_flag = 1;
							chassic.users.spin_val = 0; //斜坡输入的值
						}
					}
	
				}

				else
				{
					
					if(devices_online)
						status->tar.Yaw = status->meas.Yaw + VALUE_PI / 2.0f;

					
					//根据当前pitch测量给定一个pitch目标
					
					if(abs_(status->meas.Pitch) <= 8.f*ANGLE_CONVERSION_RADIAN)
					{
						chassic.users.spin_err = 0;
						if(chassic.users.spin_handler_flag == 1) 
						{
							status->tar.Pitch = -4.5f*ANGLE_CONVERSION_RADIAN;
						}						
						else if(chassic.users.spin_handler_flag == 2)
						{
							status->tar.Pitch = -4.5f*ANGLE_CONVERSION_RADIAN;
						}
					}
					else
					{
						chassic.users.spin_err++;
						if(chassic.users.spin_err >= 100)
						{
							Chassic_Flag_Set(ALINE);
							chassic.users.spin_err = 0;
						}
							
					}
				}
	
				status->tar.Yaw = Float_ZeroManage( status->tar.Yaw, VALUE_PI );		
				

				
			}
		
			//关闭小陀螺,根据头的位置就近回位,误差小到一定程度然后设置底盘状态为对齐
			else if(State.chassic_flag.chas_motion_mode == SMALL_TOP_OFF)
			{					
				//更新系统目标值
				status->tar.M_X_L = status->tar.M_X_R = 0;
				status->tar.Pitch = 0;
				
				chassic.users.start_spin_flag = 0;
				chassic.users.spin_handler_flag = 0;
				
				//等转到快对齐才改变目标值
				if( abs_(chassic.info.Status.meas.Yaw) <= 3.f*ANGLE_CONVERSION_RADIAN )
				{
					status->tar.Yaw = 0.f;
				}
				else if( abs_(chassic.info.Status.meas.Yaw) >= (VALUE_PI-3.f*ANGLE_CONVERSION_RADIAN) )
				{
					status->tar.Yaw = VALUE_PI;
				}
				
				
				//等姿态较为完美才设置为回正
				if( abs_(status->err.Yaw) <= 2.f*ANGLE_CONVERSION_RADIAN && \
						abs_(status->err.Pitch) <= 2.f*ANGLE_CONVERSION_RADIAN ) 
				{
					chassic.users.spin_close_cnt++;
					
					if(chassic.users.spin_close_cnt >= 10)
					{
						chassic.users.spin_close_cnt = 0;
						Chassic_Flag_Set(ALINE);
					}
						
				}				
				else
				{
					chassic.users.spin_close_cnt = 0;
					Chas_Clear_X_Info();
					
					//优先平衡再转向
					if( abs_(status->err.Pitch) > 2.f*ANGLE_CONVERSION_RADIAN )
					{
						chassic.info.C_LQR_Gain[P] = L_C_LQR_Param[P] * 4;
						chassic.info.C_LQR_Gain[Y] = L_C_LQR_Param[Y] / 4;
					}
					else
					{
						chassic.info.C_LQR_Gain[P] = L_C_LQR_Param[P];
						chassic.info.C_LQR_Gain[Y] = L_C_LQR_Param[Y]*3/2;
					}
					
				}

			}		
		}
		//底盘打符
		else if(State.chassic_flag.now_state == DAFU)
		{
			status->tar.Yaw = status->meas.Yaw;
			
			//打符模式关闭后
			if(State.chassic_flag.dafu_switch == CLOSE)
			{
				State.chassic_flag.now_state = ALINE;
			}
		}
		

	}
	
	#endif
	
	
	if( status->tar.C_V != 0 )	 
	{
		Chas_Clear_X_Info();
	}
		
	
	
}

//采用指针+for循环算误差
void Chassic_Status_Err_UP(Sys_Status_t *status)
{
	if(status == NULL)
		return;
	
	float*  meas_p = &status->meas.C_X;
	float*  tar_p  = &status->tar.C_X;
	float*	err_p  = &status->err.C_X;
	//浮点型数据的个数
	uint8_t i_max = ( sizeof(status_t) - sizeof(status->meas.L_M_Pos) * STRUCT_STATUS_UNNOT_UP_VAL_NUM ) \
									/ sizeof(status->meas.C_X);
	
	//这里对除了动量块的机械角度以外的状态变量误差计算
	for(int i = 0; i < i_max; i ++)
	{
		(*err_p) = (*meas_p) - (*tar_p);
		
		//如果对yaw的误差进行计算，要进行过零点
		if(i == Y)
			*err_p = Float_ZeroManage( *err_p, VALUE_PI );
		
		if(i != i_max - 1)
			{	err_p ++, meas_p ++, tar_p ++; }
		 
	}
	
	//以防后续还用到,先变为最开始的值
	meas_p = &status->meas.C_X;
	tar_p  = &status->tar.C_X;
	err_p  = &status->err.C_X;


}





//----------------------------------------系统输入计算








//发送给电机计算好的系统输入

void Chassic_Sys_Input_Tx(void)
{
	Chassic_Brake_Over_Dynamic_Ctrl();
	
	Chassic_Sys_Input_Cal();

	if(State.chassic_flag.chas_ctrl_mode == TORQUE_CTRL)
		Tx_KT_Motor_Iq_Input();
	
	else
		Tx_KT_Motor_SpeedControl_Input();
	
	Tx_RM_Motor_Iq_Input();
	
}

void Chassic_Sys_Input_Cal(void)
{
	//各个电机的系统输入,扭矩

	Motor_Rx_Torque_UP(&chassic.info.C_L_M);
	Cal_Motor_Sys_Input(&chassic.info.C_L_M);
	Motor_Err_Torque_UP(&chassic.info.C_L_M);
	
	Motor_Rx_Torque_UP(&chassic.info.C_R_M);
	Cal_Motor_Sys_Input(&chassic.info.C_R_M);
	Motor_Err_Torque_UP(&chassic.info.C_R_M);

	//各个电机的iq控制量
	Cal_Motor_Iq_Input(&chassic.info.C_L_M);
	Cal_Motor_Iq_Input(&chassic.info.C_R_M);
	
	

	Cal_Motor_Sys_Input(&chassic.info.M_L_M);
	Cal_Motor_Sys_Input(&chassic.info.M_R_M);
	Cal_Motor_Iq_Input(&chassic.info.M_L_M);
	Cal_Motor_Iq_Input(&chassic.info.M_R_M);
		
	//Momentum_At_Limited_Pos_Protect();

	
}



void Motor_Rx_Torque_UP(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	//加一个负号
	if(input == &chassic.info.C_L_M)
		input->Rx_Torque = -Leg_L_info->torque;
	
	else if(input == &chassic.info.C_R_M)
		input->Rx_Torque = Leg_R_info->torque;
	
}
//计算单个电机的系统输入,也就是扭矩，其中左边系统轮子扭矩加负号
void Cal_Motor_Sys_Input(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	//某一侧系统的状态变量误差，指向这个系统误差的第一个元素
	float* status_err_p = &chassic.info.Status.err.C_X;
	float* LQR_p = NULL;
	float  buffer = CHASSIC_MAX_POWER_BUFFER;
	
	//由于左右轮涉及到旋转,所以定义两个变量,即左+右,左-右;动量块不涉及
	float total_spd_t = 0, total_bal_t = 0, total_gravity_t = 0, total_yaw_t = 0;

	if(input == &chassic.info.M_L_M || input == &chassic.info.M_R_M)
		LQR_p = chassic.info.M_LQR_Gain;
	else
		LQR_p = chassic.info.C_LQR_Gain;
	if(LQR_p == NULL)
		return;
	
	
	
	
	//计算2个电机的分量扭矩和
	for(int i = 0; i < STATUS_VAL_NUM; i ++)
	{
		//速度扭矩
		if(i >= C_X && i <= C_V)
			total_spd_t += *(status_err_p + i) * LQR_p[i];
		//重心扭矩	
		else if(i >= M_X && i <= M_V)
		{
			if(input == &chassic.info.M_L_M)
				input->Gravity_Torque = chassic.info.Status.err.M_X_L * LQR_p[M_X] + \
																chassic.info.Status.err.M_V_L * LQR_p[M_V];	
			
			else if(input == &chassic.info.M_R_M)
				input->Gravity_Torque = chassic.info.Status.err.M_X_R * LQR_p[M_X] + \
															  chassic.info.Status.err.M_V_R * LQR_p[M_V];	
			
			else
				total_gravity_t += *(status_err_p + i) * LQR_p[i];
		}
			
		//平衡扭矩	
		else if(i >= P && i <= P1)
			total_bal_t += *(status_err_p + i) * LQR_p[i];
		//转向扭矩
		else if(i >= Y && i <= Y1)
			total_yaw_t += *(status_err_p + i) * LQR_p[i];
	}

	
	//左腿电机或右腿电机,用动量块的平均测量
	if(input == &chassic.info.C_L_M || input == &chassic.info.C_R_M)
	{
		//单电机扭矩分量
		input->Spd_Torque = constrain(total_spd_t / 2, -MAX_SPEED_TORQUE, MAX_SPEED_TORQUE);
		Chassis_Motor_Power_Limit(&input->Spd_Torque, buffer);

		input->Gravity_Torque = total_gravity_t / 2;
		
		input->Balance_Torque = total_bal_t / 2;
		
		if(input == &chassic.info.C_L_M)
			input->Spin_Torque = total_yaw_t / 2;
		else
			input->Spin_Torque = -total_yaw_t / 2;
		Chassis_Motor_Power_Limit(&input->Spin_Torque, buffer);	
		
		//单电机总扭矩
		input->Tx_Torque = input->Spd_Torque + input->Gravity_Torque + \
											 input->Balance_Torque + input->Spin_Torque;
		
		
		//落地一瞬间给的扭矩进行限幅
		if( chassic.users.torque_compensation_request == 1 )
		{	
			input->Tx_Torque = input->Balance_Torque;	
		}
		
		//扭矩限幅
		input->Tx_Torque = constrain(input->Tx_Torque, -KT_9025_MAX_TORQUE, KT_9025_MAX_TORQUE );
		
	}

	
	//左动量块或右动量块,用各自的位移速度测量
	else if(input == &chassic.info.M_L_M || input == &chassic.info.M_R_M)
	{
		//单电机扭矩分量
		input->Spd_Torque = total_spd_t;
		input->Balance_Torque = total_bal_t;
		input->Spin_Torque = 0;
		
		//单电机总扭矩
		input->Tx_Torque = input->Spd_Torque + input->Gravity_Torque + \
											 input->Balance_Torque + input->Spin_Torque;
		
		
		//扭矩限幅
		input->Tx_Torque = constrain(input->Tx_Torque, -RM_6020_MAX_TORQUE, RM_6020_MAX_TORQUE );	
	}
	
	
	
}

void Motor_Err_Torque_UP(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	if(input == &chassic.info.C_L_M || input == &chassic.info.C_R_M)
		input->Err_Torque = input->Rx_Torque - input->Tx_Torque;
	
}

//将系统输入变为电机的控制输入量
void Cal_Motor_Iq_Input(Sys_Input_t *input)
{
	if(input == NULL)
		return;
	
	//左腿电机或右腿电机---->iq是电流,所以扭矩--->电流
	if(input == &chassic.info.C_L_M || input == &chassic.info.C_R_M)
	{
		input->Current = input->Tx_Torque * KT_TORQUE_TO_CURRENT;
		
		//限幅iq控制量
		input->Current = constrain(input->Current, -KT_9025_MAX_CURRENT , KT_9025_MAX_CURRENT);
	
		//写到KT电机的发送结构体中
		if(input == &chassic.info.C_L_M)
			KT_motor[LEG_L].W_iqControl(&KT_motor[LEG_L], -(int16_t)input->Current);
		
		else
			KT_motor[LEG_R].W_iqControl(&KT_motor[LEG_R], (int16_t)input->Current);
	}
	
	//左动量块或右动量块---->iq是电压,所以扭矩--->电压
	else if(input == &chassic.info.M_L_M || input == &chassic.info.M_R_M)
	{
		input->Voltage = input->Tx_Torque * RM_6020_TORQUE_TO_VOLTAGE;
		
		//限幅iq控制量
		input->Voltage = constrain(input->Voltage, -RM_6020_MAX_VOLTAGE , RM_6020_MAX_VOLTAGE);
		
		//左边电压×负号
		if(input == &chassic.info.M_L_M)
			input->Voltage *= -1.f;
	}
}

//动量块特殊情况下会计算它的PID输出
void Cal_Angle_Pid_Out(float L_Angle_Tar, float R_Angle_Tar)
{
	memset(&chassic.info.M_L_M, 0 ,sizeof(Sys_Input_t));
	chassic.info.M_L_M.Voltage = 	MOM_motor[MOMENTUM_L].ctr_pid2( &MOM_motor[MOMENTUM_L].pid.angle, \
																																&MOM_motor[MOMENTUM_L].pid.angle_in, \
																																MOM_motor[MOMENTUM_L].rx_info.angle_offset, \
																																MOM_motor[MOMENTUM_L].rx_info.speed, \
																																L_Angle_Tar, \
																																0);
	
	memset(&chassic.info.M_R_M, 0 ,sizeof(Sys_Input_t));
	chassic.info.M_R_M.Voltage = MOM_motor[MOMENTUM_R].ctr_pid2( &MOM_motor[MOMENTUM_R].pid.angle, \
																															 &MOM_motor[MOMENTUM_R].pid.angle_in, \
																																MOM_motor[MOMENTUM_R].rx_info.angle_offset, \
																																MOM_motor[MOMENTUM_R].rx_info.speed, \
																																R_Angle_Tar, \
																																0);
	
}

//KT电机发送自己的iq控制量
void Tx_KT_Motor_Iq_Input(void)
{
	KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], 0);
	KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], 0);
	
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID );
  KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID );
}
//KT电机发送自己的速度目标值
void Tx_KT_Motor_SpeedControl_Input(void)
{				
	float speed_ctrl; 
	Chas_Clear_X_Info();
	
	chassic.info.Status.tar.C_V = 0;
	RC_LEFT_CH_UD_VALUE = 0;
	
	memset(&chassic.info.C_L_M, 0, sizeof(Sys_Input_t));
	KT_motor[LEG_L].W_iqControl(&KT_motor[LEG_L], 0);
	memset(&chassic.info.C_R_M, 0, sizeof(Sys_Input_t));
	KT_motor[LEG_R].W_iqControl(&KT_motor[LEG_R], 0);
	
	
	if(State.chassic_flag.chas_ctrl_mode == H_SPEED_CTRL)
		speed_ctrl = KT_SPEED_TO_SPEED_CONTROL(H_SPEED_CONTROL);
	else if(State.chassic_flag.chas_ctrl_mode == L_SPEED_CTRL)
		speed_ctrl = KT_SPEED_TO_SPEED_CONTROL(L_SPEED_CONTROL);


//反转
	if(State.chassic_flag.chas_ws_dir == ALINE_TO_F)
	{
		KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], speed_ctrl );
		KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], -speed_ctrl);
	}
	else
	{
		KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], -speed_ctrl );
		KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], speed_ctrl);
	}


	
	/*卡墙时,速度控制的方向要看此时的倾角倾向哪边*/	
	if(State.chassic_flag.now_state == STUCK) 
	{
		//车后卡墙
		if( one(imu_sensor.info->base_info.pitch) == 1 )
		{
			KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], speed_ctrl*8.f );
			KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], -speed_ctrl*8.f );
		}
		
		//车前卡墙
		else
		{
			KT_motor[LEG_L].W_speedControl(&KT_motor[LEG_L], -speed_ctrl*8.f );
			KT_motor[LEG_R].W_speedControl(&KT_motor[LEG_R], speed_ctrl*8.f );
		}
		
	}

	
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], SPEED_CLOSE_LOOP_ID );
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], SPEED_CLOSE_LOOP_ID );
	
}

//RM电机发送自己的电压控制
void Tx_RM_Motor_Iq_Input(void)
{
	//对于动量快系统，有一个电压数组，两个数组序号，一个发送的ID
	int16_t M_txbuf[4] = {0};
	uint8_t M_L_Pos = MOM_motor[MOMENTUM_L].id.buff_p, \
					M_R_Pos = MOM_motor[MOMENTUM_R].id.buff_p;
	
	uint32_t Tx_ID  = MOM_motor[MOMENTUM_L].id.tx_id;
	
	M_txbuf[M_L_Pos] = (int16_t)chassic.info.M_L_M.Voltage;
	M_txbuf[M_R_Pos] = (int16_t)chassic.info.M_R_M.Voltage;
	
	CAN2_Send_With_int16_to_uint8(Tx_ID, M_txbuf);
	
}
//-----------------------------------------处理措施







//清除LQR计算的扭矩、电流、电压,允许系统更新,恢复原始LQR,清除窗口数组

void Chassic_Stop(void)
{
	
	//清除4个电机的系统输入
	memset( &chassic.info.C_L_M, 0, sizeof(Sys_Input_t) );
	memset( &chassic.info.C_R_M, 0, sizeof(Sys_Input_t) );
	memset( &chassic.info.M_L_M, 0, sizeof(Sys_Input_t) );
	memset( &chassic.info.M_R_M, 0, sizeof(Sys_Input_t) );
	
	
	//对于9025系统
	KT_motor[LEG_L].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_L].KT_motor_info.tx_info.speedControl = 0;
	KT_motor[LEG_L].tx_W_cmd( &KT_motor[LEG_L], TORQUE_CLOSE_LOOP_ID);
	
	KT_motor[LEG_R].KT_motor_info.tx_info.iqControl = 0;
	KT_motor[LEG_R].KT_motor_info.tx_info.speedControl = 0;
	KT_motor[LEG_R].tx_W_cmd( &KT_motor[LEG_R], TORQUE_CLOSE_LOOP_ID);
	
	

	//对于动量快系统
	int16_t M_txbuf[4] = {0}; 
	uint32_t Tx_ID  = MOM_motor[MOMENTUM_L].id.tx_id;
	CAN2_Send_With_int16_to_uint8( Tx_ID, M_txbuf );
	
}

//遥控器失联底盘措施,不用虚函数了,在这里写较方便
void Chassic_In_RC_OFFLINE_Handler(void)
{
	//清除用户数据
	memset(&chassic.users, 0 , sizeof(User_typedef_t));
	
	//测量值处理,把9025算出来的总弧度清除
	Chassic_Status_Meas_UP(&chassic.info.Status);
	Chassic_Gesture_TOF_Meas_UP();
	Chas_Clear_X_Info();
	
	
	//目标值处理
	float yaw_meas = imu_sensor.info->base_info.yaw * ANGLE_CONVERSION_RADIAN;
	memset( &chassic.info.Status.tar.C_X, 0, sizeof(status_t) );
	chassic.info.Status.tar.Yaw = yaw_meas;
	
	//误差值依旧更新
	Chassic_Status_Err_UP(&chassic.info.Status);

}


//-----------------------------------------检测异常状态函数



void Chassic_Unnormal_State_Handler(void)
{
	Chassic_Judge_Is_In_Air();
	
	Check_KT_Motor_State();
	
	Momen_Offline_Handler();
}


__weak void Chassic_Flag_Set(const char flag)
{
	
}
__weak void Chas_Show_UI_Stuck(const char flag)
{
	
}


void Chassic_Judge_Is_In_Air(void)
{
	float L_spd = abs_(chassic.info.Status.meas.C_V_L), \
				R_spd = abs_(chassic.info.Status.meas.C_V_R), \
				g     = imu_sensor.info->base_info.g, \
				roll  = imu_sensor.info->base_info.roll, \
				vert_x = chassic.info.TOF_Meas.L_Vert_X;
	
	//这里只去获取离地前的失重状态，结束飞坡或下台阶后的1秒内再去获取是否失重
	if(imu_sensor.info->off_ground_flag == 1 && chassic.users.imu_off_ground == 0)
	{
		//延时1秒，这样能过滤噪音较多的重力加速度
		if(chassic.users.bounce_delay_cnt != 0)
		{
			if(HAL_GetTick() >= chassic.users.bounce_delay_cnt + 1000)
			{
				chassic.users.bounce_delay_cnt = 0;
			}
		}
		else
		{
			chassic.users.imu_off_ground = 1;
		}
		
	}
	

	
	if( State.chassic_flag.is_in_air == Fly_Out_Of_Ground )
	{
		/*-----------------动量块的处理开始--------------*/
		chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = \
		chassic.info.M_LQR_Gain[P] = chassic.info.M_LQR_Gain[P1] = 0;
		chassic.info.M_LQR_Gain[M_X] = 10*L_M_LQR_Param[M_X];

		if(State.chassic_flag.chas_ws_dir == ALINE_TO_F)
		{
		  chassic.info.Status.tar.M_X_L = -MOMENTUM_MAX_X_MEAS;
		  chassic.info.Status.tar.M_X_R = -MOMENTUM_MAX_X_MEAS;					
		}
		else
		{
		  chassic.info.Status.tar.M_X_L = MOMENTUM_MAX_X_MEAS;
		  chassic.info.Status.tar.M_X_R = MOMENTUM_MAX_X_MEAS;	
		}
		/*-----------------动量块的处理结束--------------*/
		
		/*-----------------底盘的处理开始-----------------*/
		Chas_Clear_X_Info();
		chassic.info.Status.tar.C_V = 0;
		Chassic_Stop();
			
		/*-----------------底盘的处理结束-----------------*/

        
  
		/*-----------------落地的处理开启------------------*/
		if( (vert_x <= CHAS_Horizontal_Height+2.f) && (g > 0) )
		{
			Chassic_Flag_Set(Bounce_Up_Of_Ground);		
		}
		
		/*-----------------落地的处理结束------------------*/
	}
 
 
	else if( State.chassic_flag.is_in_air == Bounce_Up_Of_Ground )
	{
  
		/*-----------------动量块的处理开始--------------*/
		chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = \
		chassic.info.M_LQR_Gain[P] = chassic.info.M_LQR_Gain[P1] = 0;
		chassic.info.M_LQR_Gain[M_X] = 10*L_M_LQR_Param[M_X];
		/*-----------------动量块的处理结束--------------*/
		
		
		/*-----------------底盘的处理开始-----------------*/
		chassic.info.Status.tar.C_V = 0;
		chassic.users.torque_compensation_request = 1;
		Chassic_Stop();
		Chas_Clear_X_Info();

		/*-----------------底盘的处理结束-----------------*/
		
		/*-----------------弹跳的处理开始-----------------*/

		//第一次弹起
		if( (vert_x >=(CHAS_Horizontal_Height + 4.f) ) )
		{
			Chassic_Flag_Set(Fly_Out_Of_Ground);
		}	
		
		//稳定在地上
		if( (vert_x <=(CHAS_Horizontal_Height + 2.f) ) && (vert_x >=(CHAS_Horizontal_Height - 1.f) ) )
		{
			Chassic_Flag_Set(ON_GROUND);
			Chassic_Flag_Set(TORQUE_CTRL);
			chassic.users.imu_off_ground = 0;
			chassic.users.torque_compensation_request = 0;
			chassic.users.bounce_delay_cnt = HAL_GetTick();
		}	
			
	}
	
	
	else if( State.chassic_flag.is_in_air == ON_GROUND )
	{
		
		/*--------------------检测开始----------------------*/
//			
//		if( chassic.users.imu_off_ground == 1 )
//		{
//			Chassic_Flag_Set(Fly_Out_Of_Ground);
//		}
		if( (vert_x >=(CHAS_Horizontal_Height + 5.f)) && (g < -5.f) )
		{
			Chassic_Flag_Set(Fly_Out_Of_Ground);
		}	
		
		/*--------------------检测结束----------------------*/
		if(State.chassic_flag.now_state != STUCK)
			Chassic_Flag_Set(TORQUE_CTRL);
		
		chassic.info.Status.tar.M_X_L = 0;
		chassic.info.Status.tar.M_X_R = 0;

	}
	


}





//-----------------------------------------辅助函数






/**
 *	@brief	动量块当前位置转变为位移
 */
float PosMeas_2_XMeas(uint16_t pos_meas, uint16_t mid_pos_meas, MOM_motor_list_e name)
{
  float scale = 0;//刻度，记录电机走1m要走多少机械角度
	
	float x_meas = 0;
	
	//确保左动量快在前面的位移是正的
	if( name == MOMENTUM_L )
	{	
		scale = ((float)L_MOMENTUM_B_2_F_ANGLE_SUM) / CHASSIC_LENGTH ; 
		x_meas = ((float)(mid_pos_meas - pos_meas)) / scale ;
	}
	else if( name == MOMENTUM_R )
	{	
		scale = ((float)R_MOMENTUM_B_2_F_ANGLE_SUM) / CHASSIC_LENGTH ; 
		x_meas = ((float)(pos_meas - mid_pos_meas)) / scale ;	
	}
	
	x_meas = constrain(x_meas, -MOMENTUM_MAX_X_MEAS, MOMENTUM_MAX_X_MEAS);

	return x_meas;
}

/**
 *	@brief	底盘映射目标值
 */
void Chassic_Sim(float WS, float AD, float tar_v)
{
	//左右系统的速度目标一致
	//Chassic_X_Speed_Tar_Dynamic_Changes(WS);
	if(JUDGE_ONLINE)
	{
		if(State.chassic_flag.speed_up_switch == OPEN)
			chassic.info.Status.tar.C_V = WS / CH_MAX * tar_v;
		else
			chassic.info.Status.tar.C_V = WS / CH_MAX * tar_v;
	}
	else
	{
		chassic.info.Status.tar.C_V = WS / CH_MAX * Judge_Offline_X_Speed;
	}
	
	
	float v_tar = chassic.info.Status.tar.C_V, \
				v_err = chassic.info.Status.err.C_V;	
	
	float yaw_tar = chassic.info.Status.tar.Yaw, \
				yaw_err = chassic.info.Status.err.Yaw;

	
	if( abs_(yaw_err) < MAX_YAW_ANGLE_ERR * ANGLE_CONVERSION_RADIAN )
	{
		//如果只是原地旋转
		if( v_tar == 0 && abs_(v_err) < SPIN_STATE_X_SPEED )
			yaw_tar += AD / -CH_MAX * ONLY_SPIN_RATE;  
		
		//如果是还没刹车完就旋转
		if( v_tar == 0 && abs_(v_err) > SPIN_STATE_X_SPEED )
			yaw_tar += AD / -CH_MAX * SPIN_AS_BRAKING_RATE;  
		
		//如果是前进的时候要旋转
		if( v_tar != 0 )
			yaw_tar += AD / -CH_MAX * SPIN_AS_FORWARD_RATE;  
	
	}

	yaw_tar = Float_ZeroManage( yaw_tar, VALUE_PI );
	
	chassic.info.Status.tar.Yaw = yaw_tar; 
	
}






//bias_angle是想要变为0的原机械角度
void Chassic_Yaw_Motor_Angle_Standard(Sys_Status_t *status, 
																			const uint16_t now_angle,
																			const uint16_t bias_angle)
{
	status->meas.Y_Pos = now_angle;
	
	uint16_t err = RM_TOTAL_ANGLE - bias_angle;
	
	status->meas.Y_Standard_Pos = status->meas.Y_Pos + err;
	
	if(status->meas.Y_Standard_Pos >= RM_TOTAL_ANGLE)
		status->meas.Y_Standard_Pos -= RM_TOTAL_ANGLE;
	
}

void Momentum_At_Limited_Pos_Protect(void)
{
	float M_X_L = chassic.info.Status.meas.M_X_L, \
		    M_X_R = chassic.info.Status.meas.M_X_R;
	
	float M_L_T = chassic.info.M_L_M.Tx_Torque, \
		    M_R_T = chassic.info.M_R_M.Tx_Torque;
	
	//左动量块在最前或在最后
	if( abs_(M_X_L) >= MOMENTUM_MAX_X_MEAS - 0.005f ) //0.17m
	{
		//动量块在前(即正的)且扭矩一直往前(即负的)
		//动量块在后(即负的)且扭矩一直往后(即正的)
		if( M_X_L * M_L_T < 0 )
		{
			chassic.info.M_L_M.Tx_Torque = 0;
			chassic.info.M_L_M.Voltage /= 5;
		}
	}
	
	if( abs_(M_X_R) >= MOMENTUM_MAX_X_MEAS - 0.005f )
	{
		//动量块在前(即正的)且扭矩一直往前(即负的)
		//动量块在后(即负的)且扭矩一直往后(即正的)
		if( M_X_R * M_R_T < 0 )
		{
			chassic.info.M_R_M.Tx_Torque = 0;
			chassic.info.M_R_M.Voltage /= 5;
		}
			
	}
	
}


//如果底盘速度在起步最大速度附近且此时遥控器通道值一直都拉满
//就让动态提高目标速度
void Chassic_X_Speed_Tar_Dynamic_Changes(float WS)
{
//	float spd_meas = chassic.info.Status.meas.C_V;
//	float spd_err  = chassic.info.Status.err.C_V;
//	
//	//第一阶段,先到达最大起步
//	if(abs_(spd_meas) <= X_MAX_START_SPEED - 0.2f) //当前速度小于1.8m/s
//		chassic.info.Status.tar.C_V = WS / CH_MAX * X_MAX_START_SPEED;
//		
//	//第二阶段,提高目标速度
//	else  //当前速度大于1.8m/s
//	{
//		//想要正向再加速
//		if(WS ==  CH_MAX)
//		{
//			if(abs_(spd_err) <= 0.2f)
//				chassic.info.Status.tar.C_V += 0.5f;
//		}
//		
//		//想要反向再加速
//		else if(WS ==  CH_MIN)
//		{
//			if(abs_(spd_err) <= 0.2f)
//				chassic.info.Status.tar.C_V -= 0.5f;
//		}
//		
//		else
//		{
//			chassic.info.Status.tar.C_V = WS / CH_MAX * abs_(spd_meas);
//		}
//		
//		chassic.info.Status.tar.C_V = constrain(chassic.info.Status.tar.C_V,
//																						-X_MAX_START_SPEED,
//																						X_MAX_START_SPEED );
//	}
}



//刹车调整位移目标0点
int16_t Last_WS_Val, This_WS_Val;
uint8_t brake_over;
void Chassic_Brake_Over_Dynamic_Ctrl(void)
{
	This_WS_Val = RC_LEFT_CH_UD_VALUE;
	
	//开启刹车调整
	if(Last_WS_Val != This_WS_Val && \
		 abs_(chassic.info.Status.meas.C_V) > 0.5f && \
		 This_WS_Val == 0)
	{	
		Chassic_Flag_Set(OPEN);
		
		chassic.users.brake_over_dir = one(chassic.info.Status.meas.C_V);
	}
	//如果突然动了遥控了
	if(This_WS_Val != 0)
	{
		brake_over = 0;
		Chassic_Flag_Set(CLOSE);
	}
		
		
	//如果刹车一瞬间突然要开启小陀螺了
	if(State.chassic_flag.chas_motion_mode == SMALL_TOP_ON)
	{
		Chassic_Flag_Set(CLOSE);
		brake_over = 0;
	}	
	
	if(State.chassic_flag.brake_ctrl_switch == OPEN)
	{				
		brake_over = 1;	
		
		//刹车控制可以关闭了
		if(abs_(chassic.info.Status.meas.Pitch) < 3.f && abs_(chassic.info.Status.meas.C_V) < 0.02f)
		{
			if(brake_over == 1)
			{
				Chas_Clear_X_Info();
			}			
			brake_over = 0;		
		}
	}
	//开启刹车控制
	if( brake_over == 1 )
	{
		chassic.info.M_LQR_Gain[M_X] = L_M_LQR_Param[M_X]*4.0f;
		chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = chassic.info.M_LQR_Gain[P] = 0;
		
		//chassic.info.C_LQR_Gain[M_X] = chassic.info.C_LQR_Gain[M_V] = 0;
		chassic.info.Status.tar.M_X_L = chassic.info.Status.tar.M_X_R = chassic.users.brake_over_dir*MOMENTUM_MAX_X_MEAS;
	}
	//关闭刹车控制
	else
	{
		chassic.info.Status.tar.M_X_L = chassic.info.Status.tar.M_X_R = 0;
		chassic.info.M_LQR_Gain[M_X] = L_M_LQR_Param[M_X]*4.0f;
		Chassic_Flag_Set(CLOSE);

	}		
	
	Last_WS_Val = This_WS_Val;
}

//姿态距离结算
void Chassic_Gesture_TOF_Meas_UP(void)
{
	//原始测距数据
	chassic.info.TOF_Meas.L_Raw_X = TOF_Sensor[POS_L].info.raw_dis_meas;
	chassic.info.TOF_Meas.R_Raw_X = TOF_Sensor[POS_R].info.raw_dis_meas;
	
	//计算得到的垂直距离
	chassic.info.TOF_Meas.L_Vert_X = arm_cos_f32(abs_(chassic.info.Status.meas.Pitch)) * \
																							 chassic.info.TOF_Meas.L_Raw_X;
	chassic.info.TOF_Meas.R_Vert_X = arm_cos_f32(abs_(chassic.info.Status.meas.Pitch)) * \
																							 chassic.info.TOF_Meas.R_Raw_X;

}





/*检查轮子是否持续高电流*/
void Check_KT_Motor_State(void)
{
	if( abs_(Leg_L_info->temperature) > 80.f )
	{
		if(chassic.users.L_M_Over_I_Time == 0)
			chassic.users.L_M_Over_I_Time = HAL_GetTick();
		
		if(HAL_GetTick() >= (chassic.users.L_M_Over_I_Time + 8000))
			Chassic_Stop();
	}
	else
		chassic.users.L_M_Over_I_Time = 0;
	
	if( abs_(Leg_R_info->temperature) > 80.f )
	{
		if(chassic.users.R_M_Over_I_Time == 0)
			chassic.users.R_M_Over_I_Time = HAL_GetTick();
		
		if(HAL_GetTick() >= (chassic.users.R_M_Over_I_Time + 8000))
			Chassic_Stop();
	}
	else
		chassic.users.R_M_Over_I_Time = 0;	
	
}

/*清除位移信息*/
void Chas_Clear_X_Info(void)
{
	Leg_L_info->encoder_prev = Leg_R_info->encoder_prev = 0;
	Leg_L_info->encoder_sum = Leg_R_info->encoder_sum = 0;
	Leg_L_info->radian_sum = Leg_R_info->radian_sum = 0;
  chassic.info.Status.meas.C_X = 0;	
}

/*动量块失联处理*/
float M_X_param;
void Momen_Offline_Handler(void)
{
	motor_state_e  L_state = MOM_motor[MOMENTUM_L].state.work_state;
	motor_state_e  R_state = MOM_motor[MOMENTUM_R].state.work_state;
	
	//只获取一次
	if(M_X_param == 0)
		M_X_param	= L_C_LQR_Param[M_X];
	
	if(L_state == M_OFFLINE && R_state == M_OFFLINE)
	{
		chassic.users.momentum_offline = 1;
		L_C_LQR_Param[M_X] = H_C_LQR_Param[M_X] = 0;
	}
	else if(L_state == M_OFFLINE && R_state == M_ONLINE)
	{
		chassic.users.momentum_offline = 1;
		
		chassic.info.Status.tar.M_X_R = -chassic.info.Status.meas.M_X_L;				
				
		chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = \
		chassic.info.M_LQR_Gain[P] = 0;
		chassic.info.M_LQR_Gain[M_X] = 3*L_M_LQR_Param[M_X];
	}
	else if(L_state == M_ONLINE && R_state == M_OFFLINE)
	{
		chassic.users.momentum_offline = 1;
		
		chassic.info.Status.tar.M_X_L = -chassic.info.Status.meas.M_X_R;	
		
		chassic.info.M_LQR_Gain[C_X] = chassic.info.M_LQR_Gain[C_V] = \
		chassic.info.M_LQR_Gain[P] = 0;
		chassic.info.M_LQR_Gain[M_X] = 3*L_M_LQR_Param[M_X];
	}
	else
	{
		chassic.users.momentum_offline = 0;
		
		if(L_C_LQR_Param[M_X] == 0)
			L_C_LQR_Param[M_X] = H_C_LQR_Param[M_X] = M_X_param;
	}
	
}
