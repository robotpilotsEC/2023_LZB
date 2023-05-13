#include "offline.h"

void Momentum_Offline_Handler(void);
void KT_Motor_Offline_Handler(void);
void IMU_Offline_Handler(void);
void Judge_Sensor_Offline_Handler(void);
void Cap_Offline_Handler(void);
void TOF_Offline_Handler(void);
void Comm_Offline_Handler(void);
/**
	*@brief �����ⲿ����
*/
extern RM_motor_t MOM_motor[MOM_MOTOR_LIST];
extern KT_motor_t KT_motor[KT_MOTOR_LIST];


Device_State 	devices;


void Offline_Handler(void)
{
	Momentum_Offline_Handler();
	KT_Motor_Offline_Handler();
	IMU_Offline_Handler();
	Judge_Sensor_Offline_Handler();
	Cap_Offline_Handler();
	TOF_Offline_Handler();
	Comm_Offline_Handler();
}


/**
	*@brief 1.�������һ��ʧ���˵������Ϊ�˱���ƽ�⣬��һ������̶����෴λ�ò�����ʹ��LQR����
					2.������ֶ�ʧ������������̹رչ��ڶ�������������
*/
void Momentum_Offline_Handler(void)
{
	motor_state_e  L_state = MOM_motor[MOMENTUM_L].state.work_state;
	motor_state_e  R_state = MOM_motor[MOMENTUM_R].state.work_state;
	
	if(L_state == M_OFFLINE)
		devices.state.bit.rm_l = UnNormal;
	else
		devices.state.bit.rm_l = Normal;
	
	if(R_state == M_OFFLINE)
		devices.state.bit.rm_r = UnNormal;
	else
		devices.state.bit.rm_r = Normal;
	
	//����chassic.c�㴦��Ϸ���,Momen_Offline_Handler����
}

/**
	*@brief ֻҪ����ʧ����һֱ����0��������֤�л���ص��
*/
void KT_Motor_Offline_Handler(void)
{
	motor_state_e  L_state = KT_motor[LEG_L].KT_motor_info.state_info.work_state;
	motor_state_e  R_state = KT_motor[LEG_R].KT_motor_info.state_info.work_state;
	
	if(L_state == M_OFFLINE)
		devices.state.bit.kt_l = UnNormal;
	else
		devices.state.bit.kt_l = Normal;
	
	if(R_state == M_OFFLINE)
		devices.state.bit.kt_r = UnNormal;
	else
		devices.state.bit.kt_r = Normal;
	
	
	if(L_state == M_OFFLINE || R_state == M_OFFLINE)
	{
		Chassic_Stop();
		State_Reset();
	}
		
	
}

/**
	*@brief ֻҪ����ʧ����������Ť�ؿ��ƣ�һֱ����0����
*/
uint32_t time_this;
uint8_t imu_data_true;
uint16_t err_cnt;
void IMU_Offline_Handler(void)
{
	if(imu_sensor.info->init_flag == 1)
	{
		if(time_this == 0 && imu_data_true == 0)
			time_this = HAL_GetTick();
		
		if(HAL_GetTick() - time_this >= 2000)
		{
			if( (imu_sensor.info->base_info.pitch == 0) && \
					(imu_sensor.info->base_info.yaw == 0) && \
					(imu_sensor.info->base_info.roll == 0) && \
					(imu_sensor.info->base_info.rate_pitch == 0) && \
					(imu_sensor.info->base_info.rate_yaw == 0) && \
					(imu_sensor.info->base_info.rate_roll == 0) )
			{	
				err_cnt++;
				
				if(err_cnt >= 1000)
				{
					devices.state.bit.imu = UnNormal;
					Chassic_Stop();
					__set_FAULTMASK(1); 
					NVIC_SystemReset();
				}
				
			}
			else
			{
				err_cnt = 0;
				devices.state.bit.imu = Normal;
				imu_data_true = 1;
				time_this = 0;
			}
			
		}
			

	}
	
	if(State.chassic_flag.gimb_imu_state == UNNORMAL)
	{
		Chassic_Stop();
		State_Reset();
	}
}

/**
	*@brief �������Ƶ��̵��ٶȣ����ҷ��������ص�����Ҳ��������һЩ
*/
void Judge_Sensor_Offline_Handler(void)
{
	//����Master_Tx_Info_UP���������˴���������������Ϣ
	//����Chassic_Status_Tar_UP�������˴�������ǰ���ٶ�
	if(JUDGE_OFFLINE)
		devices.state.bit.judge = UnNormal;
	else
		devices.state.bit.judge = Normal;
}

/**
	*@brief ����ʧ��������ǰ���ٶȣ������ٶȣ������ٶ�
*/
void Cap_Offline_Handler(void)
{
	if(Super_2023.work_state == DEV_OFFLINE)
		devices.state.bit.super = UnNormal;
	else
		devices.state.bit.super = Normal;
	
}


/**
	*@brief ���ʧ��
*/
void TOF_Offline_Handler(void)
{
	
}

/**
	*@brief ���ʧ��
*/
void Comm_Offline_Handler(void)
{
	if(Master.work_state == DEV_OFFLINE)
		devices.state.bit.comm = UnNormal;
	else
		devices.state.bit.comm = Normal;
}
