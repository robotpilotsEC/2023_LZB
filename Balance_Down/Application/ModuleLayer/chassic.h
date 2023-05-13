#ifndef __CHASSIC_H
#define __CHASSIC_H

#include "device.h"
#include "algo.h"
#include "State.h"
#include "Master_Comm.h"

//-----------------------------���̺Ͷ�������Ϣ��--------------------------------
//״̬��������ֵ |   �仯����  
//
//����λ��			 |   ����ͷ�� +
//�����ٶ�			 |   ����ͷ�� +
//������λ��     |	 ����ͷ�� +
//�������ٶ�     |   ����ͷ�� +
//Pitch�Ƕ�      |   ����ǰ�� -
//Pitch���ٶ�    |   ����ǰ�� -
//Yaw�Ƕ�        |   ����ת   +
//Yaw���ٶ�      |   ����ת   +
//Roll�Ƕ�			 |  ����ͷ�����б + 
//Yaw���        |   ���4096 ��ǰ0����2080���� 6185

//�����������ֵ |   �仯����    |    ����ĵ���/��ѹ   |   ����ó���Ť��
//
//��9025λ���ٶ� |   ����ͷ�� +  |    ��ǰת -          |   ��ǰת -     
//��9025λ���ٶ� |   ����ͷ�� -  |    ��ǰת +          |   ��ǰת - 
//��6020λ���ٶ� |   ����ͷ�� +  |    ��ǰת -          |   ��ǰת - 
//��6020λ���ٶ� |   ����ͷ�� -  |    ��ǰת +          |   ��ǰת - 







/* Exported macro ------------------------------------------------------------*/
/*------------------------����Ϊ����Ҫ�õĺ궨��------------------------*/

#define STATUS_VAL_NUM  8
#define STRUCT_STATUS_UNNOT_UP_VAL_NUM   4  //status�ṹ���в�Ҫȥ�����������ݸ���
#define CHASSIC_MAX_POWER_BUFFER    (judge_info.power_heat_data.chassis_power_buffer)         







/*-------------------------����Ϊ��������-----------------------------------*/\
#define CHASSIC_LENGTH			0.36f						//��ͷ����β�ľ���
#define WHEEL_RADIUS        0.08f          	//���Ӱ뾶      m��λ//0.1
#define WHEEL_GIRTH					( 2 * VALUE_PI * WHEEL_RADIUS ) //�����ܳ� m��λ
#define MOMENTUM_RADIUS			0.033f					//6020�뾶  
#define MOMENTUM_GIRTH      ( 2 * VALUE_PI * MOMENTUM_RADIUS ) //�������ܳ�
//���������е�Ϊλ��Ϊ0�ĵ㣬��ǰ����λ���ǳ���
#define MOMENTUM_MAX_X_MEAS   (CHASSIC_LENGTH / 2)  

//���������Ӻ�ǰ�ĵ���ǶȺ�6590 
#define L_MOMENTUM_B_2_F_ANGLE_SUM	6150
#define R_MOMENTUM_B_2_F_ANGLE_SUM	6370 							


//ʵ���ٶ�ת��Ϊ���͸�������ٶ�
//SPEED_CONTROL * 0.01 / 360 * �ܳ� = SPEED(m/s)
#define KT_SPEED_TO_SPEED_CONTROL(SPEED)   (SPEED / WHEEL_GIRTH * 360.f * 100.f)  


/*-----------------��������------------*/
//��ǰ��,ǰ���1.8cm,����25.6cm;  ����,ǰ���24cm,����2.1cm;  ˮƽ��11.5cm
#define CHAS_F_TOF_MIN_RAW_X				(1.8f)
#define CHAS_F_TOF_MAX_RAW_X				(25.6f)
#define CHAS_B_TOF_MIN_RAW_X				(2.1f)
#define CHAS_B_TOF_MAX_RAW_X				(24.f)
#define CHAS_Horizontal_Height      (11.5f)//11.5
#define STEP_HEIGHT									(23.f)		//̨�׸߶�

//�ϳ����������13.6���������12.6��15�������13.6

/*------------------------����Ϊ�������ƵĲ���--------------------------------*/
#define H_X_MAX_START_SPEED      2.5f          //�������������ٶ�
#define M_X_MAX_START_SPEED      1.7f
#define L_X_MAX_START_SPEED      1.5f          //�������������ٶ�
#define Judge_Offline_X_Speed  	 0.9f          //����ʧ��ʱ��������ٶ�
#define SMALL_TOP_YAW_SPIN_RATE	  0.01f   //С����yaw�Ƕȱ仯��
#define H_SPEED_CONTROL    			 0.6f
#define L_SPEED_CONTROL          0.2f

#define CHASSIC_MAX_PITCH_ANGLE   (6.f*ANGLE_CONVERSION_RADIAN) //���������̬
#define MAX_SPEED_TORQUE    (	abs_(L_C_LQR_Param[P]) * CHASSIC_MAX_PITCH_ANGLE + \
															abs_(L_C_LQR_Param[M_X]) * MOMENTUM_MAX_X_MEAS ) //1����������35Ť��
					 										

																				


//--���½�Ϊ���Եó����²���Ը��ݵ�ǰ���ٶȵĴ�С�����������ת����
#define MAX_YAW_ANGLE_ERR			20        
#define SPIN_STATE_X_SPEED    0.15f       //��ת��ʱ�����ٶ��Ǵ��Ϊ0.1���ң���������0.15  
#define ONLY_SPIN_RATE	 			0.02f			//ԭ����ת����
#define SPIN_AS_BRAKING_RATE  0.02f     //ɲ������ת������
#define SPIN_AS_FORWARD_RATE	0.01f		//ǰ������ת������






/* Exported types ------------------------------------------------------------*/




//״̬�����������������
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



//״̬����������λ
typedef struct status
{
	//ƽ��
	float  C_X;       
	float  C_V;		 
	
	//ƽ��
	float  M_X;       
	float  M_V;	
		
	float  Pitch;            
	float  Rate_Pitch;   

	float  Yaw;
	float  Rate_Yaw;	
	
	//----�������Ӹ���λ���ٶȼ��ٶ�
	float  C_X_L;
	float  C_V_L;
	
	float  C_X_R;
	float  C_V_R;

	float  M_X_L;
	float  M_V_L;
	
	float  M_X_R;
	float  M_V_R;
	
	//----�������������λ��λ���ٶ�
	uint16_t L_M_Pos; //ֻ�ò���ֵ��Ŀ������	
	uint16_t R_M_Pos; //ֻ�ò���ֵ��Ŀ������	
	
	//----YAW���ԭʼ�Ƕ�,�涨����ĽǶ�
	uint16_t Y_Pos;
	uint16_t Y_Standard_Pos;
	
}	status_t;
 

//�ұ�ϵͳ������λ�ơ��ٶȡ��������ٶȼӸ���
typedef struct Sys_Status
{
	status_t meas;
	status_t err;
  status_t tar;
	
} Sys_Status_t;

//���ϵͳ������Ť�ؼӸ���
//�����ӣ���-��=��ת�����Ť��,����=��ȥ��ת������Ť��
//�Զ����飬������ת�õ���Ť��
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
	//cm��λ
	float L_Raw_X;
	float L_Vert_X;

	float R_Raw_X;
	float R_Vert_X;
	
}Gesture_TOF_Info_t;

typedef struct
{
	/*��������´���*/
	uint8_t L_M_Over_I_Time;
	uint8_t R_M_Over_I_Time;
	
	/*ɲ��˲����ٶȷ���*/
	int  brake_over_dir;
	
	/*ɲ��˲�俪��С���ݵ�ʱ��,��֤pitch���ٶ������ϵ�*/
	uint8_t low_pitch_cnt;
	uint8_t low_speed_cnt;
	
	/*��ǽ����*/
	uint8_t stuck_cnt;
	
	/*����С����ǰ,�ȸ�һ��б������*/
	uint8_t start_spin_flag;
	float   spin_val;
	uint8_t spin_handler_flag; //��֪����ʱ��ת��ʱ��ƫ���ıߣ�1ƫǰ(pitch<0)2ƫ��(pitch>0)
	uint8_t spin_close_cnt; //����ر�ʱ����Ĵ���
	uint8_t spin_err; //�����г��ִ���
	
	/*����ڵ��Ϻ����һ����ʱ���ж���һ�ε�ʧ��*/
	uint32_t bounce_delay_cnt;

	/*������ʧ����Ŀ��Ʒ�ʽ*/
	uint8_t  momentum_offline;
	
	/*��ͬʱ��̨��*/
	uint8_t  roll_in_air_flag; 
	uint8_t  imu_off_ground; 
	 
	uint8_t  torque_compensation_request; //���Ť�ز���
	uint8_t  first_bounce; //1��ʾ��һ�������
	
	uint8_t  sideways_request; //б������
	uint8_t  sideways_over; //���������ˣ������޸ĵ��̵ĳ���
	uint8_t  sideways_dir; //������ת����
	
}User_typedef_t;


typedef struct Chassic_Info
{
	
	Sys_Status_t 	Status;
	
	Gesture_TOF_Info_t 	TOF_Meas;
	
	//�����������ҵ�����������������ҵ��
	Sys_Input_t   C_L_M;
	Sys_Input_t   C_R_M;
	Sys_Input_t   M_L_M;
	Sys_Input_t   M_R_M;
	
	//ѡ����������Ϊ�ڴ�������ʱҪ��state.c�ļ��޸����棬���ֲ�ϣ��State�ļ�����chassis�ļ�
	float         C_LQR_Gain[STATUS_VAL_NUM];
	float         M_LQR_Gain[STATUS_VAL_NUM];
	
}Chassic_Info_t;

typedef struct Chassic
{
	Chassic_Info_t       info;
	User_typedef_t       users;
	
	void                (*ctrl)(void);		//�ܿ���	
	
}Chassic_t;


__weak void Chassic_Flag_Set(const char flag);
__weak void Chas_Show_UI_Stuck(const char flag);
void Chassic_Stop(void);


//��������:(�����)- -  (�������)- -   (�Ƕȵ�)+ +  (yaw)- -
extern float  H_C_LQR_Param[STATUS_VAL_NUM];
extern float  L_C_LQR_Param[STATUS_VAL_NUM];
//��������:(�����)+ +  (�������)+ +   (�Ƕȵ�)- -  (yaw)0 0
extern float  H_M_LQR_Param[STATUS_VAL_NUM];
extern float  L_M_LQR_Param[STATUS_VAL_NUM];
extern Chassic_t chassic;

#endif