#ifndef __kalman_filter_H
#define __kalman_filter_H

#include "stm32f4xx.h"                  // Device header
#include "imu_sensor.h"


#define OBSERVE_STATE_NUM 2						//状态变量个数
#define dT                1.f 			  //在设置B矩阵和更新先验估计值会用到
#define Q_MATRIX_ELEMENT  0.0025f			//由于是方差，必须为正
#define R_MATRIX_ELEMENT  0.0001f				


#define Kalman_Filter_DeInit 		Kal_Filter.Init_Flag == DeInit


//初始条件设置标志	
typedef enum Filter_Init_e
{
	Init = 0,
	DeInit,
	
}Filter_Init_e;
		
//测量值
typedef struct Measure_Data
{
	
	short gyro_x;
	short gyro_y;
	
	short acc_x;
	short acc_y;
		
}Measure_Data;

//先验估计值和它的协方差矩阵
typedef struct Prior_Hat
{
	float gyro_x;
	float gyro_y;
	
	float Prior_Hat_Err_Cov[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
}Prior_Hat;

//系统输入
typedef struct System_Input
{
	short acc_x;
	short acc_y;
	
}System_Input;

//后验估计值和它的协方差矩阵
typedef struct Posterior_Hat
{
	float gyro_x;
	float gyro_y;
	
	float Posterior_Hat_Err_Cov[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
}Posterior_Hat;

//矩阵
typedef struct Matrix
{
	uint8_t State_Tran_A[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	uint8_t Ctrl_B[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	uint8_t Observed_H[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	uint8_t Unit_Matrix_I[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	float Process_Noise_Cov_Q[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];

	float Measure_Noise_Cov_R[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
	float Kalman_Gain[OBSERVE_STATE_NUM][OBSERVE_STATE_NUM];
	
}Matrix;
//时间记录器
typedef struct Time_Cnt
{
	uint32_t Time_Last;
	
	uint32_t Time_This;
	
	uint32_t Time_dT;
	
}Time_Cnt;

//把后验估计值保存到这里
typedef struct Kal_Filter_Pass_Data
{
	float gyro_x;
	float gyro_y;
	
}Kal_Filter_Pass_Data;
//卡尔曼滤波器
typedef struct Kalman_Filter
{
	Measure_Data   				Meas_Data; //实际中测得的
	Prior_Hat 						Pri_Hat;
	System_Input					Sys_Input;
	Posterior_Hat   			Post_Hat;
	Matrix								All_Matrix;
	Time_Cnt        			Time;
	Filter_Init_e   			Init_Flag; 
	Kal_Filter_Pass_Data  KalPass_Data;
	
	
	
	void						(*Set_Initial)( struct Kalman_Filter *Kal_Filter );
	void						(*Data_Update)( struct Kalman_Filter *Kal_Filter );
	void						(*Predict)( struct Kalman_Filter *Kal_Filter );
	void						(*Correct)( struct Kalman_Filter *Kal_Filter );
	void            (*KalPass_Data_Update)( struct Kalman_Filter *Kal_Filter );
	
}Kalman_Filter;

extern Kalman_Filter Kal_Filter;

void Start_Kalman_Filter(void);

#endif
