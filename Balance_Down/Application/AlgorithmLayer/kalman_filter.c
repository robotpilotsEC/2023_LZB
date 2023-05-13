#include "kalman_filter.h"


void Set_Matrix( Matrix *All_Matrix );	
void Set_Posterior_Hat_Initial( Posterior_Hat *Post_Hat );
void Set_Initial_Step( struct Kalman_Filter *Kal_Filter );


void Meas_Data_Updata( Measure_Data  *Meas_Data );
void Prior_Hat_Update( Kalman_Filter *Kal_Filter );
void Posterior_Hat_Update( Kalman_Filter *Kal_Filter );
void System_Input_Update( Kalman_Filter *Kal_Filter );
void Kalman_Gain_Update( Kalman_Filter *Kal_Filter );
void Data_Update_Step( struct Kalman_Filter *Kal_Filter );


void Predict_Step( struct Kalman_Filter *Kal_Filter );


void Correct_Step( struct Kalman_Filter *Kal_Filter );


void KalPass_Data_Update( Kalman_Filter *Kal_Filter );


/**
 *	@brief	创建滤波器
 */
Kalman_Filter Kal_Filter = 
{
	.Init_Flag   = DeInit, //一开始设置为未初始化
	
	
//下面是函数指针	
	.Set_Initial = Set_Initial_Step,
	
	.Data_Update = Data_Update_Step,
	
	.Predict     = Predict_Step,
	
	.Correct     = Correct_Step,
	
	.KalPass_Data_Update = KalPass_Data_Update,
};










/**
 *	@brief	设置矩阵
 */
void Set_Matrix( Matrix *All_Matrix )
{
	//设置A矩阵
	All_Matrix->State_Tran_A[0][0] = 1;
	All_Matrix->State_Tran_A[0][1] = 0;
	All_Matrix->State_Tran_A[1][0] = 0;
	All_Matrix->State_Tran_A[1][1] = 1;
	
	//设置B矩阵
	All_Matrix->Ctrl_B[0][0] = dT;
	All_Matrix->Ctrl_B[0][1] = 0;
	All_Matrix->Ctrl_B[1][0] = 0;
	All_Matrix->Ctrl_B[1][1] = dT;
	
	//设置H矩阵
	All_Matrix->Observed_H[0][0] = 1;
	All_Matrix->Observed_H[0][1] = 0;
	All_Matrix->Observed_H[1][0] = 0;
	All_Matrix->Observed_H[1][1] = 1;
	
	//设置I矩阵
	All_Matrix->Unit_Matrix_I[0][0] = 1;
	All_Matrix->Unit_Matrix_I[0][1] = 0;
	All_Matrix->Unit_Matrix_I[1][0] = 0;
	All_Matrix->Unit_Matrix_I[1][1] = 1;
	
	//设置Q矩阵
	All_Matrix->Process_Noise_Cov_Q[0][0] = Q_MATRIX_ELEMENT;
	All_Matrix->Process_Noise_Cov_Q[0][1] = 0;
	All_Matrix->Process_Noise_Cov_Q[1][0] = 0;
	All_Matrix->Process_Noise_Cov_Q[1][1] = Q_MATRIX_ELEMENT;
	
	//设置R矩阵
	All_Matrix->Measure_Noise_Cov_R[0][0] = R_MATRIX_ELEMENT;
	All_Matrix->Measure_Noise_Cov_R[0][1] = 0;
	All_Matrix->Measure_Noise_Cov_R[1][0] = 0;
	All_Matrix->Measure_Noise_Cov_R[1][1] = R_MATRIX_ELEMENT;
}

/**
 *	@brief	设置初始的后验估计和协方差矩阵
 */
void Set_Posterior_Hat_Initial( Posterior_Hat *Post_Hat )
{
	Post_Hat->gyro_x = 0.f;
	Post_Hat->gyro_y = 0.f;
	
	Post_Hat->Posterior_Hat_Err_Cov[0][0] = 1.f;
	Post_Hat->Posterior_Hat_Err_Cov[0][1] = 0.f;
	Post_Hat->Posterior_Hat_Err_Cov[1][0] = 0.f;
	Post_Hat->Posterior_Hat_Err_Cov[1][1] = 1.f;
}

/**
 *	@brief	设置初始状态步骤
 */
void Set_Initial_Step( Kalman_Filter *Kal_Filter )
{	
	Set_Matrix( &( Kal_Filter->All_Matrix ) );
	
	Set_Posterior_Hat_Initial( &( Kal_Filter->Post_Hat ) );
	
	Kal_Filter->Init_Flag = Init;
}


/**
 *	@brief	保存要用的测量值
 */
void Meas_Data_Updata( Measure_Data  *Meas_Data )
{
	Meas_Data->gyro_x = imu_sensor.info->raw_info.gyro_x;
	
	Meas_Data->gyro_y = imu_sensor.info->raw_info.gyro_y;
	
	Meas_Data->acc_x  = imu_sensor.info->raw_info.acc_x;
	
	Meas_Data->acc_y  = imu_sensor.info->raw_info.acc_y;
		
}
/**
 *	@brief	保存估计方程的系统输入加速度
 */
void System_Input_Update( Kalman_Filter *Kal_Filter )
{
	Measure_Data    *Meas_Data  = &( Kal_Filter->Meas_Data );
	System_Input		*Sys_Input	= &( Kal_Filter->Sys_Input );
	
	Sys_Input->acc_x = Meas_Data->acc_x;
	
	Sys_Input->acc_y = Meas_Data->acc_y;
}
/**
 *	@brief	更新数据步骤
 */
void Data_Update_Step( Kalman_Filter *Kal_Filter )
{
	Meas_Data_Updata( &( Kal_Filter->Meas_Data ) );
	
	System_Input_Update( Kal_Filter );

}


/**
 *	@brief	先验估计值和它的协方差更新
 */
void Prior_Hat_Update( Kalman_Filter *Kal_Filter )
{
	Prior_Hat 			*Pri_Hat	 	= &( Kal_Filter->Pri_Hat );
	System_Input		*Sys_Input	= &( Kal_Filter->Sys_Input );
	Posterior_Hat   *Post_Hat  	= &( Kal_Filter->Post_Hat );
	Matrix					*All_Matrix = &( Kal_Filter->All_Matrix );
	Time_Cnt        *Time       = &( Kal_Filter->Time );
	
//注意，这一次的先验估计值和先验误差的协方差矩阵都要用到上一次的后验估计值和后验误差的协方差矩阵
	
	//获取时间间隔，ms单位
	Time->Time_This = HAL_GetTick();
	
	Time->Time_dT = Time->Time_This - Time->Time_Last;
	
	Time->Time_Last = Time->Time_This;
	
	//先验估计值gyro_x
	Pri_Hat->gyro_x = Post_Hat->gyro_x + Time->Time_dT * Sys_Input->acc_x ;
	
	//先验估计值gyro_y
	Pri_Hat->gyro_y = Post_Hat->gyro_y + Time->Time_dT * Sys_Input->acc_y ;
	
	//先验误差的协方差矩阵
	Pri_Hat->Prior_Hat_Err_Cov[0][0] = Post_Hat->Posterior_Hat_Err_Cov[0][0] + All_Matrix->Process_Noise_Cov_Q[0][0];
	
	Pri_Hat->Prior_Hat_Err_Cov[1][1] = Post_Hat->Posterior_Hat_Err_Cov[1][1] + All_Matrix->Process_Noise_Cov_Q[1][1];
	
}

/**
 *	@brief	预测步骤
 */
void Predict_Step( Kalman_Filter *Kal_Filter )
{
	Prior_Hat_Update( Kal_Filter );

}


/**
 *	@brief	卡尔曼增益矩阵更新
 */
void Kalman_Gain_Update( Kalman_Filter *Kal_Filter )
{
	Prior_Hat 			*Pri_Hat	 	= &( Kal_Filter->Pri_Hat );
	Matrix					*All_Matrix = &( Kal_Filter->All_Matrix );
	
	All_Matrix->Kalman_Gain[0][0] = ( Pri_Hat->Prior_Hat_Err_Cov[0][0] + All_Matrix->Measure_Noise_Cov_R[0][0] )
																	* Pri_Hat->Prior_Hat_Err_Cov[0][0];
	
	All_Matrix->Kalman_Gain[1][1] = ( Pri_Hat->Prior_Hat_Err_Cov[1][1] + All_Matrix->Measure_Noise_Cov_R[1][1] )
																	* Pri_Hat->Prior_Hat_Err_Cov[1][1];
	
}
/**
 *	@brief	后验估计值和它的协方差更新
 */
void Posterior_Hat_Update( Kalman_Filter *Kal_Filter )
{
	Prior_Hat 			*Pri_Hat	 	= &( Kal_Filter->Pri_Hat );
	Measure_Data    *Meas_Data  = &( Kal_Filter->Meas_Data );
	Posterior_Hat   *Post_Hat  	= &( Kal_Filter->Post_Hat );
	Matrix					*All_Matrix = &( Kal_Filter->All_Matrix );
	
//注意，这一次的后验估计值和后验误差的协方差矩阵都要用到这一次的先验估计值和先验误差的协方差矩阵
	
	//后验估计值gyro_x
	Post_Hat->gyro_x = Pri_Hat->gyro_x + 
										 All_Matrix->Kalman_Gain[0][0] * ( Meas_Data->gyro_x - Pri_Hat->gyro_x );
	
	//后验估计值gyro_y
	Post_Hat->gyro_y = Pri_Hat->gyro_y + 
										 All_Matrix->Kalman_Gain[1][1] * ( Meas_Data->gyro_y - Pri_Hat->gyro_y );
	
	//后验估计误差的协方差矩阵
	Post_Hat->Posterior_Hat_Err_Cov[0][0] = ( 1 - All_Matrix->Kalman_Gain[0][0] ) * Pri_Hat->Prior_Hat_Err_Cov[0][0];
				
	Post_Hat->Posterior_Hat_Err_Cov[1][1] = ( 1 - All_Matrix->Kalman_Gain[1][1] ) * Pri_Hat->Prior_Hat_Err_Cov[1][1];
		
}

/**
 *	@brief	校正步骤
 */
void Correct_Step( Kalman_Filter *Kal_Filter )
{
	Kalman_Gain_Update( Kal_Filter );
	
	Posterior_Hat_Update( Kal_Filter );
}

/**
 *	@brief	把每一次的后验估计保存到这里
 */
void KalPass_Data_Update( Kalman_Filter *Kal_Filter )
{
	Kal_Filter->KalPass_Data.gyro_x = Kal_Filter->Post_Hat.gyro_x;
	
	Kal_Filter->KalPass_Data.gyro_y = Kal_Filter->Post_Hat.gyro_y;
}





//-----------------------------卡尔曼滤波总过程开始----------------------------------//
void Start_Kalman_Filter(void)
{
	//第一步，设置初始条件
	if( Kalman_Filter_DeInit )
		Kal_Filter.Set_Initial( &Kal_Filter );
	
	//第二步，数据更新
	Kal_Filter.Data_Update( &Kal_Filter );
	
	//第三步，预测
	Kal_Filter.Predict( &Kal_Filter );
	
	//第四步，校正
	Kal_Filter.Correct( &Kal_Filter );
	
	//第五步，保存
	Kal_Filter.KalPass_Data_Update( &Kal_Filter );
	
}


//-----------------------------卡尔曼滤波总过程结束----------------------------------//





