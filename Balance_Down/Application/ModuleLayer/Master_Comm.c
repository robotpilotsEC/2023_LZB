#include "Master_Comm.h"

//----------------------------------上层函数
void Master_Ctrl(void);
static void Master_Comm_Heart_Beat(Master_Comm_Info_t *self);
void Master_Rx_Info_Buff(uint8_t *rxBuf); //在串口1中断回调被调用

//-----------------------------------应用
void Comm_UP_RC_Flag_Handler(const uint16_t status);
void Comm_UP_Chassic_Flag_Handler(const uint16_t status);


//------------------------------------基础函数
void Master_Info_UP(void);
void Master_Tx_Info_Buff(void);
void Not_Link_RC_CH_UP(void); 
void Not_Link_RC_Flag_UP(void);



//------------------------------------辅助函数
void Master_Rx_Info_UP(uint8_t *rxbuf);
void Master_Tx_Info_UP(void);




//* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;


Master_Comm_Info_t  Master = 
{
	.comm_way = UART1_COMM,
	
	.offline_max_cnt = 100,
	
	
	.ctrl     	= Master_Ctrl,
	.rx         = Master_Rx_Info_Buff,
	.heart_beat = Master_Comm_Heart_Beat,
};






void Master_Ctrl(void)
{
	
	Master_Info_UP();
	
	
}

//更新要发送的原始数据,更新遥控器通道,更新上主控发来的标志位
void Master_Info_UP(void)
{
	Master_Tx_Info_Buff();
	
	//如果下主控接了遥控器,那么上主控发来的遥控器通道必须关闭更新
	#ifdef NO_LINK_TO_RECEIVE
	{
		Not_Link_RC_CH_UP();
	}
	#endif
	

	Not_Link_RC_Flag_UP();
	
}


//---------------------------------------------通讯收发过程

//接收原始数据,写到结构体中
void Master_Rx_Info_Buff(uint8_t *rxBuf)
{
	if( rxBuf == NULL )
		return;
	
	Master_Rx_Info_UP(rxBuf);
	
}

//发送处理
void Master_Tx_Info_Buff(void)
{
	Master_Tx_Info_UP();
	
	if( __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) )
	{
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
		HAL_UART_Transmit_DMA(&huart1, Master.tx_buf, MASTER_TX_PACK_LENGTH);
	}
		
}

//校验+写入到接收结构体中
void Master_Rx_Info_UP(uint8_t *rxBuf)
{
	uint8_t  frame_length_sum = 0;							 //原始数据包的总长度
	uint8_t  frame_head_length = LEN_FRAME_HEAD; //原始数据包的帧头长度
	uint8_t  data_length; 
	uint8_t  frame_end_length  = LEN_FRAME_END;  //原始数据包的帧尾长度
		
	/* SOF检验 */
	if(rxBuf[RX_SOF] == MASTER_COMM_FRAME_HEADER) 
	{
		//原始数据包的数据段长度
		data_length	= rxBuf[RX_DATA_LEN];

		/* 帧头CRC8校验 */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* 统计一帧的总数据长度，用于CRC16校验 */    
			frame_length_sum = frame_head_length + data_length + frame_end_length;
			
			//CRC16校验
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length_sum) == true)	
			{
				//保存到接收结构体中,frame_length_sum相当于整个数据包的字节个数了
				memcpy(&Master.rx_pack, rxBuf, frame_length_sum );
				
				//清除失联计数值
				Master.offline_cnt = 0;		

				State.comm_flag.state_flag = COMM_OK;
			}
			//CRC16错误
			else	
				State.comm_flag.state_flag = CRC16_ERR;	
		}		
		
		//CRC8错误
		else
			State.comm_flag.state_flag = CRC8_ERR;
		
	}
	//帧头错误
	else
		State.comm_flag.state_flag = FRAME_HEADER_ERR;

	
	
}

//数据包的写入到结构体,再写到数组中
void Master_Tx_Info_UP(void)
{
	uint8_t robot_id = judge_info.game_robot_status.robot_id;
	
	//----------------------------帧头数据-------------------------------------
	Master.tx_pack.std_frame_tx_header_t.SOF = MASTER_COMM_FRAME_HEADER;
	Master.tx_pack.std_frame_tx_header_t.data_length = MASTER_TX_PACK_LENGTH;

	
	
	//-----------------------------数据段-----------------------------------------
	if(JUDGE_ONLINE)
	{
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_limit = judge_info.game_robot_status.shooter_id1_17mm_cooling_limit;
		Master.tx_pack.std_frame_tx_data_t.shooter_speed_limit = judge_info.game_robot_status.shooter_id1_17mm_speed_limit;
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_heat = judge_info.power_heat_data.shooter_id1_17mm_cooling_heat;
		Master.tx_pack.std_frame_tx_data_t.bullet_speed = judge_info.shoot_data.bullet_speed;
	}
	else
	{
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_limit = 120;
		Master.tx_pack.std_frame_tx_data_t.shooter_speed_limit = 0;
		Master.tx_pack.std_frame_tx_data_t.shooter_cooling_heat = 0;
		Master.tx_pack.std_frame_tx_data_t.bullet_speed = -1.f;
	}
	
	if(robot_id >=3 && robot_id <= 5) //红色步兵3,4,5
		Master.tx_pack.std_frame_tx_data_t.robot_id = 0;
	else if(robot_id >=103 && robot_id <=105) //蓝色步兵3,4,5
		Master.tx_pack.std_frame_tx_data_t.robot_id = 1;
	else //如果不符合,默认为红色
		Master.tx_pack.std_frame_tx_data_t.robot_id = 0;
	
	//----------------------------CRC数据--------------------------------------
			//为了计算CRC
	memcpy(Master.tx_buf, &Master.tx_pack, MASTER_TX_PACK_LENGTH );
	
			//CRC8用帧头
	Append_CRC8_Check_Sum(Master.tx_buf, LEN_FRAME_HEAD);
																															  
			//CRC16用总长度
	Append_CRC16_Check_Sum(Master.tx_buf, MASTER_TX_PACK_LENGTH);	
																														    																									  
	memcpy(&Master.tx_pack, Master.tx_buf, MASTER_TX_PACK_LENGTH);
}

//---------------------------------------------对收到的数据进行细化处理




//根据板间通讯的数据来更新State里的遥控器,底盘标志位
void Not_Link_RC_Flag_UP(void)
{
	
	Comm_UP_RC_Flag_Handler(Master.rx_pack.std_frame_rx_data_t.status);
	
	Comm_UP_Chassic_Flag_Handler(Master.rx_pack.std_frame_rx_data_t.status);		
}
//根据板间通讯的数据来更新RC_sensor里的通道值
void Not_Link_RC_CH_UP(void)
{
	//RC_LEFT_CH_LR_VALUE
	rc_sensor_info.ch3 = Master.rx_pack.std_frame_rx_data_t.rc_ch_ws_val;
	
	//RC_LEFT_CH_UD_VALUE
	rc_sensor_info.ch2 = Master.rx_pack.std_frame_rx_data_t.rc_ch_ad_val;
	
	//遥控器通道值不正常
	if( abs_(rc_sensor_info.ch2) <= CH_MAX && abs_(rc_sensor_info.ch3) <= CH_MAX )
	{
		State.rc_flag.ch_wise = OPEN;
	}
	else
	{
		State.rc_flag.ch_wise = CLOSE;
		rc_sensor_info.ch2 = rc_sensor_info.ch3 = 0;
	}

}








//-----------------------------------------------检测函数


/**
 *	@brief	通讯心跳包，检查板间通讯是否成功
 */
static void Master_Comm_Heart_Beat(struct Master_Comm_Info *self)
{
	self->offline_cnt ++;
	if(self->offline_cnt > self->offline_max_cnt) {
		self->offline_cnt = self->offline_max_cnt + 1;
		
		self->work_state = DEV_OFFLINE;
		
		State.comm_flag.state_flag = COMM_FAIL;
		
		
	} 
	else {
		/* 离线->在线 */
		if(self->work_state == DEV_OFFLINE)
		{
			self->work_state = DEV_ONLINE;
		}
	}
	
	
}

//------------------------------------------------设置State里相关标志位






//更新遥控器的标志位
void Comm_UP_RC_Flag_Handler(const uint16_t status)
{
	//0关1开低1位
	State.rc_flag.work_state = (dev_work_state_e)( (status >> RC_WORK_STATE_BIT) & 0x01 );
	
	//0关1开低2位
	State.rc_flag.magz_state = (switch_e)( (status >> MAGZ_FLAG_BIT) & 0x01 );
	
}

//更新底盘的运动标志位
void Comm_UP_Chassic_Flag_Handler(const uint16_t status)
{
	//0关1开低3位
	if( (status >> SMALL_TOP_FLAG_BIT) & 0x01 )
		State.chassic_flag.chas_motion_mode= SMALL_TOP_ON;
	else
		State.chassic_flag.chas_motion_mode = SMALL_TOP_OFF;

	if((status >> GIMBAL_HEADER_LOW_BIT)&0x01)
		State.chassic_flag.chas_ws_dir = ALINE_TO_B;
	else
		State.chassic_flag.chas_ws_dir = ALINE_TO_F;	

	//0头锁住1正在调头低5位
	if( ((status >> GIMBAL_TURN_BIT) & 0x01) )
		State.chassic_flag.gimb_motion_mode = GIMB_TURING_AROUND;
	else
		State.chassic_flag.gimb_motion_mode = GIMB_TURING_OK;

	//0关1开自瞄
	if( ((status >> GIMBAL_AUTO_BIT) & 0x01) )
		State.chassic_flag.gimb_auto_mode = OPEN;
	else
		State.chassic_flag.gimb_auto_mode = CLOSE;

	//0关1复位系统
	if( ((status >> SYS_RESET_BIT) & 0x01) )
		State.chassic_flag.sys_reset_switch = OPEN;
	else
		State.chassic_flag.sys_reset_switch = CLOSE;
	
	//0关1开侧身
	if( ((status >> SIDEWAYS_BIT) & 0x01) )
		State.chassic_flag.sideways_state = SIDEWAYS_ON;
	else
		State.chassic_flag.sideways_state = SIDEWAYS_OFF;
	
	//0关1开加速
	if( ((status >> SPEED_UP_BIT) & 0x01) )
		State.chassic_flag.speed_up_switch = OPEN;
	else
		State.chassic_flag.speed_up_switch = CLOSE;
	
	//0关1开打符
	if( ((status >> DAFU_MODE_BIT) & 0x01) )
		State.chassic_flag.dafu_switch = OPEN;
	else
		State.chassic_flag.dafu_switch = CLOSE;
	
	//0正常1不正常
	if((status >> GIMBAL_IMU_STATE_BIT)&0x01)
		State.chassic_flag.gimb_imu_state = UNNORMAL;
	else
		State.chassic_flag.gimb_imu_state = NORMAL;
	
	//0左1右	
	if((status >> GIMBAL_HEADER_HIGH_BIT)&0x01)
		State.chassic_flag.chas_ad_dir = ALINE_TO_R;
	else
		State.chassic_flag.chas_ad_dir = ALINE_TO_L;	
}


