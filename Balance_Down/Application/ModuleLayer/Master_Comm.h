#ifndef __Master_Comm_H
#define __Master_Comm_H



#include "device.h"
#include "State.h"


#define MASTER_COMM_FRAME_HEADER  (0xA5)
#define LEN_FRAME_HEAD   				  3
#define LEN_FRAME_END		 					2


#define MASTER_RX_PACK_LENGTH     sizeof(rx_pack_t)
#define MASTER_TX_PACK_LENGTH     sizeof(tx_pack_t)





typedef enum
{
	UART1_COMM,
	UART2_COMM,
	UART3_COMM,
	UART4_COMM,
	UART5_COMM,
	
}Comm_Way_e;


typedef enum Master_Rx_Raw_Data_BufPos
{
//帧头在数组的位置	
	RX_SOF      = 0,
	RX_DATA_LEN = 1,
	RX_CRC8     = 2,
	
	
//帧尾在数组的位置	
	RX_CRC16_BEGIN    = 9,
	RX_CRC16_END      = 10,
	
}Master_Rx_Raw_Data_BufPos_e;



typedef __packed struct rx_pack
{
	//帧头
	__packed struct 
	{
		uint8_t  SOF;
		uint8_t  data_length;
		uint8_t  crc8;
	}std_frame_rx_header_t;
	
	//数据段,如果要增加数据，只需在这里修改，关于标志位设置在RC_Flag_UP_Version2增加
	__packed struct
	{
		int16_t   				rc_ch_ws_val;
		int16_t           rc_ch_ad_val;
		
		//所有标志位从低到高设置,这里不用共用体
		uint16_t  					status;		
		
		float             yaw_angle;
		int16_t 					motor_angle;
		
	}std_frame_rx_data_t;
	
	//帧尾
	__packed struct
	{
		uint16_t crc16;
		
	}std_frame_rx_end_t;
	
}rx_pack_t;

typedef __packed struct tx_pack
{
	//帧头
	__packed struct 
	{
		uint8_t  SOF;
		uint8_t  data_length;
		uint8_t  crc8;
	}std_frame_tx_header_t;
	
	//数据段，要增加数据的话,在Master_Tx_Pack_W函数增加
	__packed struct
	{
		uint16_t shooter_cooling_limit;
		
		uint16_t shooter_speed_limit;
		
		uint16_t shooter_cooling_heat;
		
		float bullet_speed;//裁判系统测得的弹速
		
		uint8_t  robot_id; //红0蓝1
		
	}std_frame_tx_data_t;
	
	//帧尾
	__packed struct
	{
		uint16_t crc16;
		
	}std_frame_tx_end_t;
	
}tx_pack_t;






typedef struct Master_Comm_Info
{
	
	Comm_Way_e   comm_way;
	

	rx_pack_t		 rx_pack;
	tx_pack_t    tx_pack;
	uint8_t      tx_buf[MASTER_TX_PACK_LENGTH];
	
	
	dev_work_state_e	work_state;
	int16_t		 offline_cnt;
	int16_t		 offline_max_cnt;
	
	void       (*ctrl)(void);
	void       (*rx)(uint8_t *rxBuf);
	void			 (*heart_beat)(struct Master_Comm_Info *self);
	
}Master_Comm_Info_t;

__weak void Comm_Judge_Is_Data_Err_Handler(char flag);
__weak void Comm_RC_CH_Not_True_Handler(void);
__weak void Comm_UP_RC_Flag_Handler(const uint16_t status);
__weak void Comm_UP_Chassic_Flag_Handler(const uint16_t status);
__weak void Comm_RC_Offline_Handler(void);
__weak void Comm_Fail_Handler(void);


extern Master_Comm_Info_t  Master;

#endif
