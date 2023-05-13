#ifndef __TOF_sensor_H
#define __TOF_sensor_H

#include "driver.h"

#define  TOF_Rx_DATA_ID_POS			 3  //在接收数据帧中的数组位置
#define  TOF_RX_FRAME_HEADER		(0x57)
#define  TOF_RX_BUFFER_LEN  		 16
#define  TOF_TX_BUFFER_LEN  		 8



//左0右1
typedef enum
{
	POS_L,
	POS_R,
	
	TOF_TOTOL,
	
}tof_idnex_e;



typedef enum
{
	UART1_,
	UART2_,
	UART3_,
	UART4_,
	UART5_,
	
}drv_way_e;

typedef enum
{
	COMM_SUCCESS,
	
	COMM_FAILED,
	
}comm_status_e;

typedef enum
{
	DATA_TRUE,
	
	DATA_ERR,
	
}data_status_e;


typedef __packed struct
{
	uint8_t 	Frame_Header; //帧头
	
	uint8_t   Function_Mark; //功能字
	
	uint8_t   reserved_f;
	uint8_t   id;
	uint32_t  System_time;
	uint8_t   dis_low_byte;
	uint8_t   dis_min_byte;
	uint8_t   dis_high_byte;
	uint8_t   dis_status;
	uint16_t  signal_strength;
	uint8_t   reserved_b;
	
	uint8_t   Sum_Check;//检查字
	
}TOF_sensor_raw_rx_info_t;

typedef __packed struct
{
	uint8_t 	Frame_Header; //帧头
	
	uint8_t   Function_Mark; //功能字
	
	uint16_t   reserved_f;
	uint8_t   id;
	uint16_t   reserved_b;
	
	uint8_t   Sum_Check;//检查字
	
}TOF_sensor_raw_tx_info_t;

typedef struct
{
	drv_way_e      drive_way;
	comm_status_e  comm_status;
	data_status_e  data_status;
	
	float          raw_dis_meas;//原始测量距离,cm单位      
	
	uint8_t        tx_id;
	uint8_t        rx_id;
	
}TOF_sensor_info_t;


typedef struct TOF_sensor
{	
	uint16_t  				offline_cnt;
	uint16_t  				offline_max_cnt;
	dev_work_state_e  work_state;
	
	TOF_sensor_raw_rx_info_t  raw_rx_info;
	TOF_sensor_raw_tx_info_t  raw_tx_info;
	TOF_sensor_info_t  		 		info;
	

	
	void	(*rx)(struct TOF_sensor* tof, uint8_t* rxBuf);
	void  (*tx)(struct TOF_sensor* tof);
	void  (*heart_beat)(struct TOF_sensor* tof);
	
}TOF_sensor_t;

extern TOF_sensor_t  TOF_Sensor[TOF_TOTOL];


void ALL_TOF_Sensor_HeartBeat(void);

#endif
