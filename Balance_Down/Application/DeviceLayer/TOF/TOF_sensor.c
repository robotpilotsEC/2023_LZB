#include "TOF_sensor.h"

void TOF_Rx_Info(TOF_sensor_t* tof, uint8_t* rxBuf);
void TOF_Tx_Info(TOF_sensor_t* tof);
void ALL_TOF_Sensor_HeartBeat(void);


static void TOF_sensor_heart_beat(TOF_sensor_t *self);
void TOF_Rx_Raw_Info(TOF_sensor_t* tof, uint8_t* rxBuf);
void TOF_Rx_Info_UP(TOF_sensor_t* tof);
void TOF_Tx_Info_UP(TOF_sensor_t* tof);




extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;

TOF_sensor_t  TOF_Sensor[TOF_TOTOL] = 
{
	[POS_L] = {
		.info.drive_way = UART3_,
		.info.tx_id 		= POS_L,
		.info.rx_id 		= POS_L,
		
		.rx             = TOF_Rx_Info,
		.tx             = TOF_Tx_Info,
		.heart_beat     = TOF_sensor_heart_beat,
		
		.offline_max_cnt = 500,
	},
	
	
	
	[POS_R] = {
		.info.drive_way = UART3_,
		.info.tx_id 		= POS_R,
		.info.rx_id 		= POS_R,
		
		.rx             = TOF_Rx_Info,
		.tx             = TOF_Tx_Info,\
		.heart_beat     = TOF_sensor_heart_beat,
		
		.offline_max_cnt = 500,
	},
};



void ALL_TOF_Sensor_HeartBeat(void)
{
	//TOF_Sensor[POS_L].tx(&TOF_Sensor[POS_L]);
	
	TOF_Sensor[POS_L].heart_beat(&TOF_Sensor[POS_L]);
	
	//TOF_Sensor[POS_R].tx(&TOF_Sensor[POS_R]);
	
	TOF_Sensor[POS_R].heart_beat(&TOF_Sensor[POS_R]);
}



void TOF_Rx_Info(TOF_sensor_t* tof, uint8_t* rxBuf)
{
	if(tof == NULL || rxBuf == NULL)
		return;
	
	TOF_Rx_Raw_Info(tof, rxBuf);
	
	TOF_Rx_Info_UP(tof);
}


uint8_t tof_F_tx_buf[TOF_TX_BUFFER_LEN];
uint8_t tof_B_tx_buf[TOF_TX_BUFFER_LEN];
uint8_t uart_thread;
void TOF_Tx_Info(TOF_sensor_t* tof)
{
	if(tof == NULL)
		return;
	
	TOF_Tx_Info_UP(tof);

	if(tof == &TOF_Sensor[POS_L] && uart_thread == 0)
	{
		memcpy(tof_F_tx_buf, &tof->raw_tx_info,TOF_TX_BUFFER_LEN);
		
		if(huart3.gState == HAL_UART_STATE_READY)
		{
			HAL_UART_Transmit_DMA(&huart3, tof_F_tx_buf, TOF_TX_BUFFER_LEN);
			uart_thread = 1;
		}
	}
		
	else if(tof == &TOF_Sensor[POS_R] && uart_thread == 1)
	{
		memcpy(tof_B_tx_buf, &tof->raw_tx_info,TOF_TX_BUFFER_LEN);

		if(huart3.gState == HAL_UART_STATE_READY)
		{
			HAL_UART_Transmit_DMA(&huart3, tof_B_tx_buf, TOF_TX_BUFFER_LEN);
			uart_thread = 0;
		}

	}

}

void TOF_Rx_Raw_Info(TOF_sensor_t* tof, uint8_t* rxBuf)
{
	if(tof == NULL || rxBuf == NULL)
		return;
	
	memcpy(&tof->raw_rx_info, rxBuf, TOF_RX_BUFFER_LEN);
	
}
	
void TOF_Rx_Info_UP(TOF_sensor_t* tof)
{
	if(tof == NULL)
		return;
	
	uint32_t Sum_Check = 0;
	uint8_t  Sum_Check_Res = 0;
	uint8_t* p = NULL;
	static uint8_t header_err_cnt;
	
	//检查帧头,检查功能字,保留字,距离标志位
	if( tof->raw_rx_info.Frame_Header == TOF_RX_FRAME_HEADER )
	{
		
		header_err_cnt = 0;
		
		tof->offline_cnt = 0;
		
		tof->info.comm_status = COMM_SUCCESS;	
		
		p = &tof->raw_rx_info.Frame_Header;
		
		//检查最后的检查字,最低字节
		for(uint8_t i = 0; i < TOF_RX_BUFFER_LEN - 1; i ++)
			Sum_Check += p[i];
		
		Sum_Check_Res = (uint8_t)(Sum_Check & (0x000000ff));
		
		if( Sum_Check_Res == tof->raw_rx_info.Sum_Check )	
		{
			tof->info.data_status = DATA_TRUE;
			tof->info.raw_dis_meas = ( tof->raw_rx_info.dis_low_byte | \
															 ( tof->raw_rx_info.dis_min_byte << 8) | \
															 ( tof->raw_rx_info.dis_high_byte << 16) );	
			
			tof->info.raw_dis_meas /= 10;
		}
		
		else
			tof->info.data_status = DATA_ERR;
		
	}
	
	else
	{
		header_err_cnt++;
		if(header_err_cnt == 5)
		{	
			tof->info.comm_status = COMM_FAILED;
			header_err_cnt = 0;
		}
	}
		
	
}	

void TOF_Tx_Info_UP(TOF_sensor_t* tof)
{
	if(tof == NULL)
		return;

	tof->raw_tx_info.Frame_Header = TOF_RX_FRAME_HEADER;
	tof->raw_tx_info.Function_Mark = 0x10;
	tof->raw_tx_info.reserved_f = tof->raw_tx_info.reserved_b = 0xffff;
	tof->raw_tx_info.id = tof->info.tx_id;
	
	tof->raw_tx_info.Sum_Check = (0x63+tof->info.tx_id);

}

static void TOF_sensor_heart_beat(TOF_sensor_t *self)
{
	self->offline_cnt ++;
	if(self->offline_cnt > self->offline_max_cnt) {
		self->offline_cnt = self->offline_max_cnt;
		self->work_state = DEV_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(self->work_state == DEV_OFFLINE)
		{
			self->work_state = DEV_ONLINE;
		}
	}

}

