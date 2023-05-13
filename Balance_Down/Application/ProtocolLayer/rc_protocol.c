/**
 * @file        rc_protocol.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       DT7&DR16 Rc Protocol.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(24-October-2021)
 *                  1.修改rc_potocol.c/.h->rc_protocol.c/.h 
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rc_protocol.h"


/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void rc_sensor_init(rc_sensor_t *rc_sen)
{
	// 初始化为离线状态
	rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
	rc_sen->work_state = DEV_OFFLINE;
	
	if(rc_sen->id == DEV_ID_RC)
		rc_sen->errno = NONE_ERR;
	else
		rc_sen->errno = DEV_ID_ERR;
}

/**
 *	@brief	遥控器数据解析协议，原始数据
 */
void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf)
{
	rc_sensor_info_t *rc_info = rc_sen->info;
	rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF;
	rc_info->ch0 -= 1024;
	rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF;
	rc_info->ch1 -= 1024;
	rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
	rc_info->ch2 -= 1024;
	rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF;
	rc_info->ch3 -= 1024;

	rc_info->s1 = ((rxBuf[5] >> 4) & 0x000C) >> 2;
	rc_info->s2 = (rxBuf[5] >> 4) & 0x0003;	
	
	rc_info->mouse_t.mouse_vx = rxBuf[6]  | (rxBuf[7 ] << 8);
	rc_info->mouse_t.mouse_vy = rxBuf[8]  | (rxBuf[9 ] << 8);
	rc_info->mouse_t.mouse_vz = rxBuf[10] | (rxBuf[11] << 8);
	rc_info->mouse_t.mouse_press_l = rxBuf[12];
	rc_info->mouse_t.mouse_press_r = rxBuf[13];
	
	rc_info->key_u.key_code_val= rxBuf[14] | (rxBuf[15] << 8);	
	
	rc_info->thumbwheel = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	rc_info->thumbwheel -= 1024;
		
	Key_Channel_Update();	
	Mouse_FS();
		
	//第一次收到的通道值数据如果都在0附近，打开通道使能标志位
	if(RC_IsChannelReset())	 
		Link_RC_CH_Is_Reset_Handler(OPEN);
	
	
	//解决由于第一次通道不正常的情况
	if(Link_RC_Judge_Is_Wise_Enable_Handler() == OPEN)
		RC_ResetData(rc_sen);
	
	rc_info->offline_cnt = 0;
}

/**
 *	@brief	在串口2中解析遥控数据协议，上主控通过串口1发送给底盘主控
 */
void USART2_rxDataHandler(uint8_t *rxBuf)
{
	// 更新遥控数据
	#ifdef LINK_TO_RECEIVE
	{
		rc_sensor.update(&rc_sensor, rxBuf);
		rc_sensor.check(&rc_sensor);
		
		if(IF_RC_SW1_UP && IF_RC_SW2_DOWN)
		{
			__set_FAULTMASK(1); 
			NVIC_SystemReset();
		}
	}
	#endif
}
