#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "rp_driver_config.h"


/*
	CAN的接收结构体
*/
typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_RxFrameTypeDef;


void CAN_Filter_Init(void);
HAL_StatusTypeDef CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat);
void CAN1_Send_With_int16_to_uint8(uint32_t stdId, int16_t *dat);
void CAN2_Send_With_int16_to_uint8(uint32_t stdId, int16_t *dat);

#endif
