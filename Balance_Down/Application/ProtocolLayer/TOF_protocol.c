#include "TOF_protocol.h"

uint8_t rx[16];

void USART3_rxDataHandler(uint8_t *rxBuf)
{
	if(rxBuf == NULL)
		return;

	memcpy(rx, rxBuf, 16);
	uint8_t device_id = rxBuf[TOF_Rx_DATA_ID_POS];
	
	switch(device_id)
	{
		case POS_L:
			TOF_Sensor[POS_L].rx(&TOF_Sensor[POS_L], rxBuf);
		break;
		
		case POS_R:
			TOF_Sensor[POS_R].rx(&TOF_Sensor[POS_R], rxBuf);
		break;
			
	}
	
}

