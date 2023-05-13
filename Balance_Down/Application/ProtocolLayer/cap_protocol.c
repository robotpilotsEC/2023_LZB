#include "cap_protocol.h"

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min);
float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min);


extern Super_t Super_2023;


void CAP_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(rxBuf == NULL)
		return;

	if(canId == CAP_RX_ID)
	{
		Super_2023.RX.cap_Ucr = int16_to_float(((uint16_t)rxBuf[0] << 8| rxBuf[1]), 32000, -32000, 30, 0);
		Super_2023.RX.cap_I = int16_to_float(((uint16_t)rxBuf[2] << 8| rxBuf[3]), 32000, -32000, 20, -20);
		Super_2023.RX.cap_state.state = ((uint16_t)rxBuf[4] << 8| rxBuf[5]);    
		Super_2023.offline_cnt = 0;			
	}
}

int16_t float_to_int16(float a, float a_max, float a_min, int16_t b_max, int16_t b_min)
{
	int16_t b = (a - a_min) / (a_max - a_min) * (float)(b_max - b_min) + (float)b_min + 0.5f;   //加0.5使向下取整变成四舍五入
	return b;
}

float int16_to_float(int16_t a, int16_t a_max, int16_t a_min, float b_max, float b_min)
{
	float b = (float)(a - a_min) / (float)(a_max - a_min) * (b_max - b_min) + b_min;
	return b;
}
