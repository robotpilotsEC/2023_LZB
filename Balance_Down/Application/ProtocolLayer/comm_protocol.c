#include "comm_protocol.h"


//遥控器的数据由上主控发送给下主控，所以串2不用了

void USART1_rxDataHandler(uint8_t *rxBuf)
{	
	if(rxBuf == NULL)
		return;
	
	Master.rx(rxBuf);

}

