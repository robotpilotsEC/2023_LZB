#include "can_protocol.h"


extern RM_motor_t MOM_motor[MOM_MOTOR_LIST];
extern KT_motor_t KT_motor[KT_MOTOR_LIST];

__weak void CAP_rxDataHandler(uint32_t canId, uint8_t *rxBuf);

void CAN1_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	switch(canId)
	{
		case CHASSIS_CAN_ID_LEG_L:
			KT_motor[LEG_L].get_info( &KT_motor[LEG_L], rxBuf );
			break;
		
		case CHASSIS_CAN_ID_LEG_R:
			KT_motor[LEG_R].get_info( &KT_motor[LEG_R], rxBuf );
			break;
		
		default:
			break;
	}

}


void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	switch(canId)
	{
		case MOMENTUM_CAN_ID_L:
			MOM_motor[MOMENTUM_L].rx( &MOM_motor[MOMENTUM_L], rxBuf );
			MOM_motor[MOMENTUM_L].ctr_offset( &MOM_motor[MOMENTUM_L], RM_TOTAL_ANGLE);
			break;
		
		case MOMENTUM_CAN_ID_R:
			MOM_motor[MOMENTUM_R].rx( &MOM_motor[MOMENTUM_R], rxBuf );
			MOM_motor[MOMENTUM_R].ctr_offset( &MOM_motor[MOMENTUM_R], RM_TOTAL_ANGLE);
			break;
		
		case CAP_CAN_RX_ID:
			CAP_rxDataHandler( CAP_CAN_RX_ID, rxBuf );
			break;
		
		default:
			break;
	}

}

__weak void CAP_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	if(rxBuf == NULL)
		return;
}
