#include "key_sensor.h"


/**
 *	@brief	Z,X,V同时按住程序复位
 */
void Key_Reset(void)
{
	 KEY_State_Judge(&keyboard.Z ,KEY_Z, CHANGE_TIM , LONG_CHANGE_TIM_Z);
	 KEY_State_Judge(&keyboard.X ,KEY_X, CHANGE_TIM , LONG_CHANGE_TIM_X);
	 KEY_State_Judge(&keyboard.V ,KEY_V, CHANGE_TIM , LONG_CHANGE_TIM_V);
	
	 if( keyboard.Z.this_State == DOWN && 
			 keyboard.X.this_State == DOWN && 
			 keyboard.V.this_State == DOWN )
	 {
//			__set_FAULTMASK(1); 
//			NVIC_SystemReset();
	 }
}

