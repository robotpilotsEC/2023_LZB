#ifndef __offline_H
#define __offline_H

#include "module.h"

void Offline_Handler(void);

typedef enum
{
	Normal,
	
	UnNormal,
	
}state;

typedef struct
{

	union
	{
		uint16_t  val;
		struct
		{
			uint16_t  imu : 1;
			uint16_t  kt_l : 1;
			uint16_t  kt_r : 1;
			uint16_t  rm_l : 1;
			uint16_t  rm_r : 1;
			uint16_t  super : 1;
			uint16_t  judge : 1;
			uint16_t  tof : 1;
			uint16_t  comm : 1;
		}bit;
		
	}state;
	
}Device_State;

extern Device_State devices;

#endif
