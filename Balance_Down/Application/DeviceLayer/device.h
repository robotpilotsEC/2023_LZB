#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "State.h"

#include "imu_sensor.h"
#include "rc_sensor.h"
#include "user_motor.h"
#include "key_sensor.h"
#include "TOF_sensor.h"
#include "judge_sensor.h"
#include "super.h"


/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct 
{
	imu_sensor_t    *imu_sen;
	rc_sensor_t 		*rc_sen;
	//KT_motor_t      *kt_motor;
	//RM_motor_t      *rm_motor;
	
} dev_list_t;

extern dev_list_t dev_list;

/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void);

#endif
