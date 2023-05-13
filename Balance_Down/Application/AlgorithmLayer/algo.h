#ifndef __ALGO_H
#define __ALGO_H


#include "stm32f4xx.h"                  // Device header
#include "stdio.h"
#include "string.h"
#include "rp_math.h"
#include "ave_filter.h"
#include "kalman_filter.h"
#include "crc.h"
#include "power_limit.h"

#define VALUE_PI  	3.1415926f   //Π


#define RADIAN_CONVERSION_ANGLE (180/VALUE_PI)  //弧度转角度，180/3.14
#define ANGLE_CONVERSION_RADIAN (VALUE_PI/180) 	//角度转弧度，3.14/180


#endif
