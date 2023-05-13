#ifndef __LED_H
#define __LED_H

#include "offline.h"
#include "main.h"

#define LED(port, state)  HAL_GPIO_WritePin(GPIOC, port, (GPIO_PinState)state)   

typedef enum
{
	ON, //相当于GPIO_PinState的低电平
	OFF,
	
}LED_State_e;

	
//PC10---红色 Red_Pin
//PC11---绿色 Green_Pin
//PC13---蓝色 Blue_Pin
//PC14---橙色 Orange_Pin

/*
1）PC10:imu正常则常亮，否则熄灭
2）PC11:通讯正常则常亮，否则熄灭
3）PC13:2个电机，KT0为1，KT1为2，所以都正常则显示3，否则要么是1要么是2
4）PC14:3个设备裁判+超电+测距
*/

void LED_Task(void);

#endif
