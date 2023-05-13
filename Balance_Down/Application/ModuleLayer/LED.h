#ifndef __LED_H
#define __LED_H

#include "offline.h"
#include "main.h"

#define LED(port, state)  HAL_GPIO_WritePin(GPIOC, port, (GPIO_PinState)state)   

typedef enum
{
	ON, //�൱��GPIO_PinState�ĵ͵�ƽ
	OFF,
	
}LED_State_e;

	
//PC10---��ɫ Red_Pin
//PC11---��ɫ Green_Pin
//PC13---��ɫ Blue_Pin
//PC14---��ɫ Orange_Pin

/*
1��PC10:imu��������������Ϩ��
2��PC11:ͨѶ��������������Ϩ��
3��PC13:2�������KT0Ϊ1��KT1Ϊ2�����Զ���������ʾ3������Ҫô��1Ҫô��2
4��PC14:3���豸����+����+���
*/

void LED_Task(void);

#endif
