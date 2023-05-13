#include "LED.h"

void LED_Task(void)
{
	//imu正常，红色
	if(devices.state.bit.imu == Normal)
	{
		LED(Red_Pin, ON);
	}
	else
	{
		LED(Red_Pin, OFF);
	}
	
	//通讯正常，绿色
	if(devices.state.bit.comm == Normal)
	{
		LED(Green_Pin, ON);
	}
	else
	{
		LED(Green_Pin, OFF);
	}
	
	//2个9025信息，蓝色
	if(devices.state.bit.kt_l == Normal && devices.state.bit.kt_r == Normal)//都不正常不闪
	{
		LED(Blue_Pin, ON);
	}
	else 
	{
		LED(Blue_Pin, OFF);
	}
	
	//裁判+超电+测距，橙色
	if(devices.state.bit.judge == Normal && devices.state.bit.super == Normal)
		LED(Orange_Pin, ON);
	else
		LED(Orange_Pin, OFF);
}



