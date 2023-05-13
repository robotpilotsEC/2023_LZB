#include "LED.h"

void LED_Task(void)
{
	//imu��������ɫ
	if(devices.state.bit.imu == Normal)
	{
		LED(Red_Pin, ON);
	}
	else
	{
		LED(Red_Pin, OFF);
	}
	
	//ͨѶ��������ɫ
	if(devices.state.bit.comm == Normal)
	{
		LED(Green_Pin, ON);
	}
	else
	{
		LED(Green_Pin, OFF);
	}
	
	//2��9025��Ϣ����ɫ
	if(devices.state.bit.kt_l == Normal && devices.state.bit.kt_r == Normal)//������������
	{
		LED(Blue_Pin, ON);
	}
	else 
	{
		LED(Blue_Pin, OFF);
	}
	
	//����+����+��࣬��ɫ
	if(devices.state.bit.judge == Normal && devices.state.bit.super == Normal)
		LED(Orange_Pin, ON);
	else
		LED(Orange_Pin, OFF);
}



