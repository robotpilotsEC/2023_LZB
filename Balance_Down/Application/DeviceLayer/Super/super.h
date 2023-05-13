#ifndef __SUPER_H
#define __SUPER_H
#include "rp_config.h"
#include "algo.h"
#include "driver.h"
#include "judge_sensor.h"


/***************************************************************************/

#define  CAP_RX_ID         (0x30)
#define  CAP_OUTPUT_LIMIT  150
#define  CAP_INPUT_LIMIT   150
#define  CAP_MAX_VOLTAGE   (24.5f)


typedef struct
{
	//分0x2E和0x2F两个数据包发送
	uint16_t chassis_power_buffer;  //底盘功率缓冲，0~60J
	uint16_t chassis_power_limit;   //机器人底盘功率限制上限，0~120W
	int16_t  output_power_limit;     //电容放电功率限制，-120~300W
	uint16_t input_power_limit;     //电容充电功率限制，0~150W
	uint16_t chassis_volt;			//底盘输出电压 单位 毫伏 **
	uint16_t chassis_current;		//底盘输出电流 单位 毫安 **

	union{
				uint16_t all;
				struct
				{
						uint16_t cap_switch : 1;    //电容开关
						uint16_t cap_record : 1;    //记录功能开关
						uint16_t gamegoing : 1;     //比赛进行中为1，否则为0 ***						
				}bit;
		}cap_control;
		
}cap_send_data_t;

typedef struct
{
	float cap_Ucr;    //电容两端电压Ucr，0~30V
	float cap_I;    //电容电流I，-20~20A
	union
	{
			uint16_t state;     //电容状态
			struct
			{
					uint16_t none : 1;   //无
					uint16_t cap_U_over : 1;    //电容过压
					uint16_t cap_I_over : 1;    //电容过流
					uint16_t cap_U_low : 1;     //电容欠压
					uint16_t bat_I_low : 1;     //裁判系统欠压
					uint16_t can_receive_miss : 1;    //电容未接收到CAN通信数据
			}bit;
	}cap_state;
}cap_receive_data_t;

typedef struct CAP_RP_struct 
{
  cap_send_data_t      TX;
  cap_receive_data_t   RX;
	
  uint16_t		       offline_cnt;
  uint16_t		       offline_max_cnt;	
  dev_work_state_e	 work_state; 


  void    (*ctrl)(void);	
  void		(*heart_beat)(struct CAP_RP_struct *self);	

} Super_t;


void set_message(void);
void cap_send_0x2E(char can);
void cap_send_0x2F(char can);



extern Super_t Super_2023;


#endif


