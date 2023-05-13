/**
 * @file        launcher.c
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        20-November-2022
 * @brief       Launcher Control Center
 * @update
 *              v1.0(18-November-2022)
 *              v1.1(20-November-2022)
 *                  1.修改速射连发模式，闭角度环->闭速度环
 */

/* Includes ------------------------------------------------------------------*/
#include "launcher.h"

#include "rp_math.h"
#include "rm_protocol.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Launcher_Init(void);
void Launcher_Ctrl(void);
void Launcher_SelfProtect(void);

void Launcher_GetInfo(void);
void Launcher_GetBaseInfo(void);
void Launcher_GetCtrlInfo(void);
void Launcher_GetRcState(void);
void Get_LauncherStatus(void);
void Fric_StatusCheck(void);
void Fric_SpeedInc(void);
void Fric_SpeedDec(void);
void Dial_StatusCheck(void);

void Launcher_MotorCtrl(void);
void Fric_Ctrl(void);
void Dial_Ctrl(void);

void Launcher_SendOut(void);

void Launcher_Stop(void);

/* Exported functions --------------------------------------------------------*/
//临时发送函数
extern void CAN1_Send_With_int16_to_uint8(uint32_t stdId, int16_t *dat);
/* Private variables ---------------------------------------------------------*/
float      left_speed = 0, right_speed = 0;
int16_t    launcher_out[3];

// 发射机构设备
launcher_dev_t		launcher_dev = {
	.user_motor = motor,
	.rc_sensor = &rc_sensor,
};

// 发射机构信息
launcher_info_t 	launcher_info = {
	.remote_mode = RC,
};

launcher_work_info_t  launcher_work_info = {
	.fric_status = WaitCommond_Fric,
	.dial_status = WaitCommond_Dial,
	.launcher_commond = WaitCommond_L,
	.lock_cnt = 0,
	.shoot_cnt = 0,
};

launcher_conf_t   launcher_conf = {
	.fric_speed = Fric_30,
	.dial_speed = -5000.0f,
	.dial_torque_limit = 2000.0f,
	.lock_angle_check = 1.5f,
	.lock_cnt = 50,
	.Back_Angle = 45.0f,
	.Load_Angle = -45.0f,
	.wait_time = 1000,  //发射间隔时间，单位ms
};

launcher_t launcher = {
	.dev = &launcher_dev,
	.info = &launcher_info,
	.work_info = &launcher_work_info,
	.conf = &launcher_conf,
	.init = Launcher_Init,
	.ctrl = Launcher_Ctrl,
	.self_protect = Launcher_SelfProtect,
};

/* Exported variables --------------------------------------------------------*/

/**
  * @brief  发射机构初始化
  * @param  
  * @retval 
  */
void Launcher_Init(void)
{
	if (rc_sensor.work_state == DEV_ONLINE)
	{
		launcher.info->init_s2 = rc_sensor.info->s2;
		launcher.info->last_s2 = rc_sensor.info->s2;
	}
	
	launcher.info->rc_work_state = rc_sensor.work_state;
}



/**
  * @brief  发射机构控制
  * @param  
  * @retval 
  */
void Launcher_Ctrl(void)
{
	Launcher_GetInfo();
	
	Launcher_MotorCtrl();
	
	Launcher_SendOut();
}


/**
  * @brief  读取发射机构相关信息
  * @param  
  * @retval 
  */
void Launcher_GetInfo(void)
{
	Launcher_GetBaseInfo();
	
	Launcher_GetCtrlInfo();
	
	Get_LauncherStatus();
}


/**
  * @brief  读取发射机构电机信息
  * @param  
  * @retval 
  */
void Launcher_GetBaseInfo(void)
{
	launcher.info->measure_left_speed = motor[FRIC_L].rx_info.speed;
	launcher.info->measure_right_speed = motor[FRIC_R].rx_info.speed;
	launcher.info->measure_dial_speed = motor[DIAL].rx_info.speed;
	launcher.info->measure_dial_angle = motor[DIAL].rx_info.angle_sum * M2006_ECD_TO_ANGLE;
	
	left_speed = motor[FRIC_L].rx_info.speed;
	right_speed = motor[FRIC_R].rx_info.speed;
}


/**
  * @brief  读取遥控器控制信息
  * @param  
  * @retval 
  */
void Launcher_GetCtrlInfo(void)
{
	launcher.info->remote_mode = RC;
	
	Launcher_GetRcState();
}


/**
  * @brief  读取遥控器信息
  * @param  
  * @retval 
  */
void Launcher_GetRcState(void)
{
	/**    读取遥控器上电时刻信息    **/
	if ((launcher.info->rc_work_state == DEV_OFFLINE)
		&& rc_sensor.work_state == DEV_ONLINE)
	{
		launcher.info->init_s2 = rc_sensor.info->s2;
		launcher.info->last_s2 = rc_sensor.info->s2;
	}
	
	
	if (rc_sensor.work_state == DEV_ONLINE)
	{
		/**    遥控器上电保护    **/
		if (launcher.info->init_s2 == rc_sensor.info->s2)
		{
			launcher.work_info->launcher_commond = WaitCommond_L;
		}
		else
		{
			launcher.info->init_s2 = 0;
			
			/**    读取遥控器拨杆跳变信息    **/
			if (rc_sensor.info->s1 == RC_SW_UP)
			{
				
				if (rc_sensor.info->s2 == RC_SW_MID)
				{
					launcher.work_info->launcher_commond = Func_Reset;
				}
				else if (rc_sensor.info->s2 == RC_SW_UP)
				{
					launcher.work_info->launcher_commond = Adjust_Speed;
				}
				else if (rc_sensor.info->s2 == RC_SW_DOWN)
				{
					launcher.work_info->launcher_commond = Fric_Toggle;
				}
				
			}
			else if (rc_sensor.info->s1 == RC_SW_MID)
			{
				
				if (rc_sensor.info->s2 == RC_SW_MID)
				{
					launcher.work_info->launcher_commond = Func_Reset;
				}
				else if (rc_sensor.info->s2 == RC_SW_UP)
				{
					launcher.work_info->launcher_commond = Sweep_Shoot;
					if (launcher.info->last_s2 != rc_sensor.info->s2)
					{
						launcher.work_info->dial_status = SpeedKeep_Dial;
						launcher.info->target_dial_speed = launcher.conf->dial_speed;
					}
				}
				else if (rc_sensor.info->s2 == RC_SW_DOWN)
				{
					launcher.work_info->launcher_commond = Func_Reset;
				}
				
			}
			else if (rc_sensor.info->s1 == RC_SW_DOWN)
			{
				
				if (rc_sensor.info->s2 == RC_SW_MID)
				{
					launcher.work_info->launcher_commond = Func_Reset;
				}
				else if (rc_sensor.info->s2 == RC_SW_UP)
				{
					launcher.work_info->launcher_commond = Single_Shoot;
					if (launcher.info->last_s2 != rc_sensor.info->s2)
					{
						launcher.work_info->dial_status = Reload_Dial;
						launcher.info->target_dial_angle = launcher.conf->Load_Angle + launcher.info->measure_dial_angle;
					}
				}
				else if (rc_sensor.info->s2 == RC_SW_DOWN)
				{
					launcher.work_info->launcher_commond = Keep_Shoot;
					if (launcher.info->last_s2 != rc_sensor.info->s2)
					{
						launcher.work_info->dial_status = Reload_Dial;
						launcher.info->target_dial_angle = launcher.conf->Load_Angle + launcher.info->measure_dial_angle;
					}
				}
				
			}
		}
	}
	else 
	{
		/**    遥控器离线信息    **/
		launcher.work_info->launcher_commond = WaitCommond_L;
		launcher.work_info->fric_status = WaitCommond_Fric;
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
	
}



/**
  * @brief  读取发射机构当前状态
  * @param  
  * @retval 
  */
void Get_LauncherStatus(void)
{
	Fric_StatusCheck();
	
	Dial_StatusCheck();
	
	launcher.info->rc_work_state = rc_sensor.work_state;
	launcher.info->last_s2 = rc_sensor.info->s2;
}



/**
  * @brief  摩擦轮开启跳变检测与射速调整
  * @param  
  * @retval 
  */
void Fric_StatusCheck(void)
{
	static int16_t last_thumbwheel = 0;
	if ((launcher.work_info->launcher_commond == Fric_Toggle)
		&& (launcher.info->last_s2 != rc_sensor.info->s2))
	{
		if (launcher.work_info->fric_status == WaitCommond_Fric)
		{
			launcher.work_info->fric_status = On_Fric;
		}
		else if (launcher.work_info->fric_status == On_Fric)
		{
			launcher.work_info->fric_status = Off_Fric;
		}
		else if (launcher.work_info->fric_status == Off_Fric)
		{
			launcher.work_info->fric_status = On_Fric;
		}
	}
	if (launcher.work_info->launcher_commond == Adjust_Speed)
	{
		if (last_thumbwheel <= -600 && rc_sensor.info->thumbwheel > -590)
		{
			Fric_SpeedInc();
		}
		else if (last_thumbwheel >= 600 && rc_sensor.info->thumbwheel < 590)
		{
			Fric_SpeedDec();
		}
		last_thumbwheel = rc_sensor.info->thumbwheel;
	}
}

/**
  * @brief  增加摩擦轮转速
  * @param  
  * @retval 
  */
void Fric_SpeedInc(void)
{
	if(launcher.conf->fric_speed == Fric_30)
	{
		launcher.conf->fric_speed = Fric_30;
	}
	else if (launcher.conf->fric_speed == Fric_22)
	{
		launcher.conf->fric_speed = Fric_30;
	}
	else if (launcher.conf->fric_speed == Fric_20)
	{
		launcher.conf->fric_speed = Fric_22;
	}
	else if (launcher.conf->fric_speed == Fric_18)
	{
		launcher.conf->fric_speed = Fric_20;
	}
	else if (launcher.conf->fric_speed == Fric_15)
	{
		launcher.conf->fric_speed = Fric_18;
	}
}

/**
  * @brief  降低摩擦轮转速
  * @param  
  * @retval 
  */
void Fric_SpeedDec(void)
{
	if(launcher.conf->fric_speed == Fric_15)
	{
		launcher.conf->fric_speed = Fric_15;
	}
	else if (launcher.conf->fric_speed == Fric_18)
	{
		launcher.conf->fric_speed = Fric_15;
	}
	else if (launcher.conf->fric_speed == Fric_20)
	{
		launcher.conf->fric_speed = Fric_18;
	}
	else if (launcher.conf->fric_speed == Fric_22)
	{
		launcher.conf->fric_speed = Fric_20;
	}
	else if (launcher.conf->fric_speed == Fric_30)
	{
		launcher.conf->fric_speed = Fric_22;
	}
}


/**
  * @brief  更新拨盘控制信息
  * @param  
  * @retval 
  */
void Dial_StatusCheck(void)
{
	/**    拨盘功能复位    **/
	if (launcher.work_info->launcher_commond == Func_Reset)
	{
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
	
	/**    摩擦轮状态检测    **/
	if (launcher.work_info->fric_status != On_Fric)
	{
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
	
	/**    更新拨盘控制信息    **/
	if (launcher.work_info->dial_status == Reload_Dial)
	{
		if (launcher.info->measure_dial_angle < launcher.info->target_dial_angle + launcher.conf->lock_angle_check)
		{
			if (launcher.work_info->launcher_commond == Single_Shoot)
			{
				/**    单发    **/
				launcher.work_info->dial_status = WaitCommond_Dial;
			}
			else if (launcher.work_info->launcher_commond == Keep_Shoot)
			{
				/**    连发    **/
				if (++launcher.work_info->shoot_cnt == 60000)
				{
					launcher.work_info->shoot_cnt = 0;
				}
				
				if (launcher.work_info->shoot_cnt % launcher.conf->wait_time == 0)
				{
					launcher.info->target_dial_angle = launcher.conf->Load_Angle + launcher.info->measure_dial_angle;
				}
			}
			else if (launcher.work_info->launcher_commond == Sweep_Shoot)
			{
				/**    速射连发    **/
				launcher.work_info->dial_status = SpeedKeep_Dial;
			}
		}
		else 
		{
			/**    拨盘角度堵转检测    **/
			if (abs(launcher.info->measure_dial_speed) < 5)
			{
				launcher.work_info->lock_cnt ++;
				if (launcher.work_info->lock_cnt > launcher.conf->lock_cnt)
				{
					launcher.work_info->dial_status = F_Lock_Dial;
					launcher.info->target_dial_angle = launcher.conf->Back_Angle + launcher.info->measure_dial_angle;
					launcher.work_info->lock_cnt = 0;
				}
			}
			else 
			{
				launcher.work_info->lock_cnt = 0;
			}
		}
	}
	else if (launcher.work_info->dial_status == SpeedKeep_Dial)
	{
		/**    拨盘速度堵转检测    **/
		if (motor[DIAL].c_stuck_flag(&motor[DIAL], launcher.conf->dial_torque_limit))
		{
			launcher.work_info->dial_status = F_Lock_Dial;
			launcher.info->target_dial_angle = launcher.conf->Back_Angle + launcher.info->measure_dial_angle;
		}
		else
		{
			launcher.info->target_dial_speed = launcher.conf->dial_speed;
		}
	}
	else if (launcher.work_info->dial_status == F_Lock_Dial)
	{
		/**    拨盘堵转处理    **/
		if (launcher.info->measure_dial_angle > launcher.info->target_dial_angle - launcher.conf->lock_angle_check)
		{
			launcher.info->target_dial_angle = launcher.conf->Load_Angle + launcher.info->measure_dial_angle;
			launcher.work_info->dial_status = Reload_Dial;
		}
		else 
		{
			if (abs(launcher.info->measure_dial_speed) < 5)
			{
				launcher.work_info->lock_cnt ++;
				if (launcher.work_info->lock_cnt > launcher.conf->lock_cnt)
				{
					launcher.work_info->dial_status = Reload_Dial;
					launcher.info->target_dial_angle = -launcher.conf->Back_Angle + launcher.info->measure_dial_angle;
					launcher.work_info->lock_cnt = 0;
				}
			}
			else 
			{
				launcher.work_info->lock_cnt = 0;
			}
		}
	}
	else
	{
		launcher.work_info->dial_status = WaitCommond_Dial;
	}
}



/**
  * @brief  发射机构电机控制
  * @param  
  * @retval 
  */
void Launcher_MotorCtrl(void)
{
	Fric_Ctrl();
	Dial_Ctrl();
}


/**
  * @brief  摩擦轮电机控制
  * @param  
  * @retval 
  */
void Fric_Ctrl(void)
{
	if (launcher.work_info->fric_status == On_Fric)
	{
		launcher.info->target_left_speed = -launcher.conf->fric_speed;
		launcher.info->target_right_speed = launcher.conf->fric_speed;
	
		launcher_out[motor[FRIC_L].id.buff_p] = motor[FRIC_L].c_speed(&motor[FRIC_L], 
		                                        launcher.info->target_left_speed);
		
		launcher_out[motor[FRIC_R].id.buff_p] = motor[FRIC_R].c_speed(&motor[FRIC_R], 
		                                        launcher.info->target_right_speed);
		
	}
	else if (launcher.work_info->fric_status == Off_Fric || launcher.work_info->fric_status == WaitCommond_Fric)
	{
		launcher.info->target_left_speed = 0.0f;
		launcher.info->target_right_speed = 0.0f;
		
		launcher_out[motor[FRIC_L].id.buff_p] = motor[FRIC_L].c_speed(&motor[FRIC_L], 
		                                        launcher.info->target_left_speed);
		
		launcher_out[motor[FRIC_R].id.buff_p] = motor[FRIC_R].c_speed(&motor[FRIC_R], 
		                                        launcher.info->target_right_speed);
		
	}
}

/**
  * @brief  拨盘电机控制
  * @param  
  * @retval 
  */
void Dial_Ctrl(void)
{
	if (launcher.work_info->dial_status == SpeedKeep_Dial)
	{
		
		launcher_out[motor[DIAL].id.buff_p] = motor[DIAL].c_pid1(&motor[DIAL].pid.position_in, 
		                                                         motor[DIAL].rx_info.speed,
                                                      		   launcher.info->target_dial_speed);
		
	}
	else if (launcher.work_info->dial_status != WaitCommond_Dial)
	{
		// 过零点处理
		if (launcher.info->target_dial_angle < -360.0f)
		{
			launcher.info->target_dial_angle += 360.0f;
		}
		else if (launcher.info->target_dial_angle > 360.0f)
		{
			launcher.info->target_dial_angle -= 360.0f;
		}
		
		if (launcher.info->target_dial_angle - launcher.info->measure_dial_angle > 180.0f)
		{
			launcher.info->target_dial_angle -= 360.0f;
		}
		else if (launcher.info->target_dial_angle - launcher.info->measure_dial_angle < -180.0f)
		{
			launcher.info->target_dial_angle += 360.0f;
		}
		
		launcher_out[motor[DIAL].id.buff_p] = motor[DIAL].c_posit(&motor[DIAL], 
		                                      launcher.info->target_dial_angle / M2006_ECD_TO_ANGLE);
		
	}
	else 
	{
		launcher_out[motor[DIAL].id.buff_p] = motor[DIAL].c_posit(&motor[DIAL], 
		                                      launcher.info->measure_dial_angle / M2006_ECD_TO_ANGLE);
	}
	
}




/**
  * @brief  发送发射机构电机控制数据
  * @param  
  * @retval 
  */
void Launcher_SendOut(void)
{
	CAN1_Send_With_int16_to_uint8(motor[FRIC_L].id.tx_id,launcher_out);
//	CAN1_Send_With_int16_to_uint8(motor[FRIC_R].id.tx_id,launcher_out);
//	CAN1_Send_With_int16_to_uint8(motor[DIAL].id.tx_id,launcher_out);
	
}



/**
  * @brief  发射机构控制离线保护
  * @param  
  * @retval 
  */
void Launcher_SelfProtect(void)
{
	Launcher_Stop();
	Launcher_GetInfo();
}


/**
  * @brief  发射机构卸力
  * @param  
  * @retval 
  */
void Launcher_Stop(void)
{
	launcher_out[0] = 0;
	launcher_out[1] = 0;
	launcher_out[2] = 0;
	
	CAN1_Send_With_int16_to_uint8(motor[FRIC_L].id.tx_id,launcher_out);
//	CAN1_Send_With_int16_to_uint8(motor[FRIC_R].id.tx_id,launcher_out);
//	CAN1_Send_With_int16_to_uint8(motor[DIAL].id.tx_id,launcher_out);
}
