/**
 * @file        launcher.c/.h
 * @author      SSDCFXB
 * @Version     V1.0
 * @date        20-November-2022
 * @brief       Launcher Control Center
 * @update
 *              v1.0(18-November-2022)
 *              v1.1(20-November-2022)
 *                  1.修改速射连发模式，闭角度环->闭速度环
 */

#ifndef __LAUNCHER_H
#define __LAUNCHER_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"

#include "rc_sensor.h"
#include "user_motor.h"

/* Exported macro ------------------------------------------------------------*/
#define Fric_15 4400.0f
#define Fric_18 4700.0f
#define Fric_20 4950.0f
#define Fric_22 5200.0f
#define Fric_30 7020.0f

/* Exported types ------------------------------------------------------------*/
// 摩擦轮状态枚举
typedef enum
{
	On_Fric,        // 开启
	Off_Fric,       // 关闭
	WaitCommond_Fric // 等待指令
} fric_status_e;

// 拨盘状态枚举
typedef enum
{
	Reload_Dial,     // 补弹
	F_Lock_Dial,     // 正向卡弹
	SpeedKeep_Dial,  // 速度保持
//	Unload_Dial,   // 退弹
//	B_Lock_Dial ,  // 反向卡弹
	WaitCommond_Dial // 等待指令
} dial_status_e;

// 发射机构指令枚举
typedef enum
{
	Fric_Toggle,    // 开关摩擦轮
	Func_Reset,     // 功能复位
	Adjust_Speed,   // 调整摩擦轮转速
	Sweep_Shoot,    // 快速连发
	Single_Shoot,   // 单发
	Keep_Shoot,     // 连发
	WaitCommond_L // 等待指令
} launcher_commond_e;

typedef struct 
{
	motor_t  *user_motor;
	rc_sensor_t		*rc_sensor;
} launcher_dev_t;

typedef struct
{
	remote_mode_t	 remote_mode;
	int16_t  measure_left_speed;
	int16_t  measure_right_speed;
	int16_t  measure_dial_speed;
	float    measure_dial_angle;
	
	float    limit_speed;
	float    measure_launcher_speed;
	uint16_t limit_heat;
	uint16_t measure_launcher_heat;
	
	float  target_left_speed;
	float  target_right_speed;
	float  target_dial_speed;
	float  target_dial_angle;
	
	uint8_t  init_s2;
	uint8_t  last_s2;
	
	dev_work_state_t  rc_work_state;
} launcher_info_t;

typedef struct
{
 	launcher_commond_e  launcher_commond;
	fric_status_e       fric_status;
	dial_status_e       dial_status;
	uint16_t            lock_cnt;
	uint16_t            shoot_cnt;
} launcher_work_info_t;

typedef struct
{
	float    fric_speed;
	float    dial_speed;
	float    dial_torque_limit;
	float    lock_angle_check;
	int16_t  lock_cnt;
	float    Back_Angle;
	float    Load_Angle;
	uint16_t wait_time;
} launcher_conf_t;

typedef struct 
{
	launcher_dev_t        *dev;
	launcher_info_t       *info;
	launcher_work_info_t  *work_info;
	launcher_conf_t       *conf;
	void			(*init)(void);
	void			(*ctrl)(void);
	void			(*self_protect)(void);
} launcher_t;

extern launcher_t launcher;

/* Exported functions --------------------------------------------------------*/
//void Launcher_GetInfo(void);
//void Launcher_Stop(void);
//void Launcher_RcCtrl(void);
//void Launcher_PidCtrl(void);

#endif
