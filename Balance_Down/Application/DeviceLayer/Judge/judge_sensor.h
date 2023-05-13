#ifndef __JUDGE_SENSOR_H
#define __JUDGE_SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"                  // Device header
#include "rp_config.h"

#define JUDGE_ONLINE   judge_sensor.work_state == DEV_ONLINE
#define JUDGE_OFFLINE  judge_sensor.work_state == DEV_OFFLINE

#define CHAS_MAX_POWER_BUFFER  60

/*
裁判系统发给我们的数据：

1）帧头------------std_frame_header_t(5个字节)

-----帧头的偏移---->judge_frame_header_offset_t

2）命令码------------CMD_ID（2个字节）



3）数据段-----------从第7个开始，长度不是完全相同
不同的指令对应不同的数据包



4）帧尾------2个字节





有两个偏移位置：
judge_frame_offset_t--------------->对整个数据包的偏移分类

judge_frame_header_offset_t--------->对帧头的单独偏移分类
*/



typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;

//帧字节（单位/字节）
typedef enum {//帧头占五个字节，命令码占两个字节，从第七个开始为数据帧
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7
}judge_frame_offset_t;
//帧头字节（单位/bit）
typedef enum  {//帧头的详细位置
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
}judge_frame_header_offset_t;//FRAME_HEADER

//LEN_CMD_ID
//LEN_FRAME_TAIL
/* ID: 0x0001	Byte: 	11	比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type     : 4;		// 比赛类型
	uint8_t game_progress : 4;		// 比赛阶段
	uint16_t stage_remain_time;		// 当前阶段剩余时间(单位:s)
	uint64_t SyncTimeStamp;       //机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效
} ext_game_status_t; //LEN_GAME_STATUS   表3-1
/* ID: 0x0002	Byte:	1	比赛结果数据 */
typedef __packed struct
{
	uint8_t winner;
} ext_game_result_t; //LEN_GAME_RESULT   表3-2
/* ID: 0x0003	Byte:	32	机器人血量数据数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
	uint16_t red_2_robot_HP;	// 红2工程机器人血量
	uint16_t red_3_robot_HP;	// 红3步兵机器人血量
	uint16_t red_4_robot_HP;	// 红4步兵机器人血量
	uint16_t red_5_robot_HP;	// 红5步兵机器人血量
	uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
	uint16_t red_outpost_HP;	// 红方前哨站血量
	uint16_t red_base_HP;			// 红方基地血量
	uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
	uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
	uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
	uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
	uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
	uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
	uint16_t blue_outpost_HP;	// 蓝方前哨站血量
	uint16_t blue_base_HP;		// 蓝方基地血量	
} ext_game_robot_HP_t; //LEN_GAME_ROBOT_HP  表3-3
///* ID: 0x0004 	Byte:	3	飞镖发射状态 */
//typedef __packed struct
//{
//	uint8_t dart_belong;
//	uint16_t stage_remaining_time;
//} ext_dart_status_t;//LEN_DART_STATUS   表3-4
/* ID: 0x0005 	Byte:	11	人工智能挑战赛加成与惩罚区状态 */
typedef __packed struct
{
	uint8_t F1_zone_status:1;            //激活状态
	uint8_t F1_zone_buff_debuff_status:3;//红方回血区
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3;//红方弹药补给区
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3;//蓝方回血区
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3;//蓝方弹药补给区
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3;//禁止射击区
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;//禁止移动区
	uint16_t red1_bullet_left;           //红方 1 号剩余弹量
	uint16_t red2_bullet_left;           //红方 2 号剩余弹量
	uint16_t blue1_bullet_left;          //蓝方 1 号剩余弹量
	uint16_t blue2_bullet_left;          //蓝方 2 号剩余弹量
} ext_ICRA_buff_debuff_zone_status_t;//LEN_ICRA_BUFF_DEBUFF_ZONE_STATUS  表3-5
/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;//己方补给站补血点占领状态,能量机关状态, 环形高地占领状态,己方基地护盾状态,己方前哨战状态
} ext_event_data_t; //LEN_EVENT_DATA   表3-6
/* ID: 0x0102  Byte:  4    补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;  //补给站口 ID
	uint8_t supply_robot_id;       //补弹机器人 ID
	uint8_t supply_projectile_step;//出弹口开闭状态
	uint8_t supply_projectile_num; //补弹数量
} ext_supply_projectile_action_t;//LEN_SUPPLY_PROJECTILE_ACTION  表3-7
//LEN_SUPPLY_PROJECTILE_BOOKING
/* ID: 0X0104  Byte:  2	   裁判警告信息 */
typedef __packed struct
{
	uint8_t level;        //警告等级
	uint8_t foul_robot_id;//犯规机器人 ID
} ext_referee_warning_t;//LEN_REFEREE_WARNING   表3-8
/* ID: 0X0105  Byte:  1	   飞镖发射口倒计时 */
typedef __packed struct
{
	uint8_t dart_remaining_time;//15s 倒计时
} ext_dart_remaining_time_t;//LEN_DART_REMAINING_TIME
/* ID: 0X0201  Byte: 27    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;                       //本机器人 ID
	uint8_t robot_level;                    //机器人等级
	uint16_t remain_HP;                     //机器人剩余血量
	uint16_t max_HP;                        //机器人上限血量
	uint16_t shooter_id1_17mm_cooling_rate; //机器人 1 号 17mm 枪口每秒冷却值
	uint16_t shooter_id1_17mm_cooling_limit;//机器人 1 号 17mm 枪口热量上限
	uint16_t shooter_id1_17mm_speed_limit;  //机器人 1 号 17mm 枪口上限速度 单位 m/s
	uint16_t shooter_id2_17mm_cooling_rate; //机器人 2 号 17mm 枪口每秒冷却值
	uint16_t shooter_id2_17mm_cooling_limit;//机器人 2 号 17mm 枪口热量上限
	uint16_t shooter_id2_17mm_speed_limit;  //机器人 2 号 17mm 枪口上限速度 单位 m/s
	uint16_t shooter_id1_42mm_cooling_rate; //机器人 42mm 枪口每秒冷却值
	uint16_t shooter_id1_42mm_cooling_limit;//机器人 42mm 枪口热量上限
	uint16_t shooter_id1_42mm_speed_limit;  //机器人 42mm 枪口上限速度 单位 m/s
	uint16_t chassis_power_limit;           //机器人底盘功率限制上限
	//主控电源输出情况
	uint8_t mains_power_gimbal_output  : 1; //gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出
	uint8_t mains_power_chassis_output : 1; //chassis 口输出：1 为有 24V 输出，0 为无 24v 输出
	uint8_t mains_power_shooter_output : 1; //shooter 口输出：1 为有 24V 输出，0 为无 24v 输出
} ext_game_robot_status_t; //LEN_GAME_ROBOT_STATUS  表3-10
/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   				// 底盘输出电压，单位：mV
	uint16_t chassis_current;				// 底盘输出电流，单位：mA
	float chassis_power;   					// 底盘瞬时功率，单位：W
	uint16_t chassis_power_buffer;	// 底盘功率缓冲，单位：60J焦耳缓冲能量(飞坡根据规则增加至250J)
	uint16_t shooter_id1_17mm_cooling_heat; //1 号 17mm 枪口热量
	uint16_t shooter_id2_17mm_cooling_heat; //2 号 17mm 枪口热量
	uint16_t shooter_id1_42mm_cooling_heat;	//42mm 枪口热量
} ext_power_heat_data_t; //LEN_POWER_HEAT_DATA   表3-11
/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; //位置枪口,单位度数
} ext_game_robot_pos_t; //LEN_GAME_ROBOT_POS   表3-12
/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; //LEN_BUFF_MASK  表3-13
/* ID: 0x0205  Byte:  1    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t attack_time;//可攻击时间 单位 s。30s 递减至 0
} ext_aerial_robot_energy_t; //LEN_AERIAL_ROBOT_ENERGY  表3-14
/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 	// 装甲伤害时代表装甲ID
	uint8_t hurt_type : 4; 	// 0x0装甲伤害 0x1模块掉线 0x2超射速 0x3超热量 0x4超功率 0x5撞击
} ext_robot_hurt_t; //LEN_ROBOT_HURT  表3-15
/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type; 	// 子弹类型(1-17mm, 2-42mm)
	uint8_t shooter_id;   // 发射机构 ID（1：1 号 17mm 发射机构，2：2 号 17mm 发射机构，3：42mm 发射机构）
	uint8_t bullet_freq;  // 子弹射频(Hz)
	float bullet_speed;		// 子弹射速(m/s)
} ext_shoot_data_t; //LEN_SHOOT_DATA  表3-16
/* ID: 0x0208  Byte:  6    子弹剩余发射数数据 */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;//17mm 子弹剩余发射数目
	uint16_t bullet_remaining_num_42mm;//42mm 子弹剩余发射数目
	uint16_t coin_remaining_num;       //剩余金币数量
} ext_bullet_remaining_t; //LEN_BULLET_REMAINING   表3-17
/* ID: 0x0209  Byte:  4 	机器人RFID状态 */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;//LEN_RFID_STATUS  表3-18
/*ID：0x020A   Byte：6  飞镖机器人客户端指令数据*/
typedef struct{
	uint8_t dart_launch_opening_status;//当前飞镖发射口的状态
	uint8_t dart_attack_target;        //飞镖的打击目标，默认为前哨站（1：前哨站，2：基地）
	uint16_t target_change_time;       //切换打击目标时的比赛剩余时间
	uint16_t operate_launch_cmd_time;  //最近一次操作手确定发射指令时的比赛剩余时间，单位秒
} ext_dart_client_cmd_t; //LEN_DART_CLIENT_DIRECTIVE  表3-19

/*
	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	9，雷达（红）
	101，英雄(蓝)；
	102，工程(蓝)；
	103/104/105，步兵(蓝)；
	106，空中(蓝)；
	107，哨兵(蓝)；
	109，雷达（蓝）

	客户端 ID： 
	0x0101 为英雄操作手客户端(红) ；
	0x0102 为工程操作手客户端( 红 )；
	0x0103/0x0104/0x0105 为步兵操作手客户端(红)；
	0x0106 为空中操作手客户端((红)； 

	0x0165，英雄操作手客户端(蓝)；
	0x0166，工程操作手客户端(蓝)；
	0x0167/0x0168/0x0169，步兵操作手客户端(蓝)；
	0x016A，空中操作手客户端(蓝)。 
*/
enum judge_robot_ID{
	hero_red       = 1,
	engineer_red   = 2,
	infantry3_red  = 3,
	infantry4_red  = 4,
	infantry5_red  = 5,
	plane_red      = 6,
	
	hero_blue      = 101,
	engineer_blue  = 102,
	infantry3_blue = 103,
	infantry4_blue = 104,
	infantry5_blue = 105,
	plane_blue     = 106,
};
typedef struct{
	uint16_t teammate_hero;
	uint16_t teammate_engineer;
	uint16_t teammate_infantry3;
	uint16_t teammate_infantry4;
	uint16_t teammate_infantry5;
	uint16_t teammate_plane;
	uint16_t teammate_sentry;
	
	uint16_t client_hero;
	uint16_t client_engineer;
	uint16_t client_infantry3;
	uint16_t client_infantry4;
	uint16_t client_infantry5;
	uint16_t client_plane;
} ext_interact_id_t;

typedef __packed struct
{
	uint8_t find_enemy   :1;
	uint8_t find_hero    :1;
	uint8_t fine_infantry:2;//后面可以加发现几个步兵
	uint8_t stealed_tower:2;//被偷塔
	uint8_t meaningless  :2;//后面加功能
}ext_sentry_t;

typedef struct judge_info_struct {
	std_frame_header_t							fream_header;				// 帧头信息
	
	ext_game_status_t 							game_status;				// 0x0001
	ext_game_result_t 							game_result;				// 0x0002
	ext_game_robot_HP_t 						game_robot_HP;			// 0x0003
//	ext_dart_status_t								dart_status;				// 0x0004
	ext_ICRA_buff_debuff_zone_status_t	ICRA_buff;
	
	ext_event_data_t								event_data;					// 0x0101
	ext_supply_projectile_action_t	supply_projectile_action;		// 0x0102
	//ext_supply_projectile_booking_t supply_projectile_booking;// 0x0103
	ext_referee_warning_t						referee_warning;		// 0x0104
	ext_dart_remaining_time_t				dart_remaining_time;// 0x0105
	
	ext_game_robot_status_t					game_robot_status;	// 0x0201
	ext_power_heat_data_t						power_heat_data;		// 0x0202
	ext_game_robot_pos_t						game_robot_pos;			// 0x0203
	ext_buff_t											buff;								// 0x0204
	ext_aerial_robot_energy_t				aerial_robot_energy;// 0x0205
	ext_robot_hurt_t								robot_hurt;					// 0x0206
	ext_shoot_data_t								shoot_data;					// 0x0207
	ext_bullet_remaining_t					bullet_remaining;		// 0x0208	
	ext_rfid_status_t								rfid_status;				// 0x0209	
	ext_dart_client_cmd_t           dart_client;        // 0x020A
	
	ext_interact_id_t								ids;								//与本机交互的机器人id
	
	uint16_t                        self_client;        //本机客户端
	//????????????????????????????????????
	ext_sentry_t                    sentry_Info_rescue; //哨兵提供的信息
	
	bool	 		data_valid;	// 数据有效性
	bool			err_cnt;
	uint16_t		offline_cnt;
	uint16_t		offline_max_cnt;	
} judge_info_t;



typedef struct judge_sensor_struct {
	judge_info_t		*info;
	drv_uart_t			*driver;
	void		(*init)(struct judge_sensor_struct *self);
	void		(*update)(struct judge_sensor_struct *self, uint8_t *rxBuf);
	void		(*check)(struct judge_sensor_struct *self);	
	void		(*heart_beat)(struct judge_sensor_struct *self);
	void		(*sendclient)(struct judge_sensor_struct *self);
	void		(*sendteammate)(struct judge_sensor_struct *self);
	dev_work_state_e	work_state;
	dev_errno_t				errno;
	dev_id_t					id;
} judge_sensor_t;




extern judge_info_t judge_info;
extern judge_sensor_t judge_sensor;

/* Exported functions --------------------------------------------------------*/

#endif
