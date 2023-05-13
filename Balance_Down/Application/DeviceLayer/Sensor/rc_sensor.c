/**
 * @file        rc_sensor.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Device Rc.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rc_sensor.h"

#include "rp_math.h"


extern void rc_sensor_init(rc_sensor_t *rc_sen);
extern void rc_sensor_update(rc_sensor_t *rc_sen, uint8_t *rxBuf);

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void rc_sensor_check(rc_sensor_t *rc_sen);
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// 遥控器驱动
drv_uart_t	rc_sensor_driver = {
	.id = DRV_UART1,
	.tx_byte = NULL,
};

Keyboard_Info_t keyboard;

// 遥控器信息
rc_sensor_info_t 	rc_sensor_info = {
	.offline_max_cnt = 60,
};

// 遥控器传感器
rc_sensor_t	rc_sensor = {
	.info = &rc_sensor_info,
	.init = rc_sensor_init,
	.update = rc_sensor_update,
	.check = rc_sensor_check,
	.heart_beat = rc_sensor_heart_beat,
	.work_state = DEV_OFFLINE,
	.id = DEV_ID_RC,
};


//-------------------------------------------虚函数接口


//设置State里的ch_wise标志位
__weak void Link_RC_CH_Is_Reset_Handler(char flag)
{
	
}
//返回State里的ch_wise标志位,没有重写就一直返回0
__weak bool Link_RC_Judge_Is_Wise_Enable_Handler(void)
{
	return 0;
}
//接了接收机时遥控器失联要做的事
//必须重置State里面有关遥控器的标志位
__weak void Link_RC_Offline_Handler(void)
{
	
}
//连接接收机后更新State关于遥控器的标志位
__weak void Link_RC_State_UP_Handler(char state)
{
	
}




/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	遥控器数据检查
 */
static void rc_sensor_check(rc_sensor_t *rc_sen)
{
	rc_sensor_info_t *rc_info = rc_sen->info;
	
	if(abs_(rc_info->ch0) > 660 ||
	   abs_(rc_info->ch1) > 660 ||
	   abs_(rc_info->ch2) > 660 ||
	   abs_(rc_info->ch3) > 660)
	{
		rc_sen->errno = DEV_DATA_ERR;
		rc_info->ch0 = 0;
		rc_info->ch1 = 0;
		rc_info->ch2 = 0;
		rc_info->ch3 = 0;		
		rc_info->s1 = RC_SW_MID;
		rc_info->s2 = RC_SW_MID;
		rc_info->thumbwheel = 0;
	}
	else
	{
		rc_sen->errno = NONE_ERR;
	}
}



/**
 *	@brief	遥控器心跳包
 */
static void rc_sensor_heart_beat(rc_sensor_t *rc_sen)
{
	rc_sensor_info_t *rc_info = rc_sen->info;

	rc_info->offline_cnt++;
	if(rc_info->offline_cnt > rc_info->offline_max_cnt) {
		rc_info->offline_cnt = rc_info->offline_max_cnt;
		rc_sen->work_state = DEV_OFFLINE;
	} 
	else {
		/* 离线->在线 */
		if(rc_sen->work_state == DEV_OFFLINE)
		{
			rc_sen->work_state = DEV_ONLINE;
		}
	}
	
	Link_RC_State_UP_Handler(rc_sen->work_state);
}

/* Exported functions --------------------------------------------------------*/


/**
 *	@brief	遥控器检查是否在中值处
 */
bool RC_IsChannelReset(void)
{
	if(  (DeathZoom(rc_sensor_info.ch0, 0, 1) == 0) && 
		 (DeathZoom(rc_sensor_info.ch1, 0, 1) == 0) && 
		 (DeathZoom(rc_sensor_info.ch2, 0, 1) == 0) && 
		 (DeathZoom(rc_sensor_info.ch3, 0, 1) == 0)   )	
	{
		return true;
	}
	return false;		
}

/**
 *	@brief	强制设置遥控器的值
 */
void RC_ResetData(rc_sensor_t *rc)
{
	// 通道值强行设置成中间值(不拨动摇杆的状态)
	rc->info->ch0 = 0;
	rc->info->ch1 = 0;
	rc->info->ch2 = 0;
	rc->info->ch3 = 0;
	// 左右开关选择强行设置成中间值状态
	rc->info->s1 = RC_SW_MID;
	rc->info->s2 = RC_SW_MID;
	// 鼠标
	rc->info->mouse_t.mouse_vx = 0;
	rc->info->mouse_t.mouse_vy = 0;
	rc->info->mouse_t.mouse_vz = 0;
	rc->info->mouse_t.mouse_press_l = 0;
	rc->info->mouse_t.mouse_press_r = 0;
	// 键盘
	rc->info->key_u.key_code_val = 0;
	// 左拨轮
	rc->info->thumbwheel = 0;
}







/*------------------------按键按下的状态开始------------------------*/
void FirstGetInto_KEY_PRESS(KEY_Info_t *str)
{
  if(str->prev_KEY_PRESS != str->KEY_PRESS)
  {
    str->state_cnt = 0;
    str->prev_KEY_PRESS = str->KEY_PRESS;
  }
}
/**
 *	@brief	强制设置遥控器的值
		@param  第二个参数表示这个键位本次的bit值，通过宏定义可以索引这个键位的bit值			
 */
void KEY_State_Judge(KEY_Info_t *str , uint8_t KEY_PRESS , int change_tim ,int long_change_tim)
{
  str->KEY_PRESS = KEY_PRESS; //保存本次的bit值
  FirstGetInto_KEY_PRESS(str);
  switch(KEY_PRESS)
  {
		//这次松手
    case KEY_UP:  {
			//如果上一次是按下
      if(str->prev_State != UP) 
      {
        str->state_cnt++;
				//达到一定条件才算1-->0
        if(str->state_cnt >= change_tim)  
        {
          str->this_State = RELAX;
          str->prev_State = RELAX;
					//达到一定条件才算0
          if(str->state_cnt >= change_tim + 1)  //抬起不分长短抬
          {
            str->this_State = UP;
            str->prev_State = UP;
          }
        }		
      }
			//如果上一次是松手
			else
				str->state_cnt = 0;
    }	
		break;
    
		
		//这次是按下
    case KEY_DOWN:    { 
			//上次是松手
      if(str->prev_State != DOWN) 
      {
        str->state_cnt++;
        if(str->state_cnt >= change_tim)  
        {
          str->this_State = PRESS;
          str->prev_State = PRESS;
          if(str->state_cnt >= change_tim + 1)
          {
            str->this_State = SHORT_DOWN;
            str->prev_State = SHORT_DOWN;
            if(str->state_cnt >= long_change_tim)  
            {
              str->this_State = DOWN;
              str->prev_State = DOWN;
            }
          }
        }
      }
			//上次是按下
			else
				str->state_cnt = 0;
    }
		break;
  }
}




/*--------------------键盘值转为遥控通道值，跟按键是长按还是短按无关 begin---------------------*/

//wsad中间通道值，最后计算结果赋予rc中模拟通道值Ch[]
float W_Chan,
      S_Chan,
      A_Chan,
      D_Chan;


void Sim_Channel(float *CH, char dir,float num)//1up 0down
{
	if( dir)*CH+=num;
	if(!dir)*CH-=num;
	
	if(*CH > CH_MAX)     *CH = CH_MAX;
	if(*CH < CH_OFFSET  )*CH = CH_OFFSET;
}

//合并前后通道值
void WS_Channel(void) 
{
	int16_t CH;
	CH = W_Chan - S_Chan;	
	if(abs_(CH) > CH_MAX)
	{
		if(CH > 0)CH = CH_MAX;
		if(CH < 0)CH = CH_MIN;
	}
	rc_sensor.info->Ch[3] = CH;
}
//合并左右通道值
void AD_Channel(void) 
{
	int16_t CH;
	CH = D_Chan - A_Chan;
	if(abs_(CH) > CH_MAX)
	{
		if(CH > 0)CH = CH_MAX;
		if(CH < 0)CH = CH_MIN;
	}
	rc_sensor.info->Ch[2] = CH;
}




/*-缓慢减速-*/
float CH_UP   = 1.f;
float CH_DOWN = 3.f;
void Key_Channel_Update(void)
{
	if(KEY_W)Sim_Channel(&W_Chan, 1, CH_UP);
	else     Sim_Channel(&W_Chan, 0, CH_DOWN);
	
	if(KEY_S)Sim_Channel(&S_Chan, 1, CH_UP);
	else		 Sim_Channel(&S_Chan, 0, CH_DOWN);

	if(KEY_A)Sim_Channel(&A_Chan, 1, CH_UP);
	else     Sim_Channel(&A_Chan, 0, CH_DOWN);

	if(KEY_D)Sim_Channel(&D_Chan, 1, CH_UP);
	else     Sim_Channel(&D_Chan, 0, CH_DOWN);
	
	WS_Channel();
	AD_Channel();
}



/*-----------------------键盘值转为遥控通道值结束---------------------*/



/**
 * @brief 鼠标卡尔曼
 * @note  已加滑动滤波
 * @param 
 */

//这一次滤波后的速度
float Mouse_X_UPDATA, Mouse_Y_UPDATA, Mouse_Z_UPDATA;
//上一次的通道值
float Mouse_X_Last, Mouse_Y_Last, Mouse_Z_Last;


//新数据保存到数组的第一个位置
float SF(float t,float *slopeFilter,float res)
{
  for(int i = SF_LENGTH-1;i>0;i--)
  {
    slopeFilter[i] = slopeFilter[i-1];
  }
	
	slopeFilter[0] = t;
	
  for(int i = 0;i<SF_LENGTH;i++)
  {
    res += slopeFilter[i];
  }
	return (res/SF_LENGTH);
}

//鼠标转变为通道值并返回本次滤波后的速度
float Mouse_X_Speed(void)
{
  float res;
	
	Mouse_X_Last = rc_sensor.info->Ch[0];
	
  if(abs_(MOUSE_X_MOVE_SPEED > Xmax))res = 0;
  else 
		res = SF(MOUSE_X_MOVE_SPEED,keyboard.MouseSF.SFX,0);
	
	//转变为-660~660
	rc_sensor.info->Ch[0] = (0.7f * MOUSE_X_RATE * res + 0.3f * Mouse_X_Last);
	
	rc_sensor.info->Ch[0] = constrain(rc_sensor.info->Ch[0], CH_MIN, CH_MAX);
	
  return res;
}


float Mouse_Y_Speed(void)
{
  float res;
	
	Mouse_Y_Last = rc_sensor.info->Ch[1];
	
  if(abs_(MOUSE_Y_MOVE_SPEED > Ymax))res = 0;
  else
		res = -SF(MOUSE_Y_MOVE_SPEED,keyboard.MouseSF.SFY,0);

	rc_sensor.info->Ch[1] = (0.7f * MOUSE_Y_RATE * res + 0.3f * Mouse_Y_Last);
	
	rc_sensor.info->Ch[1] = constrain(rc_sensor.info->Ch[1], CH_MIN, CH_MAX);
	
	if(abs_(rc_sensor_info.Ch[1]) < 1)rc_sensor.info->Ch[1] = 0;
	
  return res;
}


float Mouse_Z_Speed(void)
{
  float res;
	
	Mouse_Z_Last = Mouse_Z_UPDATA;
	 
	res = SF(MOUSE_Z_MOVE_SPEED,keyboard.MouseSF.SFZ,0);
	
	res = (0.8f * res + 0.2f * Mouse_Z_Last);
	
	//rc_sensor.info->Mouse_ROLL = res;
	
  return res;
}


//保存本次滤波后的速度
void Mouse_Updata(void)
{
	Mouse_X_UPDATA = Mouse_X_Speed();
	Mouse_Y_UPDATA = Mouse_Y_Speed();
	Mouse_Z_UPDATA = Mouse_Z_Speed();
}

void Mouse_FS(void)
{
	
	Mouse_Updata();
	
	Mouse_X_Last = rc_sensor.info->Ch[0];
	rc_sensor.info->Ch[0] = (0.7f * MOUSE_X_RATE * Mouse_X_UPDATA + 0.3f * Mouse_X_Last);

	Mouse_Y_Last = rc_sensor.info->Ch[1];
	rc_sensor.info->Ch[1] = (0.7f * MOUSE_Y_RATE * Mouse_Y_UPDATA + 0.3f * Mouse_Y_Last);

  rc_sensor.info->Mouse_ROLL = Mouse_Z_UPDATA;
	
	rc_sensor.info->Ch[0] = constrain(rc_sensor.info->Ch[0], CH_MIN, CH_MAX);
	
	rc_sensor.info->Ch[1] = constrain(rc_sensor.info->Ch[1], CH_MIN, CH_MAX);
	
}
