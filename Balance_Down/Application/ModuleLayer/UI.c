#include "UI.h"

/*----------私有变量声明----------*/
extern UART_HandleTypeDef huart4;
UI_Data_t 	UI_Data;
uint8_t  Tx_Client_Buf[200];

/*-------------------------------上层-------------------------------*/
void UI_Ctrl(void);

/*-------------------------------应用-------------------------------*/
void UI_Data_ID_UP(void);
void UI_Prepare(void);
void Save_Tx_Pack_Fixed_Info(void);

/*-------------------------------底层--------------------------------*/
void UI_Send_Data_Pack( uint8_t* txbuf, const uint8_t len);




/*-------------------------------PACK1顺序-------------------------------------*/
void UI_TX_ext_pack1(void);
void UI_Save_Chas_Vision_Wide(const uint8_t operate_type);
void UI_Save_Chas_Vision_Length(const uint8_t operate_type);
void UI_Save_Chas_State(const char state, const uint8_t operate_type);

/*-------------------------------PACK2顺序-------------------------------------*/
void UI_TX_ext_pack2(void);
void UI_Save_Aim(const uint8_t operate_type);
void UI_Save_Cap_Data(const float voltage, const uint8_t operate_type);
void UI_Save_Magz_Data(const char state, const uint8_t operate_type);
void UI_Save_Top_Data(const char state, const uint8_t operate_type);






/**	
	*@brief  Tx_Client_Tmp_Buf的大小是下面三个的长度,存放发给裁判系统数据的前固定数据
	std_frame_header_t 											Tx_FrameHeader;	 		 5字节
	uint16_t  															CmdID;							 2字节
		
	ext_student_interactive_data_header_t   Data_FrameHeader;		 6字节
*/
uint8_t  Tx_Client_Tmp_Buf[INTERACT_DATA_FIXED_LEN];


/*--------------------------------通过串口发送一帧数据-----------------------------------*/
void UI_Send_Data_Pack( uint8_t* txbuf, const uint8_t len)
{
	if(txbuf == NULL)
		return;

	HAL_UART_Transmit_DMA(&huart4, txbuf, len);
	
}

/*-----------------------------------显示指定的类型,底层函数----------------------------------------*/



void Draw_String_Or_Char(ext_client_custom_character_t*  graphic,  //讲图形的特点信息放到这里
												 
												 const char* name,     //作为客户端的索引
													 
									       uint32_t operate_type, //图形操作
												 uint32_t layer,				//图形层数
												 uint32_t color,				//图形颜色
												 uint32_t size,					//字符大小						 
												 uint32_t length,				//字符长度 
												 uint32_t width,				//线宽
												 uint32_t start_x,			//X坐标
												 uint32_t start_y,			//Y坐标						 
												 
												 const char *string)		//要显示的字符
{

	if(graphic == NULL || name == NULL || string == NULL)
		return;
	
	graphic_data_struct_t 	*grapic_info = &graphic->grapic_info;
	
	for(char i = 0 ; i < 3; i ++)
		grapic_info->graphic_name[i] = name[i];	
	grapic_info->operate_type = operate_type;
	grapic_info->graphic_type = CHAR;        //Char类型直接赋值
	grapic_info->layer = layer;
	grapic_info->color = color;
	grapic_info->start_angle = size;
	grapic_info->end_angle = length;	
	grapic_info->width = width;
	grapic_info->start_x = start_x;
	grapic_info->start_y = start_y;	
	
	grapic_info->radius = 0;
	grapic_info->end_x = 0;
	grapic_info->end_y = 0;

	memcpy(graphic->buf, string, length);
	
}

void Draw_Firgue(graphic_data_struct_t* graphic,
										
								 const char* name,
									
								 uint32_t operate_type,    
								 uint32_t graphic_type,   
								 uint32_t layer,					 
								 uint32_t color,          
								 uint32_t start_angle,     
								 uint32_t end_angle,
								 uint32_t width,
								 uint32_t start_x,
								 uint32_t start_y,
								 uint32_t radius,
								 uint32_t end_x,
								 uint32_t end_y)							
{
	if(graphic == NULL || name == NULL)
		return;
	
	for(char i = 0; i < 3; i ++)
		graphic->graphic_name[i] = name[i];	//字符索引
	
	graphic->operate_type = operate_type; 
	graphic->graphic_type = graphic_type;        
	graphic->layer        = layer;		
	graphic->color        = color;
	graphic->start_angle  = start_angle;
	graphic->end_angle    = end_angle;	
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->radius       = radius;
	graphic->end_x        = end_x;
	graphic->end_y        = end_y;
	
}

void Draw_Float_Or_Int32(int32_or_float_data_struct_t* graphic,
	
												 const char* name,
													 
												 uint32_t operate_type,
												 uint32_t graphic_type,
												 uint32_t layer,
												 uint32_t color,
												 uint32_t size,
												 uint32_t decimal,	//小数点后的有效位数, 0即显示int32_t类型
												 uint32_t width,
												 uint32_t start_x,
												 uint32_t start_y,
												 
												 int32_t number)							
{
	if(graphic == NULL || name == NULL)
		return;
	
	for(char i = 0; i < 3; i ++)
		graphic->graphic_name[i] = name[i];	
	
	graphic->operate_type = operate_type; 
	graphic->graphic_type = graphic_type;  
	graphic->layer        = layer;
	graphic->color        = color;
	graphic->start_angle  = size;
	graphic->end_angle    = decimal;
	graphic->width        = width;
	graphic->start_x      = start_x;
	graphic->start_y      = start_y;	
	graphic->number       = number;
}




/*-------------------------------总控制---------------------------------------------*/
uint16_t i, j;
void UI_Ctrl(void)
{
	UI_Prepare();
	
	i++;	
	
	//前10ms来发送pack1
	if(i <= 30)
	{
		UI_TX_ext_pack1();
	}
	//后10ms来发送pack2
	else
	{ 
		if(i >= 90)
		{
			i = 0;
			j ++;
		}	
		
		UI_TX_ext_pack2();
	}
	
	//j表示轮了多少次
	if(j <= 2)
	{
		UI_Data.operate.pack1 = UI_Data.operate.pack2 = ADD;
	}
	else if(j <= 6)
	{
		UI_Data.operate.pack1 = UI_Data.operate.pack2 = MODIFY;
	}
	else
	{
		j = 0;
	}
	
	
}










/*--------------------------------应用层-----------------------------------------------------*/




void UI_Prepare(void)
{
	Save_Tx_Pack_Fixed_Info();
	UI_Data_ID_UP();
}
/*id更新*/
void UI_Data_ID_UP(void)
{
	UI_Data.robot_id = judge_info.game_robot_status.robot_id;
	UI_Data.client_id = judge_info.self_client;
	
}
/*发送给裁判系统的数据,在前面一端数据基本是固定的*/
//需更新数据长度,CRC8,数据段头结构的数据命令码
void Save_Tx_Pack_Fixed_Info(void)
{

	//帧头
	Tx_Client_Tmp_Buf[0] = JUDGE_FRAME_HEADER;
	//数据长度
	Tx_Client_Tmp_Buf[1] = Tx_Client_Tmp_Buf[2] = 0;
	//包序号
	Tx_Client_Tmp_Buf[3] = 0;
	//CRC8
	Tx_Client_Tmp_Buf[4] = 0;
	
	
	
	//命令码
	Tx_Client_Tmp_Buf[5] = (uint8_t)ID_robot_interactive_header_data;
	Tx_Client_Tmp_Buf[6] = (uint8_t)(ID_robot_interactive_header_data>>8);
	
	
	
	
	
	//数据段头结构的数据命令码
	Tx_Client_Tmp_Buf[7] = 0;
	Tx_Client_Tmp_Buf[8] = 0;
	//数据段头结构的机器人ID
	Tx_Client_Tmp_Buf[9] =  (uint8_t)UI_Data.robot_id;
	Tx_Client_Tmp_Buf[10] = (uint8_t)(UI_Data.robot_id>>8);
	//数据段头结构的客户端ID
	Tx_Client_Tmp_Buf[11] = (uint8_t)UI_Data.client_id;
	Tx_Client_Tmp_Buf[12] = (uint8_t)(UI_Data.client_id>>8);
	
	
}















/*-------------------------------PACK1顺序-------------------------------------*/





/*发送给客户端PACK1图片组*/

void UI_TX_ext_pack1(void)
{
	/*---------先保存子信息---------*/
	char sideways_state = 0;
	if(State.chassic_flag.sideways_state == SIDEWAYS_ON)
		sideways_state = 1;
	
	UI_Save_Chas_Vision_Wide(UI_Data.operate.pack1);
	UI_Save_Chas_Vision_Length(UI_Data.operate.pack1);
	UI_Save_Chas_State(sideways_state, UI_Data.operate.pack1);
	
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.pack1.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.pack1.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																								 sizeof(graphic_data_struct_t)*5;
	
	UI_Data.ext.pack1.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;	
	
	//头校验CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.pack1.Tx_FrameHeader.sof, sizeof(ext_graphic_five_photo_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_graphic_five_photo_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_graphic_five_photo_t));
	
	
}
/*以直线形式显示底盘视野的宽*/
void UI_Save_Chas_Vision_Wide(const uint8_t operate_type)
{
	/*左宽度*/
	Draw_Firgue(&UI_Data.ext.pack1.Client_Data[0], 
							"CW1", 
							operate_type, 
							LINE,
							1,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Wide_L_X_Start,
							Chas_Vision_Wide_L_Y_Start,
							0,
							Chas_Vision_Wide_L_X_End,
							Chas_Vision_Wide_L_Y_End);	

	/*右宽度*/
	Draw_Firgue(&UI_Data.ext.pack1.Client_Data[1], 
							"CW2", 
							operate_type, 
							LINE,
							1,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Wide_R_X_Start,
							Chas_Vision_Wide_R_Y_Start,
							0,
							Chas_Vision_Wide_R_X_End,
							Chas_Vision_Wide_R_Y_End);	
}

/*以直线形式显示底盘视野的长*/
void UI_Save_Chas_Vision_Length(const uint8_t operate_type)
{
	/*上长度*/
	Draw_Firgue(&UI_Data.ext.pack1.Client_Data[2], 
							"CL1", 
							operate_type, 
							LINE,
							1,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Length_H_X_Start,
							Chas_Vision_Length_H_Y_Start,
							0,
							Chas_Vision_Length_H_X_End,
							Chas_Vision_Length_H_Y_End);	

	/*下长度*/
	Draw_Firgue(&UI_Data.ext.pack1.Client_Data[3], 
							"CL2", 
							operate_type, 
							LINE,
							1,
							Chas_Vision_Wide_Color,
							0,
							0,
							Chas_Vision_Wide_Width,
							Chas_Vision_Length_L_X_Start,
							Chas_Vision_Length_L_Y_Start,
							0,
							Chas_Vision_Length_L_X_End,
							Chas_Vision_Length_L_Y_End);	
}
/*以直线的形式去显示底盘的方位*/

void UI_Save_Chas_State(const char state, const uint8_t operate_type)
{
	if(state == 1)//侧身了用竖线
	{
		UI_Data.data.sideways_state = Open;
		Draw_Firgue(&UI_Data.ext.pack1.Client_Data[4], "CS1", operate_type, LINE, 1, YELLOW, 0, 0, \
								5, 550, 500, 0, 550, 720);
	}
	else //没侧身用横线
	{
		UI_Data.data.sideways_state = Close;
		Draw_Firgue(&UI_Data.ext.pack1.Client_Data[4], "CS1", operate_type, LINE, 1, YELLOW, 0, 0, \
								5, 480, 600, 0, 620, 600 );
	}
	
}


/*-------------------------------PACK2顺序-------------------------------------*/





/*发送给客户端PACK2图片组*/

void UI_TX_ext_pack2(void)
{
	/*---------先保存子信息---------*/
	char magz_state = 0, top_state = 0;
	if(State.rc_flag.magz_state == OPEN)
		magz_state = 1;
	if(State.chassic_flag.now_state == SPIN)	
		top_state = 1;
	
	UI_Save_Aim(UI_Data.operate.pack2);
	UI_Save_Cap_Data(Super_2023.RX.cap_Ucr, UI_Data.operate.pack2);
	UI_Save_Magz_Data(magz_state, UI_Data.operate.pack2);
	UI_Save_Top_Data(top_state, UI_Data.operate.pack2);
	
	
	//固定头数据,前13个字节
	memcpy(&UI_Data.ext.pack2.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.pack2.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																								 sizeof(graphic_data_struct_t)*7;
	
	UI_Data.ext.pack2.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;	
	
	//头校验CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.pack2.Tx_FrameHeader.sof, sizeof(ext_graphic_seven_photo_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_graphic_seven_photo_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_graphic_seven_photo_t));
	
	
}
/*准心*/
void UI_Save_Aim(const uint8_t operate_type)
{
	/*竖直线*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[0], "AI0", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 960, 600, 0, 960, 450);
	
	/*第1条横线*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[1], "AI1", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 800, 520, 0, 960-800+960, 520);
	
	/*第2条横线*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[2], "AI2", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 850, 500, 0, 960-850+960, 500);
	
	/*第3条横线*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[3], "AI3", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 850, 480, 0, 960-850+960, 480);
	
}


/*以粗直线能量条的形式去显示超电的剩余能量*/
void UI_Save_Cap_Data(const float voltage, const uint8_t operate_type)
{
	float X_End;
	uint8_t  color;
	
	UI_Data.data.cap_power_percent = voltage / CAP_MAX_VOLTAGE;
	
	X_End = Cap_Energy_Bar_X_Start+voltage*voltage/2.f;
	
	if(voltage<18)
		color = Cap_Energy_Bar_Warn_Color;
	else
		color = Cap_Energy_Bar_Color;
	
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[4], 
							"CD1", 
							operate_type, 
							LINE,
							2,
							color,
							0,
							0,
							Cap_Energy_Bar_Width,
							Cap_Energy_Bar_X_Start,
							Cap_Energy_Bar_Y_Start,
							0,
							(uint32_t)X_End,
							Cap_Energy_Bar_Y_End);
			
}

/*以正方形的形式去显示弹仓是否打开*/
void UI_Save_Magz_Data(const char state, const uint8_t operate_type)
{
	if(state == 1)
	{
		UI_Data.data.magz_state = Open;
		Draw_Firgue(&UI_Data.ext.pack2.Client_Data[5], "DC1", operate_type, RECTANGLE, 2, YELLOW,\
							 0, 0, 5, 500, 350, 0, 600, 450);
	}
		
	else
	{
		UI_Data.data.magz_state = Close;
		Draw_Firgue(&UI_Data.ext.pack2.Client_Data[5], "DC1", operate_type, RECTANGLE, 2, YELLOW,\
							 0, 0, 5, 0, 0, 0, 0, 0);//正方形特别小，相当于没显示
	}

}

/*以圆形显示陀螺是否打开*/
void UI_Save_Top_Data(const char state, const uint8_t operate_type)
{
	if(state == 1)
	{
		UI_Data.data.small_top_state = Open;
		Draw_Firgue(&UI_Data.ext.pack2.Client_Data[6], "TL1", operate_type, CIRCLE, 2, YELLOW,\
							 0, 0, 4, 250, 700, 80, 0, 0);
	}
	else
	{
		UI_Data.data.small_top_state = Close;
		Draw_Firgue(&UI_Data.ext.pack2.Client_Data[6], "TL1", operate_type, CIRCLE, 2, YELLOW,\
							 0, 0, 4, 0, 0, 0, 0, 0); //圆形特别小，相当于没显示
	}
		
}


