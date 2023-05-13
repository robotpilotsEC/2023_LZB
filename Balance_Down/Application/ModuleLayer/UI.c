#include "UI.h"

/*----------˽�б�������----------*/
extern UART_HandleTypeDef huart4;
UI_Data_t 	UI_Data;
uint8_t  Tx_Client_Buf[200];

/*-------------------------------�ϲ�-------------------------------*/
void UI_Ctrl(void);

/*-------------------------------Ӧ��-------------------------------*/
void UI_Data_ID_UP(void);
void UI_Prepare(void);
void Save_Tx_Pack_Fixed_Info(void);

/*-------------------------------�ײ�--------------------------------*/
void UI_Send_Data_Pack( uint8_t* txbuf, const uint8_t len);




/*-------------------------------PACK1˳��-------------------------------------*/
void UI_TX_ext_pack1(void);
void UI_Save_Chas_Vision_Wide(const uint8_t operate_type);
void UI_Save_Chas_Vision_Length(const uint8_t operate_type);
void UI_Save_Chas_State(const char state, const uint8_t operate_type);

/*-------------------------------PACK2˳��-------------------------------------*/
void UI_TX_ext_pack2(void);
void UI_Save_Aim(const uint8_t operate_type);
void UI_Save_Cap_Data(const float voltage, const uint8_t operate_type);
void UI_Save_Magz_Data(const char state, const uint8_t operate_type);
void UI_Save_Top_Data(const char state, const uint8_t operate_type);






/**	
	*@brief  Tx_Client_Tmp_Buf�Ĵ�С�����������ĳ���,��ŷ�������ϵͳ���ݵ�ǰ�̶�����
	std_frame_header_t 											Tx_FrameHeader;	 		 5�ֽ�
	uint16_t  															CmdID;							 2�ֽ�
		
	ext_student_interactive_data_header_t   Data_FrameHeader;		 6�ֽ�
*/
uint8_t  Tx_Client_Tmp_Buf[INTERACT_DATA_FIXED_LEN];


/*--------------------------------ͨ�����ڷ���һ֡����-----------------------------------*/
void UI_Send_Data_Pack( uint8_t* txbuf, const uint8_t len)
{
	if(txbuf == NULL)
		return;

	HAL_UART_Transmit_DMA(&huart4, txbuf, len);
	
}

/*-----------------------------------��ʾָ��������,�ײ㺯��----------------------------------------*/



void Draw_String_Or_Char(ext_client_custom_character_t*  graphic,  //��ͼ�ε��ص���Ϣ�ŵ�����
												 
												 const char* name,     //��Ϊ�ͻ��˵�����
													 
									       uint32_t operate_type, //ͼ�β���
												 uint32_t layer,				//ͼ�β���
												 uint32_t color,				//ͼ����ɫ
												 uint32_t size,					//�ַ���С						 
												 uint32_t length,				//�ַ����� 
												 uint32_t width,				//�߿�
												 uint32_t start_x,			//X����
												 uint32_t start_y,			//Y����						 
												 
												 const char *string)		//Ҫ��ʾ���ַ�
{

	if(graphic == NULL || name == NULL || string == NULL)
		return;
	
	graphic_data_struct_t 	*grapic_info = &graphic->grapic_info;
	
	for(char i = 0 ; i < 3; i ++)
		grapic_info->graphic_name[i] = name[i];	
	grapic_info->operate_type = operate_type;
	grapic_info->graphic_type = CHAR;        //Char����ֱ�Ӹ�ֵ
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
		graphic->graphic_name[i] = name[i];	//�ַ�����
	
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
												 uint32_t decimal,	//С��������Чλ��, 0����ʾint32_t����
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




/*-------------------------------�ܿ���---------------------------------------------*/
uint16_t i, j;
void UI_Ctrl(void)
{
	UI_Prepare();
	
	i++;	
	
	//ǰ10ms������pack1
	if(i <= 30)
	{
		UI_TX_ext_pack1();
	}
	//��10ms������pack2
	else
	{ 
		if(i >= 90)
		{
			i = 0;
			j ++;
		}	
		
		UI_TX_ext_pack2();
	}
	
	//j��ʾ���˶��ٴ�
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










/*--------------------------------Ӧ�ò�-----------------------------------------------------*/




void UI_Prepare(void)
{
	Save_Tx_Pack_Fixed_Info();
	UI_Data_ID_UP();
}
/*id����*/
void UI_Data_ID_UP(void)
{
	UI_Data.robot_id = judge_info.game_robot_status.robot_id;
	UI_Data.client_id = judge_info.self_client;
	
}
/*���͸�����ϵͳ������,��ǰ��һ�����ݻ����ǹ̶���*/
//��������ݳ���,CRC8,���ݶ�ͷ�ṹ������������
void Save_Tx_Pack_Fixed_Info(void)
{

	//֡ͷ
	Tx_Client_Tmp_Buf[0] = JUDGE_FRAME_HEADER;
	//���ݳ���
	Tx_Client_Tmp_Buf[1] = Tx_Client_Tmp_Buf[2] = 0;
	//�����
	Tx_Client_Tmp_Buf[3] = 0;
	//CRC8
	Tx_Client_Tmp_Buf[4] = 0;
	
	
	
	//������
	Tx_Client_Tmp_Buf[5] = (uint8_t)ID_robot_interactive_header_data;
	Tx_Client_Tmp_Buf[6] = (uint8_t)(ID_robot_interactive_header_data>>8);
	
	
	
	
	
	//���ݶ�ͷ�ṹ������������
	Tx_Client_Tmp_Buf[7] = 0;
	Tx_Client_Tmp_Buf[8] = 0;
	//���ݶ�ͷ�ṹ�Ļ�����ID
	Tx_Client_Tmp_Buf[9] =  (uint8_t)UI_Data.robot_id;
	Tx_Client_Tmp_Buf[10] = (uint8_t)(UI_Data.robot_id>>8);
	//���ݶ�ͷ�ṹ�Ŀͻ���ID
	Tx_Client_Tmp_Buf[11] = (uint8_t)UI_Data.client_id;
	Tx_Client_Tmp_Buf[12] = (uint8_t)(UI_Data.client_id>>8);
	
	
}















/*-------------------------------PACK1˳��-------------------------------------*/





/*���͸��ͻ���PACK1ͼƬ��*/

void UI_TX_ext_pack1(void)
{
	/*---------�ȱ�������Ϣ---------*/
	char sideways_state = 0;
	if(State.chassic_flag.sideways_state == SIDEWAYS_ON)
		sideways_state = 1;
	
	UI_Save_Chas_Vision_Wide(UI_Data.operate.pack1);
	UI_Save_Chas_Vision_Length(UI_Data.operate.pack1);
	UI_Save_Chas_State(sideways_state, UI_Data.operate.pack1);
	
	
	//�̶�ͷ����,ǰ13���ֽ�
	memcpy(&UI_Data.ext.pack1.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.pack1.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																								 sizeof(graphic_data_struct_t)*5;
	
	UI_Data.ext.pack1.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_five_graphic;	
	
	//ͷУ��CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.pack1.Tx_FrameHeader.sof, sizeof(ext_graphic_five_photo_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_graphic_five_photo_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_graphic_five_photo_t));
	
	
}
/*��ֱ����ʽ��ʾ������Ұ�Ŀ�*/
void UI_Save_Chas_Vision_Wide(const uint8_t operate_type)
{
	/*����*/
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

	/*�ҿ��*/
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

/*��ֱ����ʽ��ʾ������Ұ�ĳ�*/
void UI_Save_Chas_Vision_Length(const uint8_t operate_type)
{
	/*�ϳ���*/
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

	/*�³���*/
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
/*��ֱ�ߵ���ʽȥ��ʾ���̵ķ�λ*/

void UI_Save_Chas_State(const char state, const uint8_t operate_type)
{
	if(state == 1)//������������
	{
		UI_Data.data.sideways_state = Open;
		Draw_Firgue(&UI_Data.ext.pack1.Client_Data[4], "CS1", operate_type, LINE, 1, YELLOW, 0, 0, \
								5, 550, 500, 0, 550, 720);
	}
	else //û�����ú���
	{
		UI_Data.data.sideways_state = Close;
		Draw_Firgue(&UI_Data.ext.pack1.Client_Data[4], "CS1", operate_type, LINE, 1, YELLOW, 0, 0, \
								5, 480, 600, 0, 620, 600 );
	}
	
}


/*-------------------------------PACK2˳��-------------------------------------*/





/*���͸��ͻ���PACK2ͼƬ��*/

void UI_TX_ext_pack2(void)
{
	/*---------�ȱ�������Ϣ---------*/
	char magz_state = 0, top_state = 0;
	if(State.rc_flag.magz_state == OPEN)
		magz_state = 1;
	if(State.chassic_flag.now_state == SPIN)	
		top_state = 1;
	
	UI_Save_Aim(UI_Data.operate.pack2);
	UI_Save_Cap_Data(Super_2023.RX.cap_Ucr, UI_Data.operate.pack2);
	UI_Save_Magz_Data(magz_state, UI_Data.operate.pack2);
	UI_Save_Top_Data(top_state, UI_Data.operate.pack2);
	
	
	//�̶�ͷ����,ǰ13���ֽ�
	memcpy(&UI_Data.ext.pack2.Tx_FrameHeader.sof, \
				 Tx_Client_Tmp_Buf, \
				 INTERACT_DATA_FIXED_LEN);
	
	UI_Data.ext.pack2.Tx_FrameHeader.data_length = sizeof(ext_student_interactive_data_header_t) + \
																								 sizeof(graphic_data_struct_t)*7;
	
	UI_Data.ext.pack2.Data_FrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;	
	
	//ͷУ��CRC8
	memcpy(Tx_Client_Buf, &UI_Data.ext.pack2.Tx_FrameHeader.sof, sizeof(ext_graphic_seven_photo_t));

	Append_CRC8_Check_Sum(Tx_Client_Buf, sizeof(std_frame_header_t));
	
	Append_CRC16_Check_Sum(Tx_Client_Buf,sizeof(ext_graphic_seven_photo_t));
		
	UI_Send_Data_Pack(Tx_Client_Buf, sizeof(ext_graphic_seven_photo_t));
	
	
}
/*׼��*/
void UI_Save_Aim(const uint8_t operate_type)
{
	/*��ֱ��*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[0], "AI0", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 960, 600, 0, 960, 450);
	
	/*��1������*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[1], "AI1", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 800, 520, 0, 960-800+960, 520);
	
	/*��2������*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[2], "AI2", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 850, 500, 0, 960-850+960, 500);
	
	/*��3������*/
	Draw_Firgue(&UI_Data.ext.pack2.Client_Data[3], "AI3", operate_type, LINE, 2, \
							Aim_Color, 0,	0, Aim_Width, 850, 480, 0, 960-850+960, 480);
	
}


/*�Դ�ֱ������������ʽȥ��ʾ�����ʣ������*/
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

/*�������ε���ʽȥ��ʾ�����Ƿ��*/
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
							 0, 0, 5, 0, 0, 0, 0, 0);//�������ر�С���൱��û��ʾ
	}

}

/*��Բ����ʾ�����Ƿ��*/
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
							 0, 0, 4, 0, 0, 0, 0, 0); //Բ���ر�С���൱��û��ʾ
	}
		
}


