#include "Under_Layer.h"
/*Private variables*/
device_info_t  					 Device_Info;
sys_status_pack_info_t   Chas_Sys_Status;

/*Exported variables*/

/*Privated Functions*/
void Save_Device_Info(void);


/**
	*@brief  更新用户定义的目标包、数据包、误差包3个
	*@note   默认user_status_t的每个数据都是float类型
	*@param[in] status为包括了用户定义的目标包、数据包、误差包的指针
	*@return NULL
*/
//void Sys_Info_UP(user_status_pack_info_t* status)									 
//{


//}


///**
//	*@brief 	更新用户定义的数据包,可为目标包、测量包、误差包
//	*@note  	默认user_status_t的每个数据所占内存的大小一致
//	*@param[in] 	status为指向用户定义的数据包的指针
//	*@param[in] 	param_num为可变参数列表的参数个数
//	*@return 	NULL
// */
//void User_Status_Info_UP(user_status_t* status, const uint8_t param_num, ...)
//{
//	if(status == NULL)	return;
//	
//	va_list valist;
//	va_start(valist, param_num);
//	float* status_p = &status->C_X_L;
//	
//	for(uint8_t i = 0; i < param_num; i ++)
//	{
//		status_p[i] = va_arg(valist, float);
//	}	
//	
//	va_end(valist);
//	
//}
/**
	*@brief 	保存设备提供的数据
	*@return 	NULL
 */
void Save_Device_Info(void)
{
	
}
