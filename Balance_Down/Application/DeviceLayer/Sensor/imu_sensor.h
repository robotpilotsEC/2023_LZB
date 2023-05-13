#ifndef __IMU_H
#define __IMU_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "BMI.h"
#include "drv_tick.h"

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	IMU_NONE_ERR,
  IMU_TYPE_ERR,
  IMU_ID_ERR,
  IMU_INIT_ERR,
  IMU_DATA_ERR,
} imu_err_e;

typedef enum{
	DR_SPI1,
	DR_SPI2,
	DR_SPI3,
	
	DR_IIC,
	
}drv_type_e;

typedef struct drive_str{
	drv_type_e tpye;
	
	int8_t (*send)(struct drive_str *self, uint8_t *Txbuff, uint16_t len);
	int8_t (*read)(struct drive_str *self, uint8_t *Rxbuff, uint16_t len);
	int8_t (*sendread)(struct drive_str *self, uint8_t *Txbuff, uint8_t *Rxbuff, uint16_t len);
}driver_t;

typedef struct work_state_struct {
	dev_work_state_e dev_state;
	imu_err_e	       err_code;
	
	uint8_t   offline_cnt;
	uint8_t   offline_max_cnt;
	
} work_state_t;



typedef struct{
	short acc_x;
	short acc_y;
	short acc_z;
	
	short gyro_x;
	short gyro_y;
	short gyro_z;	

}	raw_info_t;

typedef struct{
	float accx;
	float accy;
	float accz;	
	
	float yaw;
	float pitch;
	float roll;
	
	float rate_yaw;
	float rate_pitch;
	float rate_roll;	
	
	float ave_rate_yaw;
	float ave_rate_pitch;	
	float ave_rate_roll;

	float g;
	
}	base_info_t;

typedef struct imu_info_struct {

	raw_info_t  raw_info;

	base_info_t base_info;
	
	uint8_t		  init_flag;

	int         off_ground_cnt; //落地计数值
	uint8_t     off_ground_flag; //0表示不离地，1表示离地下降
	
} imu_info_t;

typedef struct imu_struct {
	
  bmi_t       *bmi;	
	imu_info_t	*info;
	driver_t	  driver;
	
	void				(*init)(struct imu_struct *self);
	void				(*update)(struct imu_struct *self);
	void				(*heart_beat)(struct work_state_struct *self);
	
	work_state_t  work_state;
	dev_id_t		   id;	
} imu_sensor_t;

extern imu_sensor_t imu_sensor;

/* Exported functions --------------------------------------------------------*/
void Get_IMU_Data(void);
void TIM4_Init_Handle(uint16_t timeout);
void BMI_DATA(void);


#endif


