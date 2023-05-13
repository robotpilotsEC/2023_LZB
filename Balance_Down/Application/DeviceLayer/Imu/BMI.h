#ifndef __BMI_H
#define __BMI_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "bmi2.h"
#include "bmi270.h"
#include "common.h"
#include "arm_math.h"

/* Exported macro ------------------------------------------------------------*/
#define ACCD_X_LSB 0x0c
#define ACCD_X_MSB 0x0d
#define ACCD_Y_LSB 0x0e
#define ACCD_Y_MSB 0x0f
#define ACCD_Z_LSB 0x10
#define ACCD_Z_MSB 0x11
#define GYR_X_LSB 0x12
#define GYR_X_MSB 0x13
#define GYR_Y_LSB 0x14
#define GYR_Y_MSB 0x15
#define GYR_Z_LSB 0x16
#define GYR_Z_MSB 0x17


/* Exported types ------------------------------------------------------------*/
typedef struct bmi_struct {

	struct bmi2_dev *dev;
	
	uint8_t drive_type;
	
	int8_t (*init)(struct bmi2_dev *bmi,uint8_t intf);
	
	int8_t (*read)(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi2_dev *dev);
	int8_t (*write)(uint8_t reg_addr, const uint8_t *data, uint16_t len, struct bmi2_dev *dev);

} bmi_t;

extern SPI_HandleTypeDef hspi2;


/* Exported functions --------------------------------------------------------*/
int8_t bmi_init(struct bmi2_dev *bmi2_dev,uint8_t intf);
void BMI_Get_RawData(int16_t *ggx, int16_t *ggy, int16_t *ggz, int16_t *aax, int16_t *aay, int16_t *aaz);
uint8_t BMI_Get_EulerAngle(float *pitch,float *roll,float *yaw,\
                           float *pitch_,float *roll_,float *yaw_,\
													 float *ggx,float *ggy,float *ggz,\
													 float *aax,float *aay,float *aaz);
void BMI_Get_Acceleration(float pitch, float roll, float yaw,\
             float ax, float ay, float az,\
             float *accx, float *accy, float *accz);




#endif


