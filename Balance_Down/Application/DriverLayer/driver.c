/**
 * @file        driver.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        9-September-2020
 * @brief       Drivers' Manager.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRIVER_Init(void)
{

	SPI2_Init();
	imu_sensor.init(&imu_sensor);
	
	HAL_Delay(10);
	USART1_Init();
	USART2_Init();
	USART3_Init();
	USART4_Init();
	USART5_Init();
	
	if(imu_sensor.info->init_flag)TIM4_Init();

	CAN_Filter_Init();
}
