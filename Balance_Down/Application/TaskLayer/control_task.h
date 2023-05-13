#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "module.h"
#include "offline.h"

void StartControlTask(void const * argument);
void Start_Super_Task(void const * argument);
#endif


//陀螺仪没数据时，去找函数
//bmi2_write_config_file
