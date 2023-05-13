#ifndef __HEARTBEAT_TASK
#define __HEARTBEAT_TASK

#include "cmsis_os.h"
#include "main.h"
#include "device.h"
#include "module.h"
#include "LED.h"
void StartHeartBeatTask(void const * argument);

#endif
