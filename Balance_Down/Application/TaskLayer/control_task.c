#include "control_task.h"

void StartControlTask(void const * argument)
{
	
  for(;;)
  {
		#ifdef NO_LINK_TO_RECEIVE	
		{
			Master.ctrl();
		}
		#endif	
		
		State.ctrl();
		
		chassic.ctrl();
		
		Offline_Handler();
		
    osDelay(1);
		
  }
}

void Start_Super_Task(void const * argument)
{
	for(;;)
  {
    
		Super_2023.ctrl();
		
		osDelay(4);
		
  }
}
