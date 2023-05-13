#include "UI_Task.h"

void Start_UI_Task(void const * argument)
{
	
	
  for(;;)
  {
		
		UI_Ctrl();	
		
    osDelay(1);
  }

}
