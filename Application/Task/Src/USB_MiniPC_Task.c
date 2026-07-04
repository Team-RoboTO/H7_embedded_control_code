/**
  ******************************************************************************
  * @file    USB_MiniPC_Task.c
  * @brief   Jetson USB CDC communication task (ARC RMUL protocol)
  *          Sends GimbalToVision (43 B) at 1 kHz; VisionToGimbal (35 B)
  *          is parsed in the USB IRQ (see MiniPC.c)
  ******************************************************************************
  */

#include "USB_MiniPC_Task.h"
#include "MiniPC.h"

  /************/
 /*   TASK   */
/************/

void USB_MiniPC_Task(void const *pvParameters)
{
		const TickType_t xPeriod = 1 / portTICK_PERIOD_MS;
	
		for (;;)
    {
        MiniPC_Transmit_Info();
			  vTaskDelay(xPeriod);
    }
}
