/**
  ******************************************************************************
  * @file    USB_MiniPC_Task.c
  * @brief   MiniPC USB CDC communication task
  *          Drop-in replacement for USART_MiniPC.c (F407 UART+DMA)
  *          Adapted for STM32H723 USB CDC
  ******************************************************************************
  */

#include "USB_MiniPC_Task.h"
#include "MiniPC.h"
#include "string.h"
#include "Referee_System.h"
#include "INS_task.h"
#include "math_utils.h"
#include "Robot_config.h" 



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
