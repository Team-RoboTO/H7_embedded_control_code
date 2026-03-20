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
#include "logic_utils.h"
#include "INS_task.h"
#include "math_utils.c"
/* To be added from old code
#include "robot_config.h" */

  /********************/
 /*   SAMPLE TIMES   */
/********************/

fp32 dt_usb_minipc;
uint32_t dt_usb_minipc_ms;

static uint32_t dt_usb_minipc_std_circ_ms = 10;
static uint32_t dt_usb_minipc_std_rect_ms = 10;
static uint32_t dt_usb_minipc_sentry_ms   = 10;
static uint32_t dt_usb_minipc_hero_ms     = 10;

static uint32_t usb_minipc_task_start_timestamp_ms;
static uint32_t usb_minipc_task_end_timestamp_ms;
static uint32_t usb_minipc_task_elapsed_time_ms;
static uint32_t usb_minipc_task_sleep_time_ms;

  /************/
 /*   TASK   */
/************/

void USB_MiniPC_Task(void const *pvParameters)
{
    /* Set task period based on robot type */
	/*
#if IS_STD_CIRC
    dt_usb_minipc_ms = dt_usb_minipc_std_circ_ms;
#elif IS_STD_RECT
    dt_usb_minipc_ms = dt_usb_minipc_std_rect_ms;
#elif IS_SENTRY
    dt_usb_minipc_ms = dt_usb_minipc_sentry_ms;
#elif IS_HERO
    dt_usb_minipc_ms = dt_usb_minipc_hero_ms;
#endif*/
		dt_usb_minipc_ms = 1; //to remove after test
	
    dt_usb_minipc = SEC(dt_usb_minipc_ms);

    /* NOTE: No init needed here — USB is initialized in MX_USB_DEVICE_Init()
       which is called from freertos() before the scheduler starts. */

    /* Task loop - periodic TX every dt_usb_minipc_ms */
    while (TRUE)
    {
        usb_minipc_task_start_timestamp_ms = HAL_GetTick();

        /* Pack referee data and transmit 24 bytes to MiniPC over USB CDC.
           RX is handled automatically via CDC_Receive_HS callback -> MiniPC_Receive_Info(). */
        MiniPC_Transmit_Info();

        /* Compute delay until next task iteration */
        usb_minipc_task_end_timestamp_ms    = HAL_GetTick();
        usb_minipc_task_elapsed_time_ms     = usb_minipc_task_end_timestamp_ms - usb_minipc_task_start_timestamp_ms;
        usb_minipc_task_sleep_time_ms       = max(dt_usb_minipc_ms - usb_minipc_task_elapsed_time_ms, (uint32_t) 0);

        //osDelay(usb_minipc_task_sleep_time_ms);
    }
}
