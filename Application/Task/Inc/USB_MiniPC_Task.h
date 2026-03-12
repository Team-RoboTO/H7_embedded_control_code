/**
  ******************************************************************************
  * @file    USB_MiniPC_Task.h
  * @brief   Header for USB_MiniPC.c
  ******************************************************************************
  */

#ifndef USB_MINIPC_H
#define USB_MINIPC_H

#include "main.h"
#include "struct_typedef.h"
#include "cmsis_os.h"

  /********************/
 /*   SAMPLE TIMES   */
/********************/

extern fp32 dt_usb_minipc;
extern uint32_t dt_usb_minipc_ms;

  /************/
 /*   TASK   */
/************/

void Start_USB_MiniPC(void const *pvParameters);

#endif // USB_MINIPC_H
