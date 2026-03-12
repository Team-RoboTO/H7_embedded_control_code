/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : MiniPC interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/02/10
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H

/* Includes --------------------------------------------------------------------*/
#include "struct_typedef.h"
#include <string.h>
#include "main.h"

/* ============================================================
   PROTOCOL DIMENSIONS
   ============================================================ */
#define NUM_FP32_TX_MINIPC      6
#define NUM_FP32_RX_MINIPC      6
#define NUM_BYTES_TX_MINIPC     (NUM_FP32_TX_MINIPC * sizeof(fp32))   /* 24 bytes */
#define NUM_BYTES_RX_MINIPC     (NUM_FP32_RX_MINIPC * sizeof(fp32))   /* 24 bytes */

/* ============================================================
   RX DATA — convenient named access to received float values
   ============================================================ */
extern fp32 Rx_miniPC_fp32_data[NUM_FP32_RX_MINIPC];

#define yaw_cv              Rx_miniPC_fp32_data[0]
#define pitch_cv            Rx_miniPC_fp32_data[1]
#define shoot_frequency_cv  Rx_miniPC_fp32_data[2]
#define fwd_bwd_cv          Rx_miniPC_fp32_data[3]
#define left_right_cv       Rx_miniPC_fp32_data[4]
#define angle_cv            Rx_miniPC_fp32_data[5]
/* ============================================================
   FUNCTIONS
   ============================================================ */

/**
  * @brief  Pack referee data and transmit to MiniPC over USB CDC.
  *         Call periodically from a task (NOT from an ISR).
  */
extern void MiniPC_Transmit_Info(void);

/**
  * @brief  Deserialize a received USB CDC packet into Rx_miniPC_fp32_data[].
  *         Called internally by CDC_Receive_HS in usbd_cdc_if.c.
  */
extern void MiniPC_Receive_Info(uint8_t *Buff, uint32_t Len);

#endif /* MINIPC_H */