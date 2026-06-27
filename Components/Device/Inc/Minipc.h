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
#include "stdint.h"

/* ============================================================
   PROTOCOL DIMENSIONS
   ============================================================ */
#define NUM_FP32_TX_MINIPC      10
#define NUM_FP32_RX_MINIPC      7
#define NUM_BYTES_TX_MINIPC     (NUM_FP32_TX_MINIPC * sizeof(float))   /* 40 bytes */
#define NUM_BYTES_RX_MINIPC     (NUM_FP32_RX_MINIPC * sizeof(float))   /* 24 bytes */

/* ============================================================
   RX DATA — convenient named access to received float values
   ============================================================ */

#define time_cv       			Rx_miniPC_fp32_data[0]
#define yaw_cv              Rx_miniPC_fp32_data[1]
#define pitch_cv            Rx_miniPC_fp32_data[2]
#define shoot_flag_cv       Rx_miniPC_fp32_data[3]
#define fwd_bwd_cv          Rx_miniPC_fp32_data[4]
#define left_right_cv       Rx_miniPC_fp32_data[5]
#define flag_rot            Rx_miniPC_fp32_data[6]

/* ============================================================
   FUNCTIONS
   ============================================================ */
extern float Rx_miniPC_fp32_data[NUM_FP32_RX_MINIPC];
void MiniPC_Transmit_Info(void);
void MiniPC_Receive_Info(uint8_t *Buff, uint32_t Len);

#endif /* MINIPC_H */