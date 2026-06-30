/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : MiniPC interfaces functions
  * @author         : GarssFan Wang (original), adapted for H723 USB CDC
  * @date           : 2025/01/22
  * @version        : v2.0
  ******************************************************************************
  * @attention      : Replaces UART+DMA (F407) with USB CDC (H723)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "MiniPC.h"
#include "usbd_cdc_if.h"
#include "Referee_System.h"
#include "gimbal_control.h"
#include "Cubemars_Motor.h"
#include "Chassis_control.h"
#include "buzzer.h"

/* Private variables ---------------------------------------------------------*/

/* Binary buffers for USB transmission */
static uint8_t Tx_miniPC_binary_data[NUM_BYTES_TX_MINIPC];

/* Received float values, accessible via macros defined in MiniPC.h */
float Rx_miniPC_fp32_data[NUM_FP32_RX_MINIPC] = {0};
float Tx_data[NUM_FP32_TX_MINIPC] = {0};

static bool first_message = 1;

/**
  * @brief  Pack referee data into float array
  * @param  Tx_data: pointer to array of NUM_FP32_TX_MINIPC floats
  */
 void MiniPC_Prepare_Tx_Data(float *Tx_data)
{
	
    Tx_data[0] = gimbal.x[0];    																											/* radians */
		#if IS_STD || IS_SENTRY
			Tx_data[1] = CM_Pitch_Motor.Data.Position*45.0f/25.0f;  													/* float, degrees */
	  #elif IS_HERO
			Tx_data[1] = CM_Pitch_Motor.Data.Position;
		#endif
	
	  Tx_data[2] = vx;   																																/* linear velocity  [float, m/s]   */
    Tx_data[3] = vy;   																																/* angular velocity [float, rad/s] */
		
		Tx_data[4] = (float)(Referee_System_Info.robot_status.robot_id > 100);             /* color: 0=RED, 1=BLUE           */  
		Tx_data[5] = (float)(Referee_System_Info.game_status.game_progress);         		  /* 1: Preparation Period
																																											 * 2: 15-Second Referee System Initialization Period
																																											 * 3: 5-Second Countdown
																																											 * 4: In Match
																																											 * 5: Match Settling      */
	
    Tx_data[6] = (float) Referee_System_Info.robot_status.current_HP;                  /* HP: uint16_t cast to fp32          */
    Tx_data[7] = (float)((Referee_System_Info.rfid_status.rfid_status >> 19) & 0x1);   /* resupply zone: bit 19              */
    Tx_data[8] = (float)((Referee_System_Info.rfid_status.rfid_status >> 23) & 0x1);   /* center: bit 23                     */
 
		Tx_data[9] = 0.0f;
}

/**
  * @brief  Serialize float array to bytes and send over USB CDC
  * @note   Call from a task.
  *         Returns immediately if USB is busy (USBD_BUSY).
  */
void MiniPC_Transmit_Info(void)
{
    // Fill float array from referee system
    MiniPC_Prepare_Tx_Data(Tx_data);

    // Serialize: fp32 array -> raw bytes, little-endian 
    for (uint8_t i = 0; i < NUM_FP32_TX_MINIPC; i++)
    {
        memcpy(&Tx_miniPC_binary_data[i * sizeof(float)], &Tx_data[i], sizeof(float));
    }

    CDC_Transmit_HS(Tx_miniPC_binary_data, NUM_BYTES_TX_MINIPC); 
}


/**
  * @brief  Deserialize received bytes into Rx_miniPC_fp32_data[]
  * @param  Buff: raw byte buffer from USB CDC
  * @param  Len:  number of bytes received
  * @note   Access received values via macros: yaw_cv, pitch_cv, etc.
  */
void MiniPC_Receive_Info(uint8_t *Buff, uint32_t Len)
{
		/* Deserialize: raw bytes -> float array, little-endian */
    for (uint8_t i = 0; i < NUM_FP32_RX_MINIPC; i++)
    {
        memcpy(&Rx_miniPC_fp32_data[i], &Buff[i * sizeof(float)], sizeof(float));	
    }
		
}
