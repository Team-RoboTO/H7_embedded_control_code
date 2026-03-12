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

/*To be added from old code
#include "referee.h"*/
#include <string.h>

fp32 random_value = 0;
/* ============================================================
   PROTOCOL SUMMARY
   TX (STM32 -> MiniPC): 6 x fp32 = 24 bytes
   +---------+---------+------------------------------------+
   | index   | bytes   | meaning                           |
   +---------+---------+------------------------------------+
   |   0     |  0-3    | Robot Color (RED=0, BLUE=1)       |
   |   1     |  4-7    | Battle mode active (0/1)          |
   |   2     |  8-11   | Current Robot HP                  |
   |   3     | 12-15   | Remaining projectiles             |
   |   4     | 16-19   | Capture point status              |
   |   5     | 20-23   | Is in resupply zone (0/1)         |
   +---------+---------+------------------------------------+

   RX (MiniPC -> STM32): 6 x fp32 = 24 bytes
   +---------+---------+------------------------------------+
   | index   | bytes   | meaning                           |
   +---------+---------+------------------------------------+
   |   0     |  0-3    | yaw_cv                            |
   |   1     |  4-7    | pitch_cv                          |
   |   2     |  8-11   | shoot_frequency_cv                |
   |   3     | 12-15   | fwd_bwd_cv                        |
   |   4     | 16-19   | left_right_cv                     |
   |   5     | 20-23   | angle_cv                          |
   +---------+---------+------------------------------------+
   All values are raw IEEE 754 float, little-endian.
   ============================================================ */

/* Private variables ---------------------------------------------------------*/

/* Binary buffers for USB transmission */
static uint8_t Tx_miniPC_binary_data[NUM_BYTES_TX_MINIPC];

/* Received float values, accessible via macros defined in MiniPC.h */
fp32 Rx_miniPC_fp32_data[NUM_FP32_RX_MINIPC] = {0}; //
uint32_t rx_count = 0;
uint32_t tx_count = 0;

fp32 Tx_data[NUM_FP32_TX_MINIPC] = {0};

/* ============================================================
   TRANSMIT
   Call this periodically (from a FreeRTOS task) to send
   robot state to the MiniPC.
   ============================================================ */

/**
  * @brief  Pack referee data into float array
  * @param  Tx_data: pointer to array of NUM_FP32_TX_MINIPC floats
  */
 void MiniPC_Prepare_Tx_Data(fp32 *Tx_data, fp32 random_value)
{
    /*Tx_data[0] = (fp32) get_Robot_Color();
    Tx_data[1] = (fp32) is_battle_mode();
    Tx_data[2] = (fp32) get_robot_current_HP();
    Tx_data[3] = (fp32) get_projectile_allowance();
    Tx_data[4] = (fp32) get_event_data();
    Tx_data[5] = (fp32) is_in_resupply_zone();*/
	
	//Dummy data
		Tx_data[0] = random_value;
		Tx_data[1] = random_value-1;
		Tx_data[2] = -random_value;
		Tx_data[3] = random_value;
		Tx_data[4] = random_value;
		Tx_data[5] = random_value;
   
}

/**
  * @brief  Serialize float array to bytes and send over USB CDC
  * @note   Call from a task.
  *         Returns immediately if USB is busy (USBD_BUSY).
  */
void MiniPC_Transmit_Info(void)
{
	random_value++;
	if(rx_count>0)
    tx_count++;

    /* Fill float array from referee system */
    MiniPC_Prepare_Tx_Data(Tx_data, random_value);

    /* Serialize: fp32 array -> raw bytes, little-endian */
    for (uint8_t i = 0; i < NUM_FP32_TX_MINIPC; i++)
    {
        memcpy(&Tx_miniPC_binary_data[i * sizeof(fp32)], &Tx_data[i], sizeof(fp32));
    }

    /* Transmit over USB CDC — correct size is NUM_BYTES_TX_MINIPC, not sizeof(ptr) */
    CDC_Transmit_HS(Tx_miniPC_binary_data, NUM_BYTES_TX_MINIPC);
}

/* ============================================================
   RECEIVE
   Called by CDC_Receive_HS in usbd_cdc_if.c when a USB OUT
   packet arrives from the MiniPC.
   ============================================================ */

/**
  * @brief  Deserialize received bytes into Rx_miniPC_fp32_data[]
  * @param  Buff: raw byte buffer from USB CDC
  * @param  Len:  number of bytes received
  * @note   Access received values via macros: yaw_cv, pitch_cv, etc.
  */
void MiniPC_Receive_Info(uint8_t *Buff, uint32_t Len)
{
    rx_count++;

    /* Ignore packets with unexpected length 
    if (Len != NUM_BYTES_RX_MINIPC)
    {
        return;
    }  */

    /* Deserialize: raw bytes -> fp32 array, little-endian */
    for (uint8_t i = 0; i < NUM_FP32_RX_MINIPC; i++)
    {
        memcpy(&Rx_miniPC_fp32_data[i], &Buff[i * sizeof(fp32)], sizeof(fp32));
    }

}
