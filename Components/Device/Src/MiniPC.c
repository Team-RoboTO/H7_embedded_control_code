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
#include "INS_Task.h"
#include "Referee_System.h"
#include "gimbal_control.h"
#include "Cubemars_Motor.h"
#include <string.h>

fp32 random_value = 0;


/* Private variables ---------------------------------------------------------*/

/* Binary buffers for USB transmission */
static uint8_t Tx_miniPC_binary_data[NUM_BYTES_TX_MINIPC];

/* Received float values, accessible via macros defined in MiniPC.h */
fp32 Rx_miniPC_fp32_data[NUM_FP32_RX_MINIPC] = {0}; //
uint32_t rx_count = 0;
uint32_t tx_count = 0;

fp32 Tx_data[NUM_FP32_TX_MINIPC] = {0};

fp32 fake_data[10] = {0};

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
   
    /* --- Referee data --- */
//		Tx_data[0] = (fp32)(Referee_System_Info.robot_status.robot_id > 100);            /* color: 0.0=RED, 1.0=BLUE           */  
//		Tx_data[1] = (fp32)(Referee_System_Info.game_status.game_progress == 4);         /* battle mode: 1.0 if in battle      */
//    Tx_data[2] = (fp32) Referee_System_Info.robot_status.current_HP;                 /* HP: uint16_t cast to fp32          */
//    Tx_data[3] = (fp32)((Referee_System_Info.rfid_status.rfid_status >> 19) & 0x1);  /* resupply zone: bit 19              */
//    Tx_data[4] = (fp32)((Referee_System_Info.event_data.event_data   >>  5) & 0x3);  /* central highland: bits 5-6 (0/1/2) */
// 
	
	
//		Tx_data[0] = (fp32)(Referee_System_Info.robot_status.robot_id > 100);            /* color: 0.0=RED, 1.0=BLUE           */  
//		Tx_data[1] = fake_data[1];         /* battle mode: 1.0 if in battle      */
    Tx_data[0] = gimbal.x[0];    /* float, degrees */
    Tx_data[1] = CM_Pitch_Motor.Data.Position*45.0f/25.0f;  /* float, degrees */
	  Tx_data[2] = fake_data[2];                 /* HP: uint16_t cast to fp32          */
    Tx_data[3] = fake_data[3];  /* resupply zone: bit 19              */
    Tx_data[4] = fake_data[4];  /* central highland: bits 5-6 (0/1/2) */
 
    /* --- IMU data from INS_Task --- */
    Tx_data[5] = INS_Info.Yaw_TolAngle ;  /* float, degrees */
    Tx_data[6] = CM_Pitch_Motor.Data.Position;  /* float, degrees */
    Tx_data[7] = INS_Info.Roll_Angle;   /* float, degrees */
 
    /* --- Chassis velocities (TODO: replace when chassis task is ready) --- */
    Tx_data[8] = fake_data[8];   /* linear velocity  [fp32, m/s]   */
    Tx_data[9] = fake_data[9];   /* angular velocity [fp32, rad/s] */
	
	//Dummy data 
		/*
		Tx_data[0] = random_value;
		Tx_data[1] = random_value-1;
		Tx_data[2] = -random_value;
		Tx_data[3] = random_value;
		Tx_data[4] = rx_count;
		Tx_data[5] = tx_count;
		*/
}

/**
  * @brief  Serialize float array to bytes and send over USB CDC
  * @note   Call from a task.
  *         Returns immediately if USB is busy (USBD_BUSY).
  */
void MiniPC_Transmit_Info(void)
{
	//Begin transmitting only once it has received something (from python test script)
	if(rx_count>0){
    tx_count++;
	}
		//random_value++;

    // Fill float array from referee system
    MiniPC_Prepare_Tx_Data(Tx_data, random_value);

    // Serialize: fp32 array -> raw bytes, little-endian 
    for (uint8_t i = 0; i < NUM_FP32_TX_MINIPC; i++)
    {
        memcpy(&Tx_miniPC_binary_data[i * sizeof(fp32)], &Tx_data[i], sizeof(fp32));
    }

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
