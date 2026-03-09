/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control.h"

/**
 * @brief The structure that contains the Information of FDCAN1 and FDCAN2 Receive.
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

/**
 * @brief Debug variables for FDCAN1 TX diagnostics.
 */
volatile HAL_StatusTypeDef fdcan1_tx_status = HAL_OK;
volatile uint32_t fdcan1_protocol_status = 0;
volatile uint32_t fdcan1_tx_error_count = 0;
volatile uint32_t fdcan1_rx_error_count = 0;
volatile uint32_t fdcan1_rx_callback_count = 0;

/**
 * @brief The structure that contains the Information of FDCAN1 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame = {
	.hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN2 Transmit(FDCAN).
 */
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame = {
  .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

/**
 * @brief The structure that contains the Information of FDCAN3 Transmit(CLASSIC_CAN).
 */
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame = {
  .hcan = &hfdcan3,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = FDCAN_DLC_BYTES_8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,
	.Header.MessageMarker = 0,
};

/**
  * @brief  Configures the FDCAN Filter. 
            FDCAN1:CLASSIC_CAN  FDCAN2:FDCAN  FDCAN3:CLASSIC_CAN
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_FilterConfig.FilterIndex = 0;
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_FilterConfig.FilterID1 = 0x000;
  FDCAN1_FilterConfig.FilterID2 = 0x000;
  
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig);
		
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
 
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  
  HAL_FDCAN_Start(&hfdcan1);
 	
	
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	
  FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000;
  FDCAN2_FilterConfig.FilterID2 = 0x00000000;
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan2);
 
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan2,14,14);

  HAL_FDCAN_Start(&hfdcan2);
	
	
//	FDCAN_FilterTypeDef FDCAN3_FilterConfig;
//	
//	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
//  FDCAN3_FilterConfig.FilterIndex = 0;
//  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
//  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
//  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
//  
//	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

//  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

//  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

//  HAL_FDCAN_Start(&hfdcan3);
}

/**
  * @brief  Function to transmit the FDCAN message with debug diagnostics.
  * @param  *FDCAN_TxFrame :the structure that contains the Information of FDCAN
  * @retval None
  */
void USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame){

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
        FDCAN_TxFrame->hcan,
        &FDCAN_TxFrame->Header,
        FDCAN_TxFrame->Data
    );
    
    /* Capture debug info for FDCAN1 */
    /* Capture debug info for FDCAN1 */
    if(FDCAN_TxFrame->hcan == &hfdcan1){
        FDCAN_ProtocolStatusTypeDef psr;
        FDCAN_ErrorCountersTypeDef err_counters;
        
        fdcan1_tx_status = status;
        HAL_FDCAN_GetProtocolStatus(FDCAN_TxFrame->hcan, &psr);
        fdcan1_protocol_status = psr.LastErrorCode;
        
        HAL_FDCAN_GetErrorCounters(FDCAN_TxFrame->hcan, &err_counters);
        fdcan1_tx_error_count = err_counters.TxErrorCnt;
        fdcan1_rx_error_count = err_counters.RxErrorCnt;
    }
}

/**
  * @brief  Function to converting the FDCAN1 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
   
	MIT_motor_Info_Update(Identifier,Data,&chassis_motor[0]);
  MIT_motor_Info_Update(Identifier,Data,&chassis_motor[1]);
	MIT_motor_Info_Update(Identifier,Data,&chassis_motor[2]);
	MIT_motor_Info_Update(Identifier,Data,&chassis_motor[3]);

}

/**
  * @brief  Function to converting the FDCAN3 received message to Fifo0.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier,uint8_t Data[8])
{



}


/**
  * @brief  Function to converting the FDCAN2 received message to Fifo1.
	* @param  Identifier: Received the identifier.
	* @param  Data: Array that contains the received massage.
  * @retval None
  */
static void FDCAN2_RxFifo1RxHandler(uint32_t *Identifier,uint8_t Data[8])
{
	
	DJI_motor_Info_Update(Identifier,Data,&shooting_motor[0]);
	DJI_motor_Info_Update(Identifier,Data,&shooting_motor[1]);
	DJI_motor_Info_Update(Identifier,Data,&shooting_motor[2]);

}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame.Header, FDCAN_RxFIFO0Frame.Data);
	
	/* Debug: count how many times RX callback fires */
	fdcan1_rx_callback_count++;
	
  if(hfdcan == &hfdcan1){	
	
   FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	 
	}

  if(hfdcan == &hfdcan3){
	
	 FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier,FDCAN_RxFIFO0Frame.Data);
	
	}
	
}
	
/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxFIFO1Frame.Header, FDCAN_RxFIFO1Frame.Data);
	
  FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header.Identifier,FDCAN_RxFIFO1Frame.Data);
	 
}