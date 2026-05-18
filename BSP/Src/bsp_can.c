/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v2.1
  ******************************************************************************
  * @attention      : Pay attention to enable the fdcan filter
  *
  *   Bus assignment (new robot):
  *     FDCAN1 (classic CAN, FIFO0) : DJI Shooting_Motor[2] + DJI Rev_Motor + DJI Yaw_Motor (GM6020)
  *     FDCAN2 (classic CAN, FIFO1) : DM Yaw + CM Pitch + CM Chassis[4]
  *
  *   FDCAN1 RX IDs (all standard):
  *     0x201  DJI_Shooting_Motor[0]   (M3508, left)
  *     0x202  DJI_Shooting_Motor[1]   (M3508, right)
  *     0x203  DJI_Rev_Motor           (M2006, feeder)
  *     0x206  DJI_Yaw_Motor           (GM6020, yaw)
  *
  *   FDCAN2 RX IDs:
  *     Standard:
  *       0x01  CM_Chassis_Motor[0]    (AK40-10)
  *       0x02  CM_Chassis_Motor[1]    (AK40-10)
  *       0x03  CM_Chassis_Motor[2]    (AK40-10)
  *       0x04  CM_Chassis_Motor[3]    (AK40-10)
  *       0x05  DM_Yaw_Motor           (DM-J6006-2EC)
  *     Extended:
  *       0x0000296A  CM_Pitch_Motor   (AK40-10)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#include "bsp_can.h"
#include "Motor.h"
#include "Remote_Control.h"

#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
#include "DJI_Motor.h"
#include "type_c_can.h"

/**
 * @brief The structure that contains the Information of FDCAN1 and FDCAN2 Receive.
 */
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;
FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;

/**
 * @brief Debug variables for FDCAN1 TX diagnostics.
 *
 *   fdcan1_tx_total             Total TX attempts (calls to AddMessageToTxFifoQ)
 *   fdcan1_fifo_overflow_count  HAL returned error (TX FIFO was full, message dropped)
 *   fdcan1_hw_bus_errors        Cumulative hardware bus errors (no ACK, bit/stuff errors)
 *   fdcan1_hw_error_snapshot    Latest raw TxErrorCnt from CAN controller
 *   fdcan1_tx_status            Last HAL return code (HAL_OK = 0)
 *   fdcan1_protocol_status      Last CAN error code from protocol status register
 *   fdcan1_rx_callback_count    Total RX callbacks fired
 */
volatile HAL_StatusTypeDef fdcan1_tx_status = HAL_OK;
volatile uint32_t fdcan1_protocol_status = 0;
volatile uint32_t fdcan1_hw_error_snapshot = 0;
volatile uint32_t fdcan1_hw_bus_errors = 0;
volatile uint32_t fdcan1_rx_error_count = 0;
volatile uint32_t fdcan1_rx_callback_count = 0;
volatile uint32_t fdcan1_tx_total = 0;
volatile uint32_t fdcan1_fifo_overflow_count = 0;

/**
 * @brief Debug variables for FDCAN2 TX diagnostics.
 *
 *   fdcan2_tx_total             Total TX attempts (calls to AddMessageToTxFifoQ)
 *   fdcan2_fifo_overflow_count  HAL returned error (TX FIFO was full, message dropped)
 *   fdcan2_hw_bus_errors        Cumulative hardware bus errors (no ACK, bit/stuff errors)
 *   fdcan2_hw_error_snapshot    Latest raw TxErrorCnt from CAN controller
 *   fdcan2_tx_status            Last HAL return code (HAL_OK = 0)
 *   fdcan2_protocol_status      Last CAN error code from protocol status register
 *   fdcan2_rx_callback_count    Total RX callbacks fired
 */
volatile HAL_StatusTypeDef fdcan2_tx_status = HAL_OK;
volatile uint32_t fdcan2_protocol_status = 0;
volatile uint32_t fdcan2_hw_error_snapshot = 0;
volatile uint32_t fdcan2_hw_bus_errors = 0;
volatile uint32_t fdcan2_rx_error_count = 0;
volatile uint32_t fdcan2_rx_callback_count = 0;
volatile uint32_t fdcan2_tx_total = 0;
volatile uint32_t fdcan2_fifo_overflow_count = 0;

uint32_t fifo_number_1 = 0;

Type_C_Can_t Type_C_Can = {0};


/**
 * @brief The structure that contains the Information of FDCAN1 Transmit(CLASSIC_CAN).
 *        Bus: DJI shooting wheels + rev motor
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
 * @brief The structure that contains the Information of FDCAN2 Transmit(CLASSIC_CAN).
 *        Bus: DM yaw + CM pitch + CM chassis[4]
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
  *         FDCAN1: CLASSIC_CAN (DJI)   FDCAN2: CLASSIC_CAN (DM + CM)
  * @param  None
  * @retval None
  */
void BSP_FDCAN_Init(void){

  /* ---- FDCAN1: DJI motors (standard ID, shooting + rev) ---- */
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

  /* ---- FDCAN2: DM + CubeMars motors (standard + extended ID) ---- */
	FDCAN_FilterTypeDef FDCAN2_FilterConfig;

  /* Standard ID filter (DM MIT + CM MIT chassis feedback) -> FIFO1 */
  FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000;
  FDCAN2_FilterConfig.FilterID2 = 0x00000000;
  
	HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);

  /* Extended ID filter (CubeMars pitch feedback: 0x0000296A) -> FIFO1 */
  FDCAN_FilterTypeDef FDCAN2_ExtFilterConfig;

  FDCAN2_ExtFilterConfig.IdType = FDCAN_EXTENDED_ID;
  FDCAN2_ExtFilterConfig.FilterIndex = 0;
  FDCAN2_ExtFilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_ExtFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN2_ExtFilterConfig.FilterID1 = 0x00000000;
  FDCAN2_ExtFilterConfig.FilterID2 = 0x00000000;

  HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_ExtFilterConfig);

  /* Accept both standard and extended non-matching frames into FIFO1 */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan2);

	FDCAN_FilterTypeDef FDCAN3_FilterConfig;
	
	FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  
	HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_Start(&hfdcan3);
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
    if(FDCAN_TxFrame->hcan == &hfdcan1){
        FDCAN_ProtocolStatusTypeDef psr;
        FDCAN_ErrorCountersTypeDef err_counters;
        
        fdcan1_tx_total++;
        if (status != HAL_OK) {
            fdcan1_fifo_overflow_count++;
        }
        
        fdcan1_tx_status = status;
        HAL_FDCAN_GetProtocolStatus(FDCAN_TxFrame->hcan, &psr);
        fdcan1_protocol_status = psr.LastErrorCode;
        
        HAL_FDCAN_GetErrorCounters(FDCAN_TxFrame->hcan, &err_counters);
        /* Accumulate: add the delta since last snapshot */
        if (err_counters.TxErrorCnt > fdcan1_hw_error_snapshot) {
            fdcan1_hw_bus_errors += (err_counters.TxErrorCnt - fdcan1_hw_error_snapshot);
        }
        fdcan1_hw_error_snapshot = err_counters.TxErrorCnt;
        fdcan1_rx_error_count = err_counters.RxErrorCnt;
    }
		
    /* Capture debug info for FDCAN2 */
    if(FDCAN_TxFrame->hcan == &hfdcan2){
        FDCAN_ProtocolStatusTypeDef psr;
        FDCAN_ErrorCountersTypeDef err_counters;
        
        fdcan2_tx_total++;
        if (status != HAL_OK) {
            fdcan2_fifo_overflow_count++;
        }
        
        fdcan2_tx_status = status;
        HAL_FDCAN_GetProtocolStatus(FDCAN_TxFrame->hcan, &psr);
        fdcan2_protocol_status = psr.LastErrorCode;
        
        HAL_FDCAN_GetErrorCounters(FDCAN_TxFrame->hcan, &err_counters);
        /* Accumulate: add the delta since last snapshot */
        if (err_counters.TxErrorCnt > fdcan2_hw_error_snapshot) {
            fdcan2_hw_bus_errors += (err_counters.TxErrorCnt - fdcan2_hw_error_snapshot);
        }
        fdcan2_hw_error_snapshot = err_counters.TxErrorCnt;
        fdcan2_rx_error_count = err_counters.RxErrorCnt;
    }
}

/**
  * @brief  FDCAN1 RX handler � DJI motors (shooting wheels + rev + yaw GM6020).
  *         Dispatches to the single matching motor based on ID.
  *
  *         RX IDs:  0x201 = shoot left, 0x202 = shoot right,
  *                  0x203 = rev, 0x206 = yaw (GM6020)
  *
  * @param  Identifier: Received standard identifier.
  * @param  Data: 8-byte CAN data buffer.
  * @retval None
  */
static void FDCAN1_RxFifo0RxHandler(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[8])
{
    uint32_t id = RxHeader->Identifier;

    switch (id)
    {
        case DJI_SHOOTING_0_RX_ID:
            DJI_Motor_Info_Update(&id, Data, &DJI_Shooting_Motor[0]);
            break;

        case DJI_SHOOTING_1_RX_ID:
            DJI_Motor_Info_Update(&id, Data, &DJI_Shooting_Motor[1]);
            break;

        case DJI_REV_RX_ID:
            DJI_Motor_Info_Update(&id, Data, &DJI_Rev_Motor);
            break;
				
				case DJI_LIDAR_RX_ID:
				    DJI_Motor_Info_Update(&id, Data, &DJI_Lidar_Motor);
            break;

        default:
            break;
    }
}

/**
  * @brief  FDCAN3 RX handler � currently unused.
  * @param  Identifier: Received identifier.
  * @param  Data: 8-byte CAN data buffer.
  * @retval None
  */
static void FDCAN3_RxFifo0RxHandler(uint32_t *Identifier, uint8_t Data[8])
{
    if (*Identifier == TYPE_C_CAN_ID)
    {
        Type_C_Can_Update(Data);
    }
}

/**
  * @brief  FDCAN2 RX handler � DM yaw + CM pitch + CM chassis[4].
  *         Dispatches to the single matching motor based on ID and frame type.
  *
  *         Standard ID frames:
  *           0x01 = chassis[0], 0x02 = chassis[1],
  *           0x03 = chassis[2], 0x04 = chassis[3],
  *           0x05 = DM yaw (J6006)
  *
  *         Extended ID frames:
  *           0x0000296A = CM pitch (AK40-10)
  *
  * @param  RxHeader: Pointer to full RX header (needed for IdType check).
  * @param  Data: 8-byte CAN data buffer.
  * @retval None
  */
static void FDCAN2_RxFifo1RxHandler(FDCAN_RxHeaderTypeDef *RxHeader, uint8_t Data[8])
{
    uint32_t id = RxHeader->Identifier;
	
    fifo_number_1 = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);

    /* Standard ID � DM yaw or CM chassis */
    switch (id)
    {
        case CM_PITCH_RX_ID:
            CM_Motor_Info_Update(&id, Data, &CM_Pitch_Motor);
            break;
        case DM_YAW_RX_ID:
            DM_Motor_Info_Update(&id, Data, &DM_Yaw_Motor);
            break;
        case CM_CHASSIS_0_RX_ID:
            CM_Motor_Info_Update(&id, Data, &CM_Chassis_Motor[0]);
            break;

        case CM_CHASSIS_1_RX_ID:
            CM_Motor_Info_Update(&id, Data, &CM_Chassis_Motor[1]);
            break;

        case CM_CHASSIS_2_RX_ID:
            CM_Motor_Info_Update(&id, Data, &CM_Chassis_Motor[2]);
            break;

        case CM_CHASSIS_3_RX_ID:
            CM_Motor_Info_Update(&id, Data, &CM_Chassis_Motor[3]);
            break;

        default:
            break;
    }
}

/**
  * @brief  Rx FIFO 0 callback � drains ALL pending messages from the FIFO.
  *
  *         The "new message" interrupt fires once per arrival, but if several
  *         frames land between ISR entry and the read, only one would be
  *         consumed without the while-loop.  Draining the FIFO here prevents
  *         overflow when 4+ motors respond nearly simultaneously.
  *
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
	if (hfdcan == &hfdcan1)
	{
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, 
			                       &FDCAN_RxFIFO0Frame.Header, 
			                       FDCAN_RxFIFO0Frame.Data);

			fdcan1_rx_callback_count++;

			FDCAN1_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header, 
			                         FDCAN_RxFIFO0Frame.Data);
		}
	}

	if (hfdcan == &hfdcan3)
	{
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, 
			                       &FDCAN_RxFIFO0Frame.Header, 
			                       FDCAN_RxFIFO0Frame.Data);

			FDCAN3_RxFifo0RxHandler(&FDCAN_RxFIFO0Frame.Header.Identifier, 
			                         FDCAN_RxFIFO0Frame.Data);
		}
	}
}
	
/**
  * @brief  Rx FIFO 1 callback � drains ALL pending messages from the FIFO.
  *
  *         Same drain-loop strategy as FIFO 0 above.  This is the critical
  *         fix for FDCAN2 where 4 CubeMars chassis + DM yaw + CM pitch can
  *         produce up to 6 near-simultaneous responses.
  *
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{ 
	if (hfdcan == &hfdcan2)
	{
		while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, 
			                       &FDCAN_RxFIFO1Frame.Header, 
			                       FDCAN_RxFIFO1Frame.Data);

			fdcan2_rx_callback_count++;

			FDCAN2_RxFifo1RxHandler(&FDCAN_RxFIFO1Frame.Header, 
			                         FDCAN_RxFIFO1Frame.Data);
		}
	}
}


void FDCAN2_Reset(void)
{
    /* Step 1 — Deinit (internally calls Stop, clears bus-off state) */
    HAL_FDCAN_DeInit(&hfdcan2);

    /* Step 2 — Re-apply full init config with ExtFiltersNbr fix */
    hfdcan2.Instance                  = FDCAN2;
    hfdcan2.Init.FrameFormat          = FDCAN_FRAME_FD_BRS;
    hfdcan2.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission   = ENABLE;
    hfdcan2.Init.TransmitPause        = DISABLE;
    hfdcan2.Init.ProtocolException    = ENABLE;
    hfdcan2.Init.NominalPrescaler     = 5;
    hfdcan2.Init.NominalSyncJumpWidth = 5;
    hfdcan2.Init.NominalTimeSeg1      = 14;
    hfdcan2.Init.NominalTimeSeg2      = 5;
    hfdcan2.Init.DataPrescaler        = 1;
    hfdcan2.Init.DataSyncJumpWidth    = 5;
    hfdcan2.Init.DataTimeSeg1         = 14;
    hfdcan2.Init.DataTimeSeg2         = 5;
    hfdcan2.Init.MessageRAMOffset     = 853;
    hfdcan2.Init.StdFiltersNbr        = 1;
    hfdcan2.Init.ExtFiltersNbr        = 1;  /* Fixed: was 0 in CubeMX */
    hfdcan2.Init.RxFifo0ElmtsNbr     = 0;
    hfdcan2.Init.RxFifo0ElmtSize     = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxFifo1ElmtsNbr     = 8;
    hfdcan2.Init.RxFifo1ElmtSize     = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxBuffersNbr        = 0;
    hfdcan2.Init.RxBufferSize        = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.TxEventsNbr         = 0;
    hfdcan2.Init.TxBuffersNbr        = 0;
    hfdcan2.Init.TxFifoQueueElmtsNbr = 8;
    hfdcan2.Init.TxFifoQueueMode     = FDCAN_TX_FIFO_OPERATION;
    hfdcan2.Init.TxElmtSize          = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Step 3 — Re-apply filters */
    FDCAN_FilterTypeDef f;

    /* Standard ID filter → FIFO1 */
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    f.FilterID1    = 0x00000000;
    f.FilterID2    = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &f) != HAL_OK)
    {
        Error_Handler();
    }

    /* Extended ID filter → FIFO1 */
    f.IdType       = FDCAN_EXTENDED_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    f.FilterID1    = 0x00000000;
    f.FilterID2    = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &f) != HAL_OK)
    {
        Error_Handler();
    }

    /* Step 4 — Global filter: accept all non-matching into FIFO1 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
            FDCAN_ACCEPT_IN_RX_FIFO1,
            FDCAN_ACCEPT_IN_RX_FIFO1,
            FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Step 5 — Re-enable RX interrupt */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2,
            FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Step 6 — Start */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}
