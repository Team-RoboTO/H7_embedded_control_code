/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : GrassFan Wang
  * @date           : 2025/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to init the  BSP_USART_Init functions
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"
#include "Referee_System.h"
#include "Image_Transmission.h"

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size);
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t );

/* FIX: Aggiunto allineamento a 32 byte per le operazioni sicure di D-Cache */
__attribute__((aligned(32), section (".AXI_SRAM"))) uint8_t USART1_Shared_MultiRx_Buf[2][256];
__attribute__((aligned(32), section (".AXI_SRAM"))) uint8_t USART7_Shared_MultiRx_Buf[2][256];

PLL2_ClocksTypeDef PLL2_ClockFreq;

/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void){
    
    // ==========================================
    // 1. USART1: Referee System
    // ==========================================
    HAL_RCCEx_GetPLL2ClockFreq(&PLL2_ClockFreq); // Get PLL2 P Q R  Clock Frequency
    uint32_t USART1_ClockFreq = PLL2_ClockFreq.PLL2_Q_Frequency; // USART1 use PLL2Q Clock Frequency
    
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->BRR = (uint32_t)(USART1_ClockFreq/115200); // Set baudrate 115200
    USART1->CR1 |= USART_CR1_UE;
    
    // FIX: Forza stato BUSY_RX e ReceptionType per la HAL
    huart1.RxState = HAL_UART_STATE_BUSY_RX; 
    huart1.ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    
    USART_RxDMA_MultiBuffer_Init(&huart1, (uint32_t *)USART1_Shared_MultiRx_Buf[0], (uint32_t *)USART1_Shared_MultiRx_Buf[1], 256);
    
    // ==========================================
    // 2. UART5: SBUS Remote Control
    // ==========================================
    // FIX: Forza stato BUSY_RX e ReceptionType per la HAL
    huart5.RxState = HAL_UART_STATE_BUSY_RX;
    huart5.ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    
    USART_RxDMA_MultiBuffer_Init(&huart5, (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], SBUS_RX_BUF_NUM);

    // ==========================================
    // 3. UART7: VT13 Controls (Robot Guida) — 921600 baud (set in CubeMX)
    // ==========================================
    // NON ri-inizializzare huart7 qui! MX_UART7_Init() in usart.c lo ha già
    // configurato a 921600. Facendo HAL_UART_Init a 115200 si distruggeva
    // la configurazione corretta e il VT13 non veniva mai parsato.
    
    // Forza lo stato Busy RX per permettere a HAL_UART_IRQHandler di processare gli interrupt
    huart7.RxState = HAL_UART_STATE_BUSY_RX;
    huart7.ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    USART_RxDMA_MultiBuffer_Init(&huart7, (uint32_t *)USART7_Shared_MultiRx_Buf[0], (uint32_t *)USART7_Shared_MultiRx_Buf[1], 256);
}

/**
  * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure
  * @param  DstAddress pointer to The source memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred
  * @retval none
  */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxXferSize    = DataLength * 2;

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
        
    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    /* Configure the source memory Buffer address  */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

    /* Configure the destination memory Buffer address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

    /* Configure DMA Stream destination address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

    /* Configure the length of data to be transferred from source to destination */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

    /* Enable double memory buffer */
    SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);  
}

/**
  * @brief  USER USART1 Reception Event Callback.(Referee_System)
  */
static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        
        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        
        if(Size > 10)
        {
            // Invalida la Cache per far leggere alla CPU i nuovi dati ricevuti dal DMA
            SCB_InvalidateDCache_by_Addr((uint32_t *)USART1_Shared_MultiRx_Buf[0], 256);
            
            // Elabora i dati ricevuti nel buffer 0 con il parser del Referee
            Referee_System_Frame_Update(USART1_Shared_MultiRx_Buf[0]);
            
            // Pulisce il buffer e aggiorna la cache in RAM
            memset(USART1_Shared_MultiRx_Buf[0], 0, 256);
            SCB_CleanDCache_by_Addr((uint32_t *)USART1_Shared_MultiRx_Buf[0], 256);
        }
        
        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, 256);
    }
    /* Current memory buffer used is Memory 1 */
    else{
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        
        /* Switch Memory 1 to Memory 0*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
    
        if(Size > 10)
        {
            // Invalida la Cache per far leggere alla CPU i nuovi dati ricevuti dal DMA
            SCB_InvalidateDCache_by_Addr((uint32_t *)USART1_Shared_MultiRx_Buf[1], 256);

            // Elabora i dati ricevuti nel buffer 1
            Referee_System_Frame_Update(USART1_Shared_MultiRx_Buf[1]);
            
            memset(USART1_Shared_MultiRx_Buf[1], 0, 256);
            SCB_CleanDCache_by_Addr((uint32_t *)USART1_Shared_MultiRx_Buf[1], 256);
        }
        /* Reset the receive count */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, 256);
    }
}

/**
  * @brief  USER UART7 Reception Event Callback (VT13 Control).
  */
static void USER_USART7_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){

    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET){
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        
        /* Switch Memory 0 to Memory 1*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        
        if(Size > 10)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)USART7_Shared_MultiRx_Buf[0], 256);
            
            Image_Transmission_Info_Update(USART7_Shared_MultiRx_Buf[0], 256);
            
            memset(USART7_Shared_MultiRx_Buf[0], 0, 256);
            SCB_CleanDCache_by_Addr((uint32_t *)USART7_Shared_MultiRx_Buf[0], 256);
        }
        
        __HAL_DMA_SET_COUNTER(huart->hdmarx, 256);
    }
    else{
        /* Disable DMA */
        __HAL_DMA_DISABLE(huart->hdmarx);
        
        /* Switch Memory 1 to Memory 0*/
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
    
        if(Size > 10)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)USART7_Shared_MultiRx_Buf[1], 256);

            Image_Transmission_Info_Update(USART7_Shared_MultiRx_Buf[1], 256);
            
            memset(USART7_Shared_MultiRx_Buf[1], 0, 256);
            SCB_CleanDCache_by_Addr((uint32_t *)USART7_Shared_MultiRx_Buf[1], 256);
        }
        __HAL_DMA_SET_COUNTER(huart->hdmarx, 256);
    }
}

static void USER_USART10_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){
    
}

static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){
    
}

static void USER_USART2_RxHandler(UART_HandleTypeDef *huart,uint16_t Size){
    
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
     /* huart5 (SBUS) is now handled directly in UART5_IRQHandler */
     
     if(huart == &huart1){
        USER_USART1_RxHandler(huart,Size);
     }
     else if(huart == &huart7){
        USER_USART7_RxHandler(huart,Size);
     }
    
   huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    
  /* Enable IDLE interrupt */
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    
  /* Enable the DMA transfer for the receiver request */
   SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    
  /* Enable DMA */
   __HAL_DMA_ENABLE(huart->hdmarx);
}

void USART_Vofa_Justfloat_Transmit(float SendValue1,float SendValue2,float SendValue3){
 
    __attribute__((section (".AXI_SRAM")))  static uint8_t Rx_Buf[16];

    uint8_t *SendValue1_Pointer,*SendValue2_Pointer,*SendValue3_Pointer;

    SendValue1_Pointer = (uint8_t *)&SendValue1;
    SendValue2_Pointer = (uint8_t *)&SendValue2;
    SendValue3_Pointer = (uint8_t *)&SendValue3;

    Rx_Buf[0] =  *SendValue1_Pointer;
    Rx_Buf[1] =  *(SendValue1_Pointer + 1);
    Rx_Buf[2] =  *(SendValue1_Pointer + 2);
    Rx_Buf[3] =  *(SendValue1_Pointer + 3);
    Rx_Buf[4] =  *SendValue2_Pointer;
    Rx_Buf[5] =  *(SendValue2_Pointer + 1);
    Rx_Buf[6] =  *(SendValue2_Pointer + 2);
    Rx_Buf[7] =  *(SendValue2_Pointer + 3);
    Rx_Buf[8] =  *SendValue3_Pointer;
    Rx_Buf[9] =  *(SendValue3_Pointer + 1);
    Rx_Buf[10] = *(SendValue3_Pointer + 2);
    Rx_Buf[11] = *(SendValue3_Pointer + 3);
    Rx_Buf[12] =  0x00;
    Rx_Buf[13] =  0x00;
    Rx_Buf[14] =  0x80;
    Rx_Buf[15] =  0x7F;
    
    HAL_UART_Transmit_DMA(&huart1,Rx_Buf,sizeof(Rx_Buf));
}