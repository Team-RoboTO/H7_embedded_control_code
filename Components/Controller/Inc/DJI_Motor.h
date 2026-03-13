#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H


#include "main.h"
#include "PID.h"
#include "config.h"
#include "stm32h723xx.h"
#include "bsp_can.h"
#include "motor.h"
/**
 * @file    dji_motor.h
 * @brief   DJI M3508 and M2006 CAN protocol + motor control
 *          STM32H7 FDCAN, classic CAN, 1Mbps
 */
 
 
// Structs
/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
    DJI_M2006,
    DJI_MOTOR_TYPE_NUM,
}DJI_Motor_Type_e;

/**
 * @brief typedef structure that contains the information for the motor FDCAN transmit and recieved .
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */

}DJI_Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the data for the Motor Device.
 */
typedef struct 
{
  bool Initlized;   /*!< init flag */
  int16_t  Current;   /*!< Motor electric current */
  int16_t  Velocity;    /*!< Motor rotate velocity (RPM)*/
  int16_t  Encoder;   /*!< Motor encoder angle */
  int16_t  Last_Encoder;   /*!< previous Motor encoder angle */
  float    Angle;   /*!< Motor angle in degree */
  uint8_t  Temperature;   /*!< Motor Temperature */
	
}DJI_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the Damiao Motor Device.
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< Type of Motor */
  DJI_Motor_CANFrameInfo_typedef FDCANFrame;    /*!< information for the CAN Transfer */
	DJI_Motor_Data_Typedef Data;   /*!< information for the Motor Device */
}DJI_Motor_Info_Typedef;

// Functions
extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, DJI_Motor_Info_Typedef *DJI_Motor);
extern void DJI_M3508_M2006_TxMessage (FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4);
extern void DJI_GM6020_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, uint32_t tx_id, int16_t vol1, int16_t vol2, int16_t vol3, int16_t vol4);
extern DJI_Motor_Info_Typedef DJI_Yaw_Motor, Chassis_Motor[4];

#endif /* DJI_MOTOR_H */