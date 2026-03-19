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
 * @brief   DJI M3508, M2006, GM6020 CAN protocol + motor control
 *          STM32H7 FDCAN, classic CAN, 1Mbps
 */

// Structs
/**
 * @brief typedef enum that contains the type of DJI Motor Device.
 */
typedef enum{
    DJI_GM6020,
    DJI_M3508,
		DJI_M3508_SHOOTING_WHEELS,
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
  bool     Initlized;   	// init flag
  int16_t  Current;   		// Motor electric current
  int16_t  Velocity_rpm;  // Motor rotate velocity at the encoder shaft(RPM) !!!pay attention to reduction ratio
	float    Velocity_rads; // Motor rotate velocity at the output shaft (rad/s) 
  int16_t  Encoder;  		  // Motor encoder angle
  int16_t  Last_Encoder;  // previous Motor encoder angle
  float    Angle;   			// Motor angle in degree
	float    Angle_sum;     // Motor total angle in rad 
  uint8_t  Temperature;   // Motor Temperature
}DJI_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< Type of Motor */
  DJI_Motor_CANFrameInfo_typedef FDCANFrame;    /*!< information for the CAN Transfer */
	DJI_Motor_Data_Typedef Data;   /*!< information for the Motor Device */
}DJI_Motor_Info_Typedef;

// Extern Variables
extern DJI_Motor_Info_Typedef DJI_Yaw_Motor;
extern DJI_Motor_Info_Typedef DJI_Chassis_Motor[4];
extern DJI_Motor_Info_Typedef DJI_Shooting_Motor[2];
extern DJI_Motor_Info_Typedef DJI_Rev_Motor;

// Extern Functions
extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, DJI_Motor_Info_Typedef *DJI_Motor);
extern void DJI_M3508_M2006_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, int16_t cur1, int16_t cur2, int16_t cur3, int16_t cur4);
extern void DJI_GM6020_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, uint32_t tx_id, int16_t vol1, int16_t vol2, int16_t vol3, int16_t vol4);

// ADC constants
static const float DJI_Motor_ADC[DJI_MOTOR_TYPE_NUM] = {
    [DJI_GM6020]                = 16384.f / 3.f,
    [DJI_M3508]                 = 16384.f / 20.f,
    [DJI_M3508_SHOOTING_WHEELS] = 16384.f / 20.f,
    [DJI_M2006]                 = 10000.0f / 10.0f,
};
#endif /* DJI_MOTOR_H */