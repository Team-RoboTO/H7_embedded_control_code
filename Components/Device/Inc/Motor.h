/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : The header file of Motor.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/2
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32h723xx.h"
#include "bsp_can.h"
#include "stdbool.h"

#define M2006_ADC_CONVERTION 1000;
#define GM6020_ADC_CONVERTION 1811.943f // Computed experimentally
#define M3508_ADC_CONVERTION 819.2f // 819.2 = 16384/20A
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
 * @brief  typedef enum that control mode the type of MIT_motor.
 */
typedef enum
{
  MIT,
	POSITION_VELOCITY,
	VELOCITY,
}MIT_motor_Control_Mode_Type_e;

/**
 * @brief  typedef enum that CMD of MIT_motor .
 */
typedef enum{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  MIT_motor_CMD_Type_Num,
}MIT_motor_CMD_Type_e;

/**
 * @brief typedef structure that contains the information for the motor FDCAN transmit and recieved .
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */

}Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the data for the Motor Device.
 */
typedef struct 
{ 
  bool Initlized;
  int16_t  Current;
  int16_t  Velocity;
  int16_t  Encoder;
  int16_t  Last_Encoder;
  float    Angle;          /*!< Motor angle in degree (-180 to 180) */
  float    Angle_sum;      /*!< Motor cumulative angle in degree (unbounded) */ // ? aggiungi
  uint8_t  Temperature;
	
}DJI_Motor_Data_Typedef;


/**
 * @brief typedef structure that contains the param range for the MIT_motor .
 */
typedef struct 
{
  float  P_MAX;
	float  V_MAX;
	float  T_MAX;
}MIT_motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DJI Motor Device.
 */
typedef struct 
{
  bool Initlized;
  uint8_t  State;
  uint16_t  P_int;
  uint16_t  V_int;
  uint16_t  T_int;
  float  Position;
  float  Last_Position;   /*!< Previous motor position (rad) */  
  float  Velocity;
  float  Torque;
  float  Temperature_MOS;
  float  Temperature_Rotor;
  float  Angle;
  float  Angle_sum;       /*!< Cumulative motor angle (rad, unbounded) */  
	
}MIT_motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the Damiao Motor Device.
 */
typedef struct
{
	DJI_Motor_Type_e Type;   /*!< Type of Motor */
  Motor_CANFrameInfo_typedef FDCANFrame;    /*!< information for the CAN Transfer */
	DJI_Motor_Data_Typedef Data;   /*!< information for the Motor Device */
}DJI_Motor_Info_Typedef;

/**
 * @brief typedef structure that contains the information for the DJI Motor Device.
 */
typedef struct
{
  
	MIT_motor_Control_Mode_Type_e	Control_Mode;
  Motor_CANFrameInfo_typedef FDCANFrame;   
	MIT_motor_Param_Range_Typedef Param_Range; 
	MIT_motor_Data_Typedef Data;   

}MIT_motor_Info_Typedef;

/**
 * @brief typedef structure that contains the control information for the DM Motor Device .
 */
typedef struct
{
  float Position;
	float Velocity;
	float KP;
	float KD;
	float Torque;
	float Angle;
}MIT_motor_Contorl_Info_Typedef;

/* Externs ------------------------------------------------------------------*/
extern DJI_Motor_Info_Typedef shooting_motor[3];

extern MIT_motor_Info_Typedef chassis_motor[4], gimbal_motor[2];

extern MIT_motor_Contorl_Info_Typedef chassis_motor_Contorl_Info[4];

extern MIT_motor_Contorl_Info_Typedef gimbal_motor_Contorl_Info[2];

extern void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor);

extern void MIT_motor_Info_Update(uint32_t *Identifier,uint8_t *Rx_Buf,MIT_motor_Info_Typedef *MIT_motor);

extern void MIT_motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,MIT_motor_Info_Typedef *MIT_motor,uint8_t CMD);

extern void MIT_motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,MIT_motor_Info_Typedef *MIT_motor,float Postion, float Velocity, float KP, float KD, float Torque);

#endif //DEVICE_MOTOR_H
