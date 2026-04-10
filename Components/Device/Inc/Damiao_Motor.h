#ifndef DAMIAO_MOTOR_H
#define DAMIAO_MOTOR_H
#include "main.h"
#include "PID.h"
#include "stm32h723xx.h"
#include "bsp_can.h"
/**
 * @file    damiao_motor.h
 * @brief   DM-J6006-2EC and DM_JM4310-2EC CAN protocol + motor control
 *          STM32H7 FDCAN, classic CAN, 1Mbps
 */
 
// Constants
// DM Motor Model Constants
#define DM_J6006_POS_MIN        -12.5f
#define DM_J6006_POS_MAX         12.5f
#define DM_J6006_VEL_MIN        -45.0f
#define DM_J6006_VEL_MAX         45.0f
#define DM_J6006_TORQUE_MIN     -12.0f
#define DM_J6006_TORQUE_MAX      12.0f
#define DM_J4310_POS_MIN        -12.5f
#define DM_J4310_POS_MAX         12.5f
#define DM_J4310_VEL_MIN        -50.0f
#define DM_J4310_VEL_MAX         50.0f
#define DM_J4310_TORQUE_MIN     -10.0f
#define DM_J4310_TORQUE_MAX      10.0f

/**
 * @brief  typedef enum that contains the type of DM Motor Device.
 */
typedef enum
{
  DM_MOTOR_J6006,
  DM_MOTOR_J4310,
  DM_MOTOR_8009,
  DM_MOTOR_TYPE_NUM,
}DM_motor_type_t;

/**
 * @brief  typedef enum that control mode the type of DM_Motor.
 */
typedef enum
{
  MIT,
	POSITION_VELOCITY,
	VELOCITY,
}DM_Motor_Control_Mode_Type_e;

/**
 * @brief  typedef enum that CMD of DM_Motor .
 */
typedef enum{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
}DM_Motor_CMD_Type_e;

/**
 * @brief typedef structure that contains the information for the motor FDCAN transmit and recieved .
 */
typedef struct
{
  uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier */
  uint32_t RxIdentifier;   /*!< Specifies FDCAN recieved identifier */
}DM_Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure that contains the param range for the DM_Motor .
 */
typedef struct 
{
  float  P_MAX;
	float  V_MAX;
	float  T_MAX;
}DM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DM Motor Device.
 */
typedef struct 
{
  bool Initlized;    				/*!< init flag */
  uint8_t  State; 				 	/*!< Motor Message */
  uint16_t  P_int;   				/*!< Motor Positon  uint16 */
	uint16_t  V_int;   				/*!< Motor Velocity uint16 */
	uint16_t  T_int;   				/*!< Motor Torque   uint16 */
	float  Position;   				/*!< Motor Positon  */
	float  Last_Position;   	/*!< Motor previous Positon  */
  float  Velocity;   				/*!< Motor Velocity */
  float  Torque;     				/*!< Motor Torque   */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS   */
	float  Temperature_Rotor; /*!< Motor Temperature_Rotor */
	float  Angle;
	float  Angle_sum;
}DM_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the information for the DM Motor Device.
 */
typedef struct
{
  DM_motor_type_t               Type;          /*!< Type of DM Motor */
	DM_Motor_Control_Mode_Type_e	Control_Mode;
  DM_Motor_CANFrameInfo_typedef FDCANFrame;   
	DM_Motor_Param_Range_Typedef  Param_Range; 
	DM_Motor_Data_Typedef         Data;   
}DM_Motor_Info_Typedef;

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
}DM_Motor_Control_Info_Typedef;

// Extern Variables
extern DM_Motor_Info_Typedef         DM_8009_Motor[4];
extern DM_Motor_Control_Info_Typedef DM_Motor_Contorl_Info[4];
extern DM_Motor_Info_Typedef         DM_Yaw_Motor;

// Extern Functions
extern void DM_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, DM_Motor_Info_Typedef *DM_Motor);
extern void DM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, DM_Motor_Info_Typedef *DM_Motor, uint8_t CMD);
extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, DM_Motor_Info_Typedef *DM_Motor, float Postion, float Velocity, float KP, float KD, float Torque);

#endif /* DAMIAO_MOTOR_H */