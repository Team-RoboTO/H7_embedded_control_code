/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.h
  * @brief          : The header file of Motor.h 
  * @author         : GrassFan Wang
  * @date           : 2025/01/2
  * @version        : v1.0
  ******************************************************************************
  * @attention      : THIS FILE WILL ONLY CONTAIN MOTOR DEFS AT THE END REST OF THE STUFF WILL BE SENT TO SPECIFIC FILES
  ******************************************************************************
  */
/* USER CODE END Header */
#include "robot_config.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

/* ===========================================================================
 *  MOTOR CAN IDENTIFIER DEFINITIONS
 * =========================================================================== */

/* Chassis Motors (CubeMars AK40-10) */
#define CM_CHASSIS_0_TX_ID 121
#define CM_CHASSIS_0_RX_ID 0x00000079
#define CM_CHASSIS_1_TX_ID 122
#define CM_CHASSIS_1_RX_ID 0x0000007A
#define CM_CHASSIS_2_TX_ID 123
#define CM_CHASSIS_2_RX_ID 0x0000007B

#if IS_STD || IS_SENTRY 
#define CM_CHASSIS_3_TX_ID 120
#define CM_CHASSIS_3_RX_ID 0x00000078
#elif IS_HERO
#define CM_CHASSIS_3_TX_ID 126
#define CM_CHASSIS_3_RX_ID 0x0000007E
#endif


/* Yaw Motor (DM-4310-2EC) */
#define DM_YAW_TX_ID       0x01
#define DM_YAW_RX_ID       0x11

/* Pitch Motor (CubeMars AK40-10) */
#define CM_PITCH_TX_ID     2
#define CM_PITCH_RX_ID     0x00000002

/* Shooting Motors (DJI M3508) */
#define DJI_SHOOTING_TX_ID 0x200
#define DJI_SHOOTING_0_RX_ID 0x201
#define DJI_SHOOTING_1_RX_ID 0x202

/* Rev / Feeder Motor (DJI M2006) */
#define DJI_REV_TX_ID      0x200
#define DJI_REV_RX_ID      0x203

/* Push Motor (DJI M2006) */
#define DJI_PUSH_TX_ID    0x200
#define DJI_PUSH_RX_ID    0x203

/* Lidar Lifter Motor (DJI M2006) */
#define DJI_LIDAR_TX_ID    0x200
#define DJI_LIDAR_RX_ID    0x204

/* REV Motor (DaMiao DM-J6006-2EC) */
#define DM_REV_TX_ID       0x09
#define DM_REV_RX_ID       0x19

typedef enum {
    Motor_Enable,
    Motor_Disable,
    Motor_Save_Zero_Position,
    Motor_CMD_Type_Num,
} Motor_CMD_Type_e;


#endif
