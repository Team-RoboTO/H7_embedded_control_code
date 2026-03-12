#ifndef CUBEMARS_MOTOR_H
#define CUBEMARS_MOTOR_H

#include "main.h"
#include "PID.h"
#include "config.h"
#include "stm32h723xx.h"
#include "bsp_can.h"


/**
 * @file    cubemars.h
 * @brief   CubeMars AK40-10 — CAN protocol + motor control
 *          STM32H7 FDCAN, MIT mini-cheetah protocol, classic CAN, 1Mbps
 *
 */

#include "main.h"
#include "PID.h"
#include <stdint.h>


#define AK40_POS_MIN        -12.5f
#define AK40_POS_MAX         12.5f
#define AK40_VEL_MIN        -50.0f
#define AK40_VEL_MAX         50.0f
#define AK40_TORQUE_MIN     -65.0f
#define AK40_TORQUE_MAX      65.0f
#define AK40_KP_MIN           0.0f
#define AK40_KP_MAX         500.0f
#define AK40_KD_MIN           0.0f
#define AK40_KD_MAX           5.0f

/* MIT bit field widths */
#define MIT_POS_BITS        16
#define MIT_VEL_BITS        12
#define MIT_KP_BITS         12
#define MIT_KD_BITS         12
#define MIT_TORQUE_BITS     12

#define CUBEMARS_CAN_FRAME_BYTES    8U

/* Special command words */
#define CUBEMARS_CMD_ENTER_CONTROL  0xFFFFFFFFFFFFFFFFULL
#define CUBEMARS_CMD_EXIT_CONTROL   0xFFFFFFFFFFFFFFFEULL
#define CUBEMARS_CMD_SET_ZERO       0xFFFFFFFFFFFFFFF5ULL

/* Default onboard impedance gains for position mode */
#define AK40_DEFAULT_KP     10.0f
#define AK40_DEFAULT_KD      0.5f


#endif /* CUBEMARS_MOTOR_H */