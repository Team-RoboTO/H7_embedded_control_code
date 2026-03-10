#ifndef CUBEMARS_MOTOR_H
#define CUBEMARS_MOTOR_H

/**
 * @file    cubemars.h
 * @brief   CubeMars AK40-10 — CAN protocol + motor control
 *          STM32H7 FDCAN, MIT mini-cheetah protocol, classic CAN, 1Mbps
 *
 * Sections:
 *   1. Constants
 *   2. Data types
 *   3. Protocol functions  (MIT frame packing, special commands, transmit)
 *   4. Control functions   (velocity, position, torque + PID)
 *
 * Control architecture:
 *   Velocity : PID_VELOCITY output ? MIT frame vel field (kp=kd=0)
 *   Position : PID_POSITION output ? MIT frame pos field (kp=10, kd=0.5)
 *   Torque   : direct Nm ? MIT frame torque field, no PID
 *
 * AK40-10 physical limits:
 *   Position : ±12.5 rad  |  Velocity : ±50 rad/s  |  Torque : ±65 Nm
 */

#include "main.h"
#include "PID.h"
#include <stdint.h>

  /****************************/
 /*   1. CONSTANTS           */
/****************************/

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

  /****************************/
 /*   2. DATA TYPES          */
/****************************/

/**
 * @brief CAN TX frame container — one per motor per control cycle.
 */
typedef struct {
    FDCAN_TxHeaderTypeDef   Tx_header;
    uint8_t                 Tx_data[CUBEMARS_CAN_FRAME_BYTES];
} CubeMars_Tx_message_t;

/**
 * @brief CubeMars motor instance — holds CAN ID, PID state, and TX frame.
 */
typedef struct {
    uint32_t                can_id;
    PID_Info_TypeDef        vel_pid;
    PID_Info_TypeDef        pos_pid;
    CubeMars_Tx_message_t   tx_msg;
} CubeMars_Motor_t;

  /****************************/
 /*   3. PROTOCOL FUNCTIONS  */
/****************************/

/**
 * @brief Initialise FDCAN TX header for a CubeMars motor.
 * @param msg      Pointer to CubeMars_Tx_message_t.
 * @param motor_id CAN ID of the target motor.
 */
void CUBEMARS_init_tx_message(CubeMars_Tx_message_t *msg, uint32_t motor_id);

/**
 * @brief Pack a full MIT mini-cheetah control frame into msg->Tx_data.
 * @param msg      Pointer to CubeMars_Tx_message_t.
 * @param position Desired position (rad), clamped to AK40 limits.
 * @param velocity Desired velocity (rad/s), clamped to AK40 limits.
 * @param kp       Onboard position stiffness gain.
 * @param kd       Onboard velocity damping gain.
 * @param torque   Feed-forward torque (Nm), clamped to AK40 limits.
 */
void CUBEMARS_pack_mit_frame(CubeMars_Tx_message_t *msg,
                             float position,
                             float velocity,
                             float kp,
                             float kd,
                             float torque);

/** @brief Send Enter Motor Control mode command. */
HAL_StatusTypeDef CUBEMARS_enter_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id);

/** @brief Send Exit Motor Control mode command. */
HAL_StatusTypeDef CUBEMARS_exit_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id);

/** @brief Send Set Zero Position command. */
HAL_StatusTypeDef CUBEMARS_set_zero(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id);

/**
 * @brief Transmit a packed MIT control frame over FDCAN.
 */
HAL_StatusTypeDef CUBEMARS_transmit(FDCAN_HandleTypeDef *hfdcan, CubeMars_Tx_message_t *msg);

  /****************************/
 /*   4. CONTROL FUNCTIONS   */
/****************************/

/**
 * @brief Initialise a CubeMars motor control instance.
 * @param motor      Pointer to CubeMars_Motor_t.
 * @param can_id     Motor CAN ID (set via CubeMars PC software).
 * @param vel_params PID params [KP,KI,KD,Alpha,Deadband,LimitI,LimitOut], or NULL.
 * @param pos_params PID params for position control, or NULL.
 */
void CUBEMARS_motor_init(CubeMars_Motor_t *motor,
                         uint32_t          can_id,
                         float             vel_params[PID_PARAMETER_NUM],
                         float             pos_params[PID_PARAMETER_NUM]);

/**
 * @brief Velocity control — PID_VELOCITY output ? MIT frame vel field.
 * @param motor    Pointer to CubeMars_Motor_t.
 * @param hfdcan   FDCAN handle.
 * @param setpoint Target velocity (rad/s).
 * @param measured Measured velocity (rad/s) from RX feedback.
 */
HAL_StatusTypeDef CUBEMARS_motor_set_velocity(CubeMars_Motor_t    *motor,
                                              FDCAN_HandleTypeDef *hfdcan,
                                              float                setpoint,
                                              float                measured);

/**
 * @brief Position control — PID_POSITION output ? MIT frame pos field.
 * @param motor    Pointer to CubeMars_Motor_t.
 * @param hfdcan   FDCAN handle.
 * @param setpoint Target position (rad).
 * @param measured Measured position (rad) from RX feedback.
 */
HAL_StatusTypeDef CUBEMARS_motor_set_position(CubeMars_Motor_t    *motor,
                                              FDCAN_HandleTypeDef *hfdcan,
                                              float                setpoint,
                                              float                measured);

/**
 * @brief Direct torque control — Nm command straight to MIT frame, no PID.
 * @param motor     Pointer to CubeMars_Motor_t.
 * @param hfdcan    FDCAN handle.
 * @param torque_nm Desired torque (Nm), clamped to AK40 limits.
 */
HAL_StatusTypeDef CUBEMARS_motor_set_torque(CubeMars_Motor_t    *motor,
                                            FDCAN_HandleTypeDef *hfdcan,
                                            float                torque_nm);

#endif /* CUBEMARS_MOTOR_H */