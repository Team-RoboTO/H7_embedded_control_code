#ifndef DAMIAO_MOTOR_H
#define DAMIAO_MOTOR_H

/**
 * @file    damiao.h
 * @brief   Damiao DM-J6006-2EC and DM-J4310-2EC — CAN protocol + motor control
 *          STM32H7 FDCAN, MIT-style protocol, classic CAN, 1Mbps
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
 * DM-J6006-2EC limits:  Pos ±12.5 rad | Vel ±45 rad/s | Torque ±30 Nm
 * DM-J4310-2EC limits:  Pos ±12.5 rad | Vel ±30 rad/s | Torque ±10 Nm
 */

#include "main.h"
#include "PID.h"
#include <stdint.h>

  /****************************/
 /*   1. CONSTANTS           */
/****************************/

/* DM-J6006-2EC limits */
#define DM_J6006_POS_MIN        -12.5f
#define DM_J6006_POS_MAX         12.5f
#define DM_J6006_VEL_MIN        -45.0f
#define DM_J6006_VEL_MAX         45.0f
#define DM_J6006_TORQUE_MIN     -30.0f
#define DM_J6006_TORQUE_MAX      30.0f

/* DM-J4310-2EC limits */
#define DM_J4310_POS_MIN        -12.5f
#define DM_J4310_POS_MAX         12.5f
#define DM_J4310_VEL_MIN        -30.0f
#define DM_J4310_VEL_MAX         30.0f
#define DM_J4310_TORQUE_MIN     -10.0f
#define DM_J4310_TORQUE_MAX      10.0f

/* Shared Kp/Kd limits */
#define DM_KP_MIN                 0.0f
#define DM_KP_MAX               500.0f
#define DM_KD_MIN                 0.0f
#define DM_KD_MAX                 5.0f

/* MIT bit field widths */
#define DM_POS_BITS             16
#define DM_VEL_BITS             12
#define DM_KP_BITS              12
#define DM_KD_BITS              12
#define DM_TORQUE_BITS          12

/* RX feedback ID offset — feedback ID = master ID + 0x100 */
#define DM_RX_ID_OFFSET         0x100U

#define DM_CAN_FRAME_BYTES      8U

/* Special command bytes (Byte 7, Bytes 0–6 = 0xFF) */
#define DM_CMD_ENTER_CONTROL    0xFCU
#define DM_CMD_EXIT_CONTROL     0xFDU
#define DM_CMD_SET_ZERO         0xFEU
#define DM_CMD_CLEAR_ERROR      0xFBU

/* Default onboard impedance gains for position mode */
#define DM_DEFAULT_KP           10.0f
#define DM_DEFAULT_KD            0.5f

  /****************************/
 /*   2. DATA TYPES          */
/****************************/

/**
 * @brief Damiao motor model selector — determines which parameter limits to apply.
 */
typedef enum {
    DM_MOTOR_J6006 = 0,
    DM_MOTOR_J4310 = 1,
} DM_motor_type_t;

/**
 * @brief CAN TX frame container — one per motor per control cycle.
 */
typedef struct {
    FDCAN_TxHeaderTypeDef   Tx_header;
    uint8_t                 Tx_data[DM_CAN_FRAME_BYTES];
} DM_Tx_message_t;

/**
 * @brief Damiao motor instance — holds model type, CAN ID, PID state, and TX frame.
 */
typedef struct {
    uint32_t            can_id;
    DM_motor_type_t     motor_type;
    PID_Info_TypeDef    vel_pid;
    PID_Info_TypeDef    pos_pid;
    DM_Tx_message_t     tx_msg;
} DM_Motor_t;

  /****************************/
 /*   3. PROTOCOL FUNCTIONS  */
/****************************/

/**
 * @brief Initialise FDCAN TX header for a Damiao motor.
 * @param msg      Pointer to DM_Tx_message_t.
 * @param motor_id Master CAN ID of the target motor.
 */
void DM_init_tx_message(DM_Tx_message_t *msg, uint32_t motor_id);

/**
 * @brief Pack a full MIT-mode control frame for a Damiao motor.
 * @param msg        Pointer to DM_Tx_message_t to populate.
 * @param motor_type DM_MOTOR_J6006 or DM_MOTOR_J4310 — selects correct limits.
 * @param position   Desired position (rad).
 * @param velocity   Desired velocity (rad/s).
 * @param kp         Onboard position stiffness gain.
 * @param kd         Onboard velocity damping gain.
 * @param torque     Feed-forward torque (Nm).
 */
void DM_pack_mit_frame(DM_Tx_message_t *msg,
                       DM_motor_type_t  motor_type,
                       float            position,
                       float            velocity,
                       float            kp,
                       float            kd,
                       float            torque);

/** @brief Send Enter Motor Control mode command. */
HAL_StatusTypeDef DM_enter_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id);

/** @brief Send Exit Motor Control mode command. */
HAL_StatusTypeDef DM_exit_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id);

/** @brief Send Set Zero Position command. */
HAL_StatusTypeDef DM_set_zero(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id);

/** @brief Send Clear Error command. */
HAL_StatusTypeDef DM_clear_error(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id);

/**
 * @brief Transmit a packed Damiao MIT control frame over FDCAN.
 */
HAL_StatusTypeDef DM_transmit(FDCAN_HandleTypeDef *hfdcan, DM_Tx_message_t *msg);

  /****************************/
 /*   4. CONTROL FUNCTIONS   */
/****************************/

/**
 * @brief Initialise a Damiao motor control instance.
 * @param motor      Pointer to DM_Motor_t.
 * @param motor_type DM_MOTOR_J6006 or DM_MOTOR_J4310.
 * @param can_id     Motor master CAN ID.
 * @param vel_params PID params [KP,KI,KD,Alpha,Deadband,LimitI,LimitOut], or NULL.
 * @param pos_params PID params for position control, or NULL.
 */
void DM_motor_init(DM_Motor_t      *motor,
                   DM_motor_type_t  motor_type,
                   uint32_t         can_id,
                   float            vel_params[PID_PARAMETER_NUM],
                   float            pos_params[PID_PARAMETER_NUM]);

/**
 * @brief Velocity control — PID_VELOCITY output ? MIT frame vel field.
 * @param motor    Pointer to DM_Motor_t.
 * @param hfdcan   FDCAN handle.
 * @param setpoint Target velocity (rad/s).
 * @param measured Measured velocity (rad/s) from RX feedback.
 */
HAL_StatusTypeDef DM_motor_set_velocity(DM_Motor_t          *motor,
                                        FDCAN_HandleTypeDef *hfdcan,
                                        float                setpoint,
                                        float                measured);

/**
 * @brief Position control — PID_POSITION output ? MIT frame pos field.
 * @param motor    Pointer to DM_Motor_t.
 * @param hfdcan   FDCAN handle.
 * @param setpoint Target position (rad).
 * @param measured Measured position (rad) from RX feedback.
 */
HAL_StatusTypeDef DM_motor_set_position(DM_Motor_t          *motor,
                                        FDCAN_HandleTypeDef *hfdcan,
                                        float                setpoint,
                                        float                measured);

/**
 * @brief Direct torque control — Nm command straight to MIT frame, no PID.
 * @param motor     Pointer to DM_Motor_t.
 * @param hfdcan    FDCAN handle.
 * @param torque_nm Desired torque (Nm), clamped to motor model limits.
 */
HAL_StatusTypeDef DM_motor_set_torque(DM_Motor_t          *motor,
                                      FDCAN_HandleTypeDef *hfdcan,
                                      float                torque_nm);

#endif /* DAMIAO_MOTOR_H */