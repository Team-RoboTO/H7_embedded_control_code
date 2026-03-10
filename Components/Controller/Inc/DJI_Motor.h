#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

/**
 * @file    dji.h
 * @brief   DJI M3508 and M2006 — CAN protocol + motor control
 *          STM32H7 FDCAN, classic CAN, 1Mbps
 *
 * Sections:
 *   1. Constants
 *   2. Data types
 *   3. Protocol functions  (CAN frame packing and transmit)
 *   4. Control functions   (velocity, position, current + PID)
 *
 * Control architecture:
 *   Velocity : PID_VELOCITY ? current_cmd ? CAN frame
 *   Position : PID_POSITION (outer) ? vel setpoint ? PID_VELOCITY (inner) ? current_cmd ? CAN frame
 *   Current  : direct clamp ? CAN frame, no PID
 *
 * DJI ESC has no onboard closed-loop control — all control is implemented here.
 */

#include "main.h"
#include "PID.h"
#include <stdint.h>

  /****************************/
 /*   1. CONSTANTS           */
/****************************/

/* TX arbitration IDs */
#define DJI_GROUP1_TX_ID        0x200U   /* Controls motors 0x201–0x204 */
#define DJI_GROUP2_TX_ID        0x1FFU   /* Controls motors 0x205–0x208 */

/* Motor CAN IDs */
#define DJI_MOTOR_ID_1          0x201U
#define DJI_MOTOR_ID_2          0x202U
#define DJI_MOTOR_ID_3          0x203U
#define DJI_MOTOR_ID_4          0x204U
#define DJI_MOTOR_ID_5          0x205U
#define DJI_MOTOR_ID_6          0x206U
#define DJI_MOTOR_ID_7          0x207U
#define DJI_MOTOR_ID_8          0x208U

/* Current limits (raw int16) */
#define DJI_M3508_MAX_CURRENT   16384    /* ±16384 maps to ±20A */
#define DJI_M2006_MAX_CURRENT   10000    /* ±10000 maps to ±10A */

/* Mechanical constants */
#define DJI_M3508_GEAR_RATIO    19.2f    /* Raw encoder RPM ? output shaft RPM */
#define DJI_M2006_GEAR_RATIO    36.0f
#define DJI_ENCODER_MAX         8191     /* Encoder ticks per revolution        */

#define DJI_CAN_FRAME_BYTES     8U
#define DJI_MOTORS_PER_GROUP    4U

  /****************************/
 /*   2. DATA TYPES          */
/****************************/

/**
 * @brief DJI motor model selector.
 */
typedef enum {
    DJI_MOTOR_M3508 = 0,
    DJI_MOTOR_M2006 = 1,
} DJI_motor_model_t;

/**
 * @brief CAN TX frame container — one per group broadcast.
 */
typedef struct {
    FDCAN_TxHeaderTypeDef   Tx_header;
    uint8_t                 Tx_data[DJI_CAN_FRAME_BYTES];
} DJI_Tx_message_t;

/**
 * @brief DJI motor instance — holds CAN ID, PID state, and current command.
 *
 *  vel_pid : PID_VELOCITY form ? output is current command
 *  pos_pid : PID_POSITION form ? output is velocity setpoint (cascaded outer loop)
 */
typedef struct {
    DJI_motor_model_t   model;
    uint16_t            can_id;
    int16_t             current_cmd;    /* Final current sent to CAN frame  */
    float               vel_setpoint;  /* RPM — set by position outer loop */
    PID_Info_TypeDef    vel_pid;
    PID_Info_TypeDef    pos_pid;
} DJI_Motor_t;

  /****************************/
 /*   3. PROTOCOL FUNCTIONS  */
/****************************/

/**
 * @brief Initialise FDCAN TX header for a DJI group frame.
 * @param msg         Pointer to DJI_Tx_message_t.
 * @param group_tx_id DJI_GROUP1_TX_ID or DJI_GROUP2_TX_ID.
 */
void DJI_init_tx_message(DJI_Tx_message_t *msg, uint32_t group_tx_id);

/**
 * @brief Pack current commands for up to 4 motors into an 8-byte DJI CAN frame.
 * @param msg        Pointer to DJI_Tx_message_t to populate.
 * @param currents   Array of int16_t current commands.
 * @param motor_ids  Array of motor CAN IDs matching currents[].
 * @param num_motors Number of entries (max DJI_MOTORS_PER_GROUP).
 */
void DJI_pack_tx_message(DJI_Tx_message_t *msg,
                         int16_t          *currents,
                         uint16_t         *motor_ids,
                         uint8_t           num_motors);

/**
 * @brief Transmit a packed DJI frame over FDCAN.
 * @param hfdcan FDCAN handle configured at 1Mbps classic CAN.
 * @param msg    Populated DJI_Tx_message_t.
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef DJI_transmit(FDCAN_HandleTypeDef *hfdcan, DJI_Tx_message_t *msg);

  /****************************/
 /*   4. CONTROL FUNCTIONS   */
/****************************/

/**
 * @brief Initialise a DJI motor control instance.
 * @param motor      Pointer to DJI_Motor_t.
 * @param model      DJI_MOTOR_M3508 or DJI_MOTOR_M2006.
 * @param can_id     Motor CAN ID (DJI_MOTOR_ID_1 … DJI_MOTOR_ID_8).
 * @param vel_params PID params [KP,KI,KD,Alpha,Deadband,LimitI,LimitOut] for velocity loop.
 * @param pos_params PID params for position outer loop, or NULL if not needed.
 */
void DJI_motor_init(DJI_Motor_t       *motor,
                    DJI_motor_model_t  model,
                    uint16_t           can_id,
                    float              vel_params[PID_PARAMETER_NUM],
                    float              pos_params[PID_PARAMETER_NUM]);

/**
 * @brief Velocity control — runs PID_VELOCITY on RPM error ? updates current_cmd.
 * @param motor    Pointer to DJI_Motor_t.
 * @param setpoint Target velocity (RPM at output shaft).
 * @param measured Measured velocity (RPM, gear ratio already applied).
 */
void DJI_motor_set_velocity(DJI_Motor_t *motor, float setpoint, float measured);

/**
 * @brief Position control — cascaded PID_POSITION (outer) ? PID_VELOCITY (inner) ? current_cmd.
 * @param motor        Pointer to DJI_Motor_t (must be init'd with pos_params).
 * @param setpoint_deg Target angle in degrees (0–360).
 * @param measured_deg Measured angle in degrees from encoder feedback.
 * @param measured_rpm Measured RPM for inner velocity loop.
 */
void DJI_motor_set_position(DJI_Motor_t *motor,
                            float        setpoint_deg,
                            float        measured_deg,
                            float        measured_rpm);

/**
 * @brief Direct current control — bypasses PID, clamps and sets current_cmd.
 * @param motor   Pointer to DJI_Motor_t.
 * @param current Desired current command (raw int16, clamped to motor limits).
 */
void DJI_motor_set_current(DJI_Motor_t *motor, int16_t current);

/**
 * @brief Pack and transmit all motors in group 1 (IDs 0x201–0x204) over FDCAN.
 * @param hfdcan     FDCAN handle.
 * @param motors     Array of DJI_Motor_t pointers for this group.
 * @param num_motors Number of motors in array (max 4).
 */
HAL_StatusTypeDef DJI_motor_transmit_group1(FDCAN_HandleTypeDef *hfdcan,
                                            DJI_Motor_t         *motors[],
                                            uint8_t              num_motors);

/**
 * @brief Pack and transmit all motors in group 2 (IDs 0x205–0x208) over FDCAN.
 */
HAL_StatusTypeDef DJI_motor_transmit_group2(FDCAN_HandleTypeDef *hfdcan,
                                            DJI_Motor_t         *motors[],
                                            uint8_t              num_motors);

#endif /* DJI_MOTOR_H */