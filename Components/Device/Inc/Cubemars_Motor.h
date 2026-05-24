#ifndef CUBEMARS_MOTOR_H
#define CUBEMARS_MOTOR_H
#include "main.h"
#include "PID.h"
#include "stm32h723xx.h"
#include "bsp_can.h"
#include "motor.h"
/**
 * @file    cubemars_motor.h
 * @brief   CubeMars AK40-10 CAN protocol + motor control
 *          STM32H7 FDCAN, MIT mini-cheetah protocol, classic CAN, 1Mbps
 *
 *          Architecture mirrors Damiao_Motor: all TX/RX functions take a
 *          CM_Motor_Info_Typedef* so CAN IDs and param ranges come from the
 *          struct, not from hardcoded #defines.
 */

/**
 * @brief typedef enum that contains the type of CubeMars Motor Device.
 */
typedef enum {
    CM_AK40_10,
		CM_AK60_06,
    CM_MOTOR_TYPE_NUM,
} CM_Motor_Type_e;

/**
 * @brief typedef enum for CubeMars control modes.
 */
typedef enum {
    CM_MIT_MODE,
    CM_POSITION_MODE,
    CM_POSITION_SPEED_MODE,
    CM_CURRENT_MODE,
    CM_RPM_MODE,
} CM_Motor_Control_Mode_e;

/**
 * @brief typedef enum for CubeMars motor commands (mirrors DM_Motor_CMD_Type_e).
 */
typedef enum {
    CM_Motor_Enable,
    CM_Motor_Disable,
    CM_Motor_Save_Zero_Position,
    CM_Motor_CMD_Type_Num,
} CM_Motor_CMD_Type_e;

/**
 * @brief CubeMars extended-ID CAN packet commands.
 */
typedef enum {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD,
} CAN_PACKET_ID;

// --------------------------------------------------------------------------
//  Structs
// --------------------------------------------------------------------------

/**
 * @brief typedef structure for the motor FDCAN transmit and received identifiers.
 */
typedef struct {
    uint32_t TxIdentifier;   /*!< Specifies FDCAN transmit identifier (standard ID for MIT) */
    uint32_t RxIdentifier;   /*!< Specifies FDCAN received identifier (extended ID for feedback) */
} CM_Motor_CANFrameInfo_typedef;

/**
 * @brief typedef structure for CubeMars Motor parameter ranges.
 */
typedef struct {
    float P_MAX;
    float V_MAX;
    float T_MAX;
    float KP_MAX;
    float KD_MAX;
} CM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure for CubeMars Motor feedback data.
 */
typedef struct {
    bool     Initlized;       /*!< init flag */
    uint16_t P_int;           /*!< Motor position  uint16 (raw) */
    uint16_t V_int;           /*!< Motor velocity  uint16 (raw) */
    uint16_t T_int;           /*!< Motor torque    uint16 (raw) */
    float    Position;        /*!< Motor position (rad) */
    float    Velocity;        /*!< Motor velocity (rad/s) */
    float    Torque;          /*!< Motor torque / current (Nm) */
    int8_t   Temperature;     /*!< Motor temperature (C) */
    int8_t   Error;           /*!< Motor error code */
    float    Angle;           /*!< Motor angle (deg, unwrapped) */
} CM_Motor_Data_Typedef;

/**
 * @brief typedef structure for the CubeMars Motor Device information.
 */
typedef struct {
    CM_Motor_Type_e              Type;          /*!< Type of CubeMars Motor */
    CM_Motor_Control_Mode_e      Control_Mode;  /*!< Control mode */
    CM_Motor_CANFrameInfo_typedef FDCANFrame;   /*!< CAN frame identifiers */
    CM_Motor_Param_Range_Typedef Param_Range;   /*!< Parameter limits */
    CM_Motor_Data_Typedef        Data;          /*!< Feedback data */
} CM_Motor_Info_Typedef;

/**
 * @brief typedef structure for CubeMars Motor control setpoints.
 */
typedef struct {
    float Position;
    float Velocity;
    float KP;
    float KD;
    float Torque;
} CM_Motor_Control_Info_Typedef;

// --------------------------------------------------------------------------
//  Extern Variables
// --------------------------------------------------------------------------
extern CM_Motor_Info_Typedef         CM_Chassis_Motor[4];
extern CM_Motor_Info_Typedef         CM_Pitch_Motor;

// --------------------------------------------------------------------------
//  Core Functions (DaMiao-style: take motor struct pointer)
// --------------------------------------------------------------------------

/**
 * @brief  Update CubeMars motor feedback from CAN RX data (MIT mode).
 *         Mirrors DM_Motor_Info_Update().
 */
extern void CM_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, CM_Motor_Info_Typedef *CM_Motor);

/**
 * @brief  Send enable / disable / set-zero command to a CubeMars motor.
 *         Mirrors DM_Motor_Command().
 */
extern void CM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, CM_Motor_Info_Typedef *CM_Motor, uint8_t CMD);

/**
 * @brief  Transmit MIT control frame to a CubeMars motor.
 *         Mirrors DM_Motor_CAN_TxMessage().
 */
extern void CM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, CM_Motor_Info_Typedef *CM_Motor,
                                   float Position, float Velocity, float KP, float KD, float Torque);

// --------------------------------------------------------------------------
//  Extended-ID Mode Functions (position, current, rpm — also take motor ptr)
// --------------------------------------------------------------------------

/**
 * @brief  Send current (ampere) command via extended CAN ID.
 */
extern void CM_Motor_CAN_TxCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, CM_Motor_Info_Typedef *CM_Motor,
                                   float current_ampere);

/**
 * @brief  Send position command via extended CAN ID.
 */
extern void CM_Motor_CAN_TxPosition(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, CM_Motor_Info_Typedef *CM_Motor,
                                    float position);

/**
 * @brief  Send RPM command via extended CAN ID.
 */
extern void CM_Motor_CAN_TxRPM(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, CM_Motor_Info_Typedef *CM_Motor,
                                float rpm);

/**
 * @brief  Send position + speed + acceleration command via extended CAN ID.
 */
extern void CM_Motor_CAN_TxPosSpdAcc(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, CM_Motor_Info_Typedef *CM_Motor,
                                     float position, int16_t max_speed, int16_t acceleration);

/**
 * @brief  Send set-origin command via extended CAN ID.
 */
extern void CM_Motor_CAN_TxSetOrigin(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, CM_Motor_Info_Typedef *CM_Motor);

// --------------------------------------------------------------------------
//  Utility
// --------------------------------------------------------------------------
extern void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
extern void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index);

#endif /* CUBEMARS_MOTOR_H */