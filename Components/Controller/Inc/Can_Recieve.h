#ifndef CAN_RECIEVE_H
#define CAN_RECIEVE_H

/**
 * @file    can_rx.h
 * @brief   Unified CAN RX feedback parser — DJI, CubeMars, and Damiao motors
 *          STM32H7 FDCAN
 *
 * Usage:
 *   Call CAN_RX_parse_frame() inside HAL_FDCAN_RxFifo0MsgPendingCallback()
 *   (or Fifo1, whichever is configured). The function identifies the motor
 *   type from the arbitration ID and populates the appropriate feedback struct.
 *
 * ID ranges:
 *   DJI     RX: 0x201–0x208  (M3508 and M2006)
 *   Damiao  RX: motor_id + 0x100  (e.g. 0x101 for motor 0x01)
 *   CubeMars RX: motor_id + 0x00  (same as TX ID — check by configured ID range)
 *
 * ? Configure your FDCAN filters in CubeMX to pass all relevant ID ranges
 *   to the same Rx FIFO before using this module.
 */

#include "main.h"
#include "dji_motor.h"
#include "cubemars_motor.h"
#include "damiao_motor.h"
#include <stdint.h>

  /**********************/
 /*      CONSTANTS     */
/**********************/

/* DJI RX ID range */
#define DJI_RX_ID_MIN           0x201U
#define DJI_RX_ID_MAX           0x208U

/* CubeMars RX ID range — adjust to match IDs set in CubeMars software */
#define CUBEMARS_RX_ID_MIN      0x001U
#define CUBEMARS_RX_ID_MAX      0x00BU

/* Damiao RX IDs = master ID + DM_RX_ID_OFFSET (0x100) */
#define DM_RX_ID_MIN            0x101U
#define DM_RX_ID_MAX            0x10BU

#define CAN_RX_FRAME_BYTES      8U

  /**********************/
 /*     DATA TYPES     */
/**********************/

/**
 * @brief   Motor type identifier — used to route RX frames to the correct decoder.
 */
typedef enum {
    MOTOR_TYPE_DJI      = 0,
    MOTOR_TYPE_CUBEMARS = 1,
    MOTOR_TYPE_DAMIAO   = 2,
    MOTOR_TYPE_UNKNOWN  = 3,
} CAN_motor_type_t;

/**
 * @brief   Generic feedback container — holds decoded data from any motor type.
 *          Only the fields relevant to the detected motor type are populated.
 */
typedef struct {
    CAN_motor_type_t    motor_type;
    uint32_t            can_id;

    /* DJI fields */
    uint16_t            dji_angle;          /* 0–8191 (mechanical angle)   */
    int16_t             dji_rpm;            /* RPM                         */
    int16_t             dji_current;        /* Raw current feedback        */
    uint8_t             dji_temperature;    /* degrees C                   */

    /* CubeMars / Damiao shared fields */
    float               position;           /* rad                         */
    float               velocity;           /* rad/s                       */
    float               torque;             /* Nm                          */
    uint8_t             temperature;        /* degrees C                   */
    uint8_t             error_flags;        /* 0 = no error                */

} CAN_Rx_feedback_t;

  /**********************/
 /*      FUNCTIONS     */
/**********************/

/**
 * @brief   Parse an incoming FDCAN RX frame and populate a feedback struct.
 *
 * @param   rx_header   Pointer to the FDCAN RX header (from HAL callback).
 * @param   rx_data     Pointer to the 8-byte data buffer (from HAL callback).
 * @param   feedback    Pointer to a CAN_Rx_feedback_t to populate.
 * @return  MOTOR_TYPE_DJI / MOTOR_TYPE_CUBEMARS / MOTOR_TYPE_DAMIAO / MOTOR_TYPE_UNKNOWN
 *
 * @note    Typical usage inside FDCAN callback:
 * @code
 *   void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan) {
 *       FDCAN_RxHeaderTypeDef rx_header;
 *       uint8_t rx_data[8];
 *       HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
 *       CAN_Rx_feedback_t fb;
 *       CAN_RX_parse_frame(&rx_header, rx_data, &fb);
 *   }
 * @endcode
 */
CAN_motor_type_t CAN_RX_parse_frame(FDCAN_RxHeaderTypeDef *rx_header,
                                    uint8_t               *rx_data,
                                    CAN_Rx_feedback_t     *feedback);

/**
 * @brief   Decode a DJI motor RX frame (0x201–0x208).
 *          Populates dji_angle, dji_rpm, dji_current, dji_temperature.
 */
void CAN_RX_decode_dji(uint32_t can_id, uint8_t *data, CAN_Rx_feedback_t *feedback);

/**
 * @brief   Decode a CubeMars motor RX frame.
 *          Populates position, velocity, torque, temperature, error_flags.
 */
void CAN_RX_decode_cubemars(uint32_t can_id, uint8_t *data, CAN_Rx_feedback_t *feedback);

/**
 * @brief   Decode a Damiao motor RX frame (motor_id + 0x100).
 *          Populates position, velocity, torque, temperature, error_flags.
 * @param   motor_type  DM_MOTOR_J6006 or DM_MOTOR_J4310 (needed for scaling).
 */
void CAN_RX_decode_damiao(uint32_t can_id, uint8_t *data,
                          DM_motor_type_t motor_type, CAN_Rx_feedback_t *feedback);

#endif /* CAN_RECIEVE_H */