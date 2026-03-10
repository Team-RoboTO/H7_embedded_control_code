#include "can_recieve.h"

/**
 * @file    can_rx.c
 * @brief   Unified CAN RX feedback parser — DJI, CubeMars, and Damiao motors
 *
 * DJI RX frame layout (8 bytes, per motor, ID 0x201–0x208):
 *   [0:1]  Mechanical angle   (uint16, 0–8191)
 *   [2:3]  RPM                (int16)
 *   [4:5]  Actual current     (int16)
 *   [6]    Temperature        (uint8, degrees C)
 *   [7]    Reserved
 *
 * CubeMars RX frame layout (8 bytes):
 *   [0]         Motor ID
 *   [1:2]       Position   (uint16, big-endian)
 *   [3] [7:4]   Velocity   (uint12, split)
 *   [3] [3:0]   + [4]
 *   [5] [7:4]   Current/Torque (uint12)
 *   [5] [3:0]   + [6]
 *   [7]         Temperature + error flags
 *
 * Damiao RX frame layout (8 bytes, ID = master_id + 0x100):
 *   [0:1]  Position   (uint16)
 *   [2:3]  Velocity   (uint12 split across nibbles)
 *   [4:5]  Torque     (uint12 split across nibbles)
 *   [6]    MOS temperature (uint8)
 *   [7]    Error flags
 */

  /********************************/
 /*   INTERNAL HELPER FUNCTIONS  */
/********************************/

/**
 * @brief  Convert a raw unsigned integer back to float in [val_min, val_max].
 */
static float uint_to_float(uint32_t raw, float val_min, float val_max, uint8_t bit_width)
{
    float max_raw = (float)((1 << bit_width) - 1);
    return val_min + (float)raw * (val_max - val_min) / max_raw;
}

  /**********************/
 /*      FUNCTIONS     */
/**********************/

void CAN_RX_decode_dji(uint32_t can_id, uint8_t *data, CAN_Rx_feedback_t *feedback)
{
    feedback->motor_type      = MOTOR_TYPE_DJI;
    feedback->can_id          = can_id;

    feedback->dji_angle       = (uint16_t)(data[0] << 8 | data[1]);
    feedback->dji_rpm         = (int16_t) (data[2] << 8 | data[3]);
    feedback->dji_current     = (int16_t) (data[4] << 8 | data[5]);
    feedback->dji_temperature = data[6];
}

void CAN_RX_decode_cubemars(uint32_t can_id, uint8_t *data, CAN_Rx_feedback_t *feedback)
{
    feedback->motor_type = MOTOR_TYPE_CUBEMARS;
    feedback->can_id     = can_id;

    /*
     * CubeMars RX bit layout:
     *   Byte 0         : motor ID (redundant, already in can_id)
     *   Byte 1         : position[15:8]
     *   Byte 2         : position[7:0]
     *   Byte 3 [7:4]   : velocity[11:8] ... actually velocity[11:4]
     *   Wait — CubeMars RX uses a slightly different packing than TX:
     *
     *   pos  = data[1]<<8 | data[2]                     (16-bit)
     *   vel  = data[3]<<4 | data[4]>>4                  (12-bit)
     *   tor  = (data[4]&0xF)<<8 | data[5]               (12-bit)
     *   temp = data[6]
     *   err  = data[7]
     */
    uint32_t p_raw = (uint32_t)(data[1] << 8)  | data[2];
    uint32_t v_raw = (uint32_t)(data[3] << 4)  | (data[4] >> 4);
    uint32_t t_raw = (uint32_t)((data[4] & 0xF) << 8) | data[5];

    feedback->position    = uint_to_float(p_raw, AK40_POS_MIN,    AK40_POS_MAX,    16);
    feedback->velocity    = uint_to_float(v_raw, AK40_VEL_MIN,    AK40_VEL_MAX,    12);
    feedback->torque      = uint_to_float(t_raw, AK40_TORQUE_MIN, AK40_TORQUE_MAX, 12);
    feedback->temperature = data[6];
    feedback->error_flags = data[7];
}

void CAN_RX_decode_damiao(uint32_t can_id, uint8_t *data,
                          DM_motor_type_t motor_type, CAN_Rx_feedback_t *feedback)
{
    feedback->motor_type = MOTOR_TYPE_DAMIAO;
    feedback->can_id     = can_id;

    /*
     * Damiao RX bit layout:
     *   [0:1]  position  (uint16)
     *   [2]    velocity[11:4]
     *   [3]    velocity[3:0] in upper nibble, torque[11:8] in lower nibble
     *   [4]    torque[7:0]
     *   [5]    unused / reserved
     *   [6]    MOS temperature
     *   [7]    error flags
     */
    uint32_t p_raw = (uint32_t)(data[0] << 8)  | data[1];
    uint32_t v_raw = (uint32_t)(data[2] << 4)  | (data[3] >> 4);
    uint32_t t_raw = (uint32_t)((data[3] & 0xF) << 8) | data[4];

    /* Select limits based on motor model */
    float pos_min, pos_max, vel_min, vel_max, tor_min, tor_max;
    if (motor_type == DM_MOTOR_J6006) {
        pos_min = DM_J6006_POS_MIN; pos_max = DM_J6006_POS_MAX;
        vel_min = DM_J6006_VEL_MIN; vel_max = DM_J6006_VEL_MAX;
        tor_min = DM_J6006_TORQUE_MIN; tor_max = DM_J6006_TORQUE_MAX;
    } else {
        pos_min = DM_J4310_POS_MIN; pos_max = DM_J4310_POS_MAX;
        vel_min = DM_J4310_VEL_MIN; vel_max = DM_J4310_VEL_MAX;
        tor_min = DM_J4310_TORQUE_MIN; tor_max = DM_J4310_TORQUE_MAX;
    }

    feedback->position    = uint_to_float(p_raw, pos_min, pos_max, 16);
    feedback->velocity    = uint_to_float(v_raw, vel_min, vel_max, 12);
    feedback->torque      = uint_to_float(t_raw, tor_min, tor_max, 12);
    feedback->temperature = data[6];
    feedback->error_flags = data[7];
}

CAN_motor_type_t CAN_RX_parse_frame(FDCAN_RxHeaderTypeDef *rx_header,
                                    uint8_t               *rx_data,
                                    CAN_Rx_feedback_t     *feedback)
{
    uint32_t id = rx_header->Identifier;

    if (id >= DJI_RX_ID_MIN && id <= DJI_RX_ID_MAX) {
        /* DJI M3508 / M2006 */
        CAN_RX_decode_dji(id, rx_data, feedback);
        return MOTOR_TYPE_DJI;
    }
    else if (id >= CUBEMARS_RX_ID_MIN && id <= CUBEMARS_RX_ID_MAX) {
        /* CubeMars AK40-10 */
        CAN_RX_decode_cubemars(id, rx_data, feedback);
        return MOTOR_TYPE_CUBEMARS;
    }
    else if (id >= DM_RX_ID_MIN && id <= DM_RX_ID_MAX) {
        /*
         * Damiao — motor type must be inferred from ID or a lookup table.
         * Here we default to J6006; replace with your actual ID-to-type map.
         * Example: if (id == 0x101 || id == 0x102) type = DM_MOTOR_J6006; else ...
         */
        DM_motor_type_t dm_type = DM_MOTOR_J6006;  /* TODO: map ID ? type at integration */
        CAN_RX_decode_damiao(id, rx_data, dm_type, feedback);
        return MOTOR_TYPE_DAMIAO;
    }

    /* Unknown ID */
    feedback->motor_type = MOTOR_TYPE_UNKNOWN;
    feedback->can_id     = id;
    return MOTOR_TYPE_UNKNOWN;
}