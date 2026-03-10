#include "damiao_motor.h"
#include <string.h>

/**
 * @file    damiao.c
 * @brief   Damiao DM-J6006-2EC and DM-J4310-2EC — CAN protocol + motor control
 *
 * MIT TX frame byte layout:
 *   [0]      = position[15:8]
 *   [1]      = position[7:0]
 *   [2]      = velocity[11:4]
 *   [3]      = velocity[3:0] << 4 | kp[11:8]
 *   [4]      = kp[7:0]
 *   [5]      = kd[11:4]
 *   [6]      = kd[3:0] << 4 | torque[11:8]
 *   [7]      = torque[7:0]
 *
 * Special command frame layout:
 *   Bytes [0:6] = 0xFF
 *   Byte  [7]   = command byte (DM_CMD_*)
 */

  /****************************/
 /*   INTERNAL HELPERS       */
/****************************/

static inline float clamp_f(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static uint32_t float_to_uint(float val, float val_min, float val_max, uint8_t bit_width)
{
    float span    = val_max - val_min;
    float max_raw = (float)((1 << bit_width) - 1);
    return (uint32_t)((val - val_min) * max_raw / span);
}

static void get_motor_limits(DM_motor_type_t  motor_type,
                             float *pos_min, float *pos_max,
                             float *vel_min, float *vel_max,
                             float *tor_min, float *tor_max)
{
    if (motor_type == DM_MOTOR_J6006) {
        *pos_min = DM_J6006_POS_MIN;    *pos_max = DM_J6006_POS_MAX;
        *vel_min = DM_J6006_VEL_MIN;    *vel_max = DM_J6006_VEL_MAX;
        *tor_min = DM_J6006_TORQUE_MIN; *tor_max = DM_J6006_TORQUE_MAX;
    } else {
        *pos_min = DM_J4310_POS_MIN;    *pos_max = DM_J4310_POS_MAX;
        *vel_min = DM_J4310_VEL_MIN;    *vel_max = DM_J4310_VEL_MAX;
        *tor_min = DM_J4310_TORQUE_MIN; *tor_max = DM_J4310_TORQUE_MAX;
    }
}

static HAL_StatusTypeDef send_special_command(FDCAN_HandleTypeDef *hfdcan,
                                              uint32_t             motor_id,
                                              uint8_t              cmd_byte)
{
    FDCAN_TxHeaderTypeDef header = {
        .Identifier          = motor_id,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
    uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd_byte };
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, data);
}

  /****************************/
 /*  COMM FUNCTIONS          */
/****************************/

void DM_init_tx_message(DM_Tx_message_t *msg, uint32_t motor_id)
{
    msg->Tx_header.Identifier          = motor_id;
    msg->Tx_header.IdType              = FDCAN_STANDARD_ID;
    msg->Tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    msg->Tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    msg->Tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    msg->Tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    msg->Tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    msg->Tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    msg->Tx_header.MessageMarker       = 0;
    memset(msg->Tx_data, 0, DM_CAN_FRAME_BYTES);
}

void DM_pack_mit_frame(DM_Tx_message_t *msg,
                       DM_motor_type_t  motor_type,
                       float            position,
                       float            velocity,
                       float            kp,
                       float            kd,
                       float            torque)
{
    float pos_min, pos_max, vel_min, vel_max, tor_min, tor_max;
    get_motor_limits(motor_type, &pos_min, &pos_max, &vel_min, &vel_max, &tor_min, &tor_max);

    position = clamp_f(position, pos_min, pos_max);
    velocity = clamp_f(velocity, vel_min, vel_max);
    kp       = clamp_f(kp,       DM_KP_MIN,  DM_KP_MAX);
    kd       = clamp_f(kd,       DM_KD_MIN,  DM_KD_MAX);
    torque   = clamp_f(torque,   tor_min,    tor_max);

    uint32_t p_raw  = float_to_uint(position, pos_min, pos_max, DM_POS_BITS);
    uint32_t v_raw  = float_to_uint(velocity, vel_min, vel_max, DM_VEL_BITS);
    uint32_t kp_raw = float_to_uint(kp,       DM_KP_MIN, DM_KP_MAX, DM_KP_BITS);
    uint32_t kd_raw = float_to_uint(kd,       DM_KD_MIN, DM_KD_MAX, DM_KD_BITS);
    uint32_t t_raw  = float_to_uint(torque,   tor_min,   tor_max,   DM_TORQUE_BITS);

    msg->Tx_data[0] = (uint8_t)(p_raw >> 8);
    msg->Tx_data[1] = (uint8_t)(p_raw);
    msg->Tx_data[2] = (uint8_t)(v_raw >> 4);
    msg->Tx_data[3] = (uint8_t)((v_raw  & 0xF) << 4) | (uint8_t)(kp_raw >> 8);
    msg->Tx_data[4] = (uint8_t)(kp_raw);
    msg->Tx_data[5] = (uint8_t)(kd_raw >> 4);
    msg->Tx_data[6] = (uint8_t)((kd_raw & 0xF) << 4) | (uint8_t)(t_raw >> 8);
    msg->Tx_data[7] = (uint8_t)(t_raw);
}

HAL_StatusTypeDef DM_enter_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id)
{
    return send_special_command(hfdcan, motor_id, DM_CMD_ENTER_CONTROL);
}

HAL_StatusTypeDef DM_exit_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id)
{
    return send_special_command(hfdcan, motor_id, DM_CMD_EXIT_CONTROL);
}

HAL_StatusTypeDef DM_set_zero(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id)
{
    return send_special_command(hfdcan, motor_id, DM_CMD_SET_ZERO);
}

HAL_StatusTypeDef DM_clear_error(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id)
{
    return send_special_command(hfdcan, motor_id, DM_CMD_CLEAR_ERROR);
}

HAL_StatusTypeDef DM_transmit(FDCAN_HandleTypeDef *hfdcan, DM_Tx_message_t *msg)
{
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &msg->Tx_header, msg->Tx_data);
}

  /****************************/
 /*      CONTROL FUNCTIONS   */
/****************************/

void DM_motor_init(DM_Motor_t      *motor,
                   DM_motor_type_t  motor_type,
                   uint32_t         can_id,
                   float            vel_params[PID_PARAMETER_NUM],
                   float            pos_params[PID_PARAMETER_NUM])
{
    memset(motor, 0, sizeof(DM_Motor_t));
    motor->can_id     = can_id;
    motor->motor_type = motor_type;
    DM_init_tx_message(&motor->tx_msg, can_id);

    if (vel_params != NULL) PID_Init(&motor->vel_pid, PID_VELOCITY, vel_params);
    if (pos_params != NULL) PID_Init(&motor->pos_pid, PID_POSITION, pos_params);
}

HAL_StatusTypeDef DM_motor_set_velocity(DM_Motor_t          *motor,
                                        FDCAN_HandleTypeDef *hfdcan,
                                        float                setpoint,
                                        float                measured)
{
    float vel_cmd = PID_Calculate(&motor->vel_pid, setpoint, measured);

    float vel_min = (motor->motor_type == DM_MOTOR_J6006) ? DM_J6006_VEL_MIN : DM_J4310_VEL_MIN;
    float vel_max = (motor->motor_type == DM_MOTOR_J6006) ? DM_J6006_VEL_MAX : DM_J4310_VEL_MAX;
    vel_cmd = clamp_f(vel_cmd, vel_min, vel_max);

    DM_pack_mit_frame(&motor->tx_msg, motor->motor_type,
                      0.0f, vel_cmd, 0.0f, 0.0f, 0.0f);

    return DM_transmit(hfdcan, &motor->tx_msg);
}

HAL_StatusTypeDef DM_motor_set_position(DM_Motor_t          *motor,
                                        FDCAN_HandleTypeDef *hfdcan,
                                        float                setpoint,
                                        float                measured)
{
    float pos_cmd = PID_Calculate(&motor->pos_pid, setpoint, measured);

    float pos_min = (motor->motor_type == DM_MOTOR_J6006) ? DM_J6006_POS_MIN : DM_J4310_POS_MIN;
    float pos_max = (motor->motor_type == DM_MOTOR_J6006) ? DM_J6006_POS_MAX : DM_J4310_POS_MAX;
    pos_cmd = clamp_f(pos_cmd, pos_min, pos_max);

    DM_pack_mit_frame(&motor->tx_msg, motor->motor_type,
                      pos_cmd, 0.0f, DM_DEFAULT_KP, DM_DEFAULT_KD, 0.0f);

    return DM_transmit(hfdcan, &motor->tx_msg);
}

HAL_StatusTypeDef DM_motor_set_torque(DM_Motor_t          *motor,
                                      FDCAN_HandleTypeDef *hfdcan,
                                      float                torque_nm)
{
    float tor_min = (motor->motor_type == DM_MOTOR_J6006) ? DM_J6006_TORQUE_MIN : DM_J4310_TORQUE_MIN;
    float tor_max = (motor->motor_type == DM_MOTOR_J6006) ? DM_J6006_TORQUE_MAX : DM_J4310_TORQUE_MAX;
    torque_nm = clamp_f(torque_nm, tor_min, tor_max);

    DM_pack_mit_frame(&motor->tx_msg, motor->motor_type,
                      0.0f, 0.0f, 0.0f, 0.0f, torque_nm);

    return DM_transmit(hfdcan, &motor->tx_msg);
}