#include "cubemars_motor.h"
#include <string.h>

/**
 * @file    cubemars.c
 * @brief   CubeMars AK40-10 — CAN protocol + motor control
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

static HAL_StatusTypeDef send_special_command(FDCAN_HandleTypeDef *hfdcan,
                                              uint32_t             motor_id,
                                              uint64_t             command_word)
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
    uint8_t data[8];
    for (int i = 0; i < 8; i++)
        data[i] = (uint8_t)(command_word >> (56 - 8 * i));

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, data);
}

  /****************************/
 /*    COMM FUNCTIONS        */
/****************************/

void CUBEMARS_init_tx_message(CubeMars_Tx_message_t *msg, uint32_t motor_id)
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
    memset(msg->Tx_data, 0, CUBEMARS_CAN_FRAME_BYTES);
}

void CUBEMARS_pack_mit_frame(CubeMars_Tx_message_t *msg,
                             float position,
                             float velocity,
                             float kp,
                             float kd,
                             float torque)
{
    position = clamp_f(position, AK40_POS_MIN,    AK40_POS_MAX);
    velocity = clamp_f(velocity, AK40_VEL_MIN,    AK40_VEL_MAX);
    kp       = clamp_f(kp,       AK40_KP_MIN,     AK40_KP_MAX);
    kd       = clamp_f(kd,       AK40_KD_MIN,     AK40_KD_MAX);
    torque   = clamp_f(torque,   AK40_TORQUE_MIN, AK40_TORQUE_MAX);

    uint32_t p_raw  = float_to_uint(position, AK40_POS_MIN,    AK40_POS_MAX,    MIT_POS_BITS);
    uint32_t v_raw  = float_to_uint(velocity, AK40_VEL_MIN,    AK40_VEL_MAX,    MIT_VEL_BITS);
    uint32_t kp_raw = float_to_uint(kp,       AK40_KP_MIN,     AK40_KP_MAX,     MIT_KP_BITS);
    uint32_t kd_raw = float_to_uint(kd,       AK40_KD_MIN,     AK40_KD_MAX,     MIT_KD_BITS);
    uint32_t t_raw  = float_to_uint(torque,   AK40_TORQUE_MIN, AK40_TORQUE_MAX, MIT_TORQUE_BITS);

    msg->Tx_data[0] = (uint8_t)(p_raw >> 8);
    msg->Tx_data[1] = (uint8_t)(p_raw);
    msg->Tx_data[2] = (uint8_t)(v_raw >> 4);
    msg->Tx_data[3] = (uint8_t)((v_raw  & 0xF) << 4) | (uint8_t)(kp_raw >> 8);
    msg->Tx_data[4] = (uint8_t)(kp_raw);
    msg->Tx_data[5] = (uint8_t)(kd_raw >> 4);
    msg->Tx_data[6] = (uint8_t)((kd_raw & 0xF) << 4) | (uint8_t)(t_raw >> 8);
    msg->Tx_data[7] = (uint8_t)(t_raw);
}

HAL_StatusTypeDef CUBEMARS_enter_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id)
{
    return send_special_command(hfdcan, motor_id, CUBEMARS_CMD_ENTER_CONTROL);
}

HAL_StatusTypeDef CUBEMARS_exit_control(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id)
{
    return send_special_command(hfdcan, motor_id, CUBEMARS_CMD_EXIT_CONTROL);
}

HAL_StatusTypeDef CUBEMARS_set_zero(FDCAN_HandleTypeDef *hfdcan, uint32_t motor_id)
{
    return send_special_command(hfdcan, motor_id, CUBEMARS_CMD_SET_ZERO);
}

HAL_StatusTypeDef CUBEMARS_transmit(FDCAN_HandleTypeDef *hfdcan, CubeMars_Tx_message_t *msg)
{
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &msg->Tx_header, msg->Tx_data);
}

  /****************************/
 /*    CONTROL FUNCTIONS     */
/****************************/

void CUBEMARS_motor_init(CubeMars_Motor_t *motor,
                         uint32_t          can_id,
                         float             vel_params[PID_PARAMETER_NUM],
                         float             pos_params[PID_PARAMETER_NUM])
{
    memset(motor, 0, sizeof(CubeMars_Motor_t));
    motor->can_id = can_id;
    CUBEMARS_init_tx_message(&motor->tx_msg, can_id);

    if (vel_params != NULL) PID_Init(&motor->vel_pid, PID_VELOCITY, vel_params);
    if (pos_params != NULL) PID_Init(&motor->pos_pid, PID_POSITION, pos_params);
}

HAL_StatusTypeDef CUBEMARS_motor_set_velocity(CubeMars_Motor_t    *motor,
                                              FDCAN_HandleTypeDef *hfdcan,
                                              float                setpoint,
                                              float                measured)
{
    float vel_cmd = PID_Calculate(&motor->vel_pid, setpoint, measured);
    vel_cmd = clamp_f(vel_cmd, AK40_VEL_MIN, AK40_VEL_MAX);

    CUBEMARS_pack_mit_frame(&motor->tx_msg,
                            0.0f,       /* position  — unused in velocity mode */
                            vel_cmd,    /* velocity  — PID output              */
                            0.0f,       /* kp        — no onboard pos gain     */
                            0.0f,       /* kd        — no onboard damping      */
                            0.0f);      /* torque    — no feed-forward         */

    return CUBEMARS_transmit(hfdcan, &motor->tx_msg);
}

HAL_StatusTypeDef CUBEMARS_motor_set_position(CubeMars_Motor_t    *motor,
                                              FDCAN_HandleTypeDef *hfdcan,
                                              float                setpoint,
                                              float                measured)
{
    float pos_cmd = PID_Calculate(&motor->pos_pid, setpoint, measured);
    pos_cmd = clamp_f(pos_cmd, AK40_POS_MIN, AK40_POS_MAX);

    CUBEMARS_pack_mit_frame(&motor->tx_msg,
                            pos_cmd,            /* position  — PID output              */
                            0.0f,               /* velocity  — let motor handle it     */
                            AK40_DEFAULT_KP,    /* kp        — onboard stiffness       */
                            AK40_DEFAULT_KD,    /* kd        — onboard damping         */
                            0.0f);              /* torque    — no feed-forward         */

    return CUBEMARS_transmit(hfdcan, &motor->tx_msg);
}

HAL_StatusTypeDef CUBEMARS_motor_set_torque(CubeMars_Motor_t    *motor,
                                            FDCAN_HandleTypeDef *hfdcan,
                                            float                torque_nm)
{
    torque_nm = clamp_f(torque_nm, AK40_TORQUE_MIN, AK40_TORQUE_MAX);

    CUBEMARS_pack_mit_frame(&motor->tx_msg,
                            0.0f, 0.0f, 0.0f, 0.0f,
                            torque_nm); /* torque — direct command */

    return CUBEMARS_transmit(hfdcan, &motor->tx_msg);
}





