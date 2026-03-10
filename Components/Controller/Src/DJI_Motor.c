#include "dji_motor.h"
#include <string.h>

/**
 * @file    dji.c
 * @brief   DJI M3508 and M2006 — CAN protocol + motor control
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

static inline float dji_current_limit(DJI_motor_model_t model)
{
    return (model == DJI_MOTOR_M3508)
           ? (float)DJI_M3508_MAX_CURRENT
           : (float)DJI_M2006_MAX_CURRENT;
}

  /****************************/
 /*   COMM FUNCTIONS         */
/****************************/

void DJI_init_tx_message(DJI_Tx_message_t *msg, uint32_t group_tx_id)
{
    msg->Tx_header.Identifier          = group_tx_id;
    msg->Tx_header.IdType              = FDCAN_STANDARD_ID;
    msg->Tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    msg->Tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    msg->Tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    msg->Tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    msg->Tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    msg->Tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    msg->Tx_header.MessageMarker       = 0;
    memset(msg->Tx_data, 0, DJI_CAN_FRAME_BYTES);
}

void DJI_pack_tx_message(DJI_Tx_message_t *msg,
                         int16_t          *currents,
                         uint16_t         *motor_ids,
                         uint8_t           num_motors)
{
    memset(msg->Tx_data, 0, DJI_CAN_FRAME_BYTES);

    for (uint8_t i = 0; i < num_motors; i++) {
        switch (motor_ids[i]) {
            case DJI_MOTOR_ID_1: case DJI_MOTOR_ID_5:
                msg->Tx_data[0] = (uint8_t)(currents[i] >> 8);
                msg->Tx_data[1] = (uint8_t)(currents[i]);
                break;
            case DJI_MOTOR_ID_2: case DJI_MOTOR_ID_6:
                msg->Tx_data[2] = (uint8_t)(currents[i] >> 8);
                msg->Tx_data[3] = (uint8_t)(currents[i]);
                break;
            case DJI_MOTOR_ID_3: case DJI_MOTOR_ID_7:
                msg->Tx_data[4] = (uint8_t)(currents[i] >> 8);
                msg->Tx_data[5] = (uint8_t)(currents[i]);
                break;
            case DJI_MOTOR_ID_4: case DJI_MOTOR_ID_8:
                msg->Tx_data[6] = (uint8_t)(currents[i] >> 8);
                msg->Tx_data[7] = (uint8_t)(currents[i]);
                break;
            default:
                break;
        }
    }
}

HAL_StatusTypeDef DJI_transmit(FDCAN_HandleTypeDef *hfdcan, DJI_Tx_message_t *msg)
{
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &msg->Tx_header, msg->Tx_data);
}

  /****************************/
 /*      CONTROL FUNCTIONS   */
/****************************/

void DJI_motor_init(DJI_Motor_t       *motor,
                    DJI_motor_model_t  model,
                    uint16_t           can_id,
                    float              vel_params[PID_PARAMETER_NUM],
                    float              pos_params[PID_PARAMETER_NUM])
{
    memset(motor, 0, sizeof(DJI_Motor_t));
    motor->model  = model;
    motor->can_id = can_id;

    PID_Init(&motor->vel_pid, PID_VELOCITY, vel_params);
    if (pos_params != NULL)
        PID_Init(&motor->pos_pid, PID_POSITION, pos_params);
}

void DJI_motor_set_velocity(DJI_Motor_t *motor, float setpoint, float measured)
{
    float output = PID_Calculate(&motor->vel_pid, setpoint, measured);
    float limit  = dji_current_limit(motor->model);
    motor->current_cmd = (int16_t)clamp_f(output, -limit, limit);
}

void DJI_motor_set_position(DJI_Motor_t *motor,
                            float        setpoint_deg,
                            float        measured_deg,
                            float        measured_rpm)
{
    /* Outer loop: position error ? velocity setpoint */
    float vel_setpoint = PID_Calculate(&motor->pos_pid, setpoint_deg, measured_deg);
    motor->vel_setpoint = vel_setpoint;

    /* Inner loop: velocity error ? current command */
    DJI_motor_set_velocity(motor, vel_setpoint, measured_rpm);
}

void DJI_motor_set_current(DJI_Motor_t *motor, int16_t current)
{
    float limit = dji_current_limit(motor->model);
    motor->current_cmd = (int16_t)clamp_f((float)current, -limit, limit);
}

HAL_StatusTypeDef DJI_motor_transmit_group1(FDCAN_HandleTypeDef *hfdcan,
                                            DJI_Motor_t         *motors[],
                                            uint8_t              num_motors)
{
    DJI_Tx_message_t msg;
    DJI_init_tx_message(&msg, DJI_GROUP1_TX_ID);

    int16_t  currents[DJI_MOTORS_PER_GROUP];
    uint16_t ids[DJI_MOTORS_PER_GROUP];
    for (uint8_t i = 0; i < num_motors && i < DJI_MOTORS_PER_GROUP; i++) {
        currents[i] = motors[i]->current_cmd;
        ids[i]      = motors[i]->can_id;
    }

    DJI_pack_tx_message(&msg, currents, ids, num_motors);
    return DJI_transmit(hfdcan, &msg);
}

HAL_StatusTypeDef DJI_motor_transmit_group2(FDCAN_HandleTypeDef *hfdcan,
                                            DJI_Motor_t         *motors[],
                                            uint8_t              num_motors)
{
    DJI_Tx_message_t msg;
    DJI_init_tx_message(&msg, DJI_GROUP2_TX_ID);

    int16_t  currents[DJI_MOTORS_PER_GROUP];
    uint16_t ids[DJI_MOTORS_PER_GROUP];
    for (uint8_t i = 0; i < num_motors && i < DJI_MOTORS_PER_GROUP; i++) {
        currents[i] = motors[i]->current_cmd;
        ids[i]      = motors[i]->can_id;
    }

    DJI_pack_tx_message(&msg, currents, ids, num_motors);
    return DJI_transmit(hfdcan, &msg);
}