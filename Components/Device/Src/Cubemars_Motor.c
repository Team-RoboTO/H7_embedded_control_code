#include "cubemars_motor.h"
#include <string.h>
#include <math.h>
#include "stdint.h"

/**
 * @file    cubemars_motor.c
 * @brief   CubeMars AK40-10 CAN protocol + motor control
 *
 *          Refactored to follow DaMiao architecture: all TX/RX functions
 *          take a CM_Motor_Info_Typedef* pointer. CAN identifiers and
 *          parameter ranges are read from the struct, never hardcoded.
 *
 *          CubeMars data packing is preserved exactly as-is.
 */

// ============================================================================
//  Internal Helpers
// ============================================================================

static float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
    float span   = X_max - X_min;
    float offset = X_min;
    return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Buffer helpers for extended-ID protocols
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index)
{
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

// ============================================================================
//  Internal CAN transmit wrappers
// ============================================================================

/**
 * @brief  Transmit a standard-ID CAN frame (used for MIT mode).
 *         Uses the motor's TxIdentifier as the standard CAN ID.
 */
static void CM_CAN_Transmit_SID(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                                CM_Motor_Info_Typedef *CM_Motor,
                                const uint8_t *data, uint8_t len)
{
    uint8_t i;
    FDCAN_TxFrame->Header.Identifier    = CM_Motor->FDCANFrame.TxIdentifier;
    FDCAN_TxFrame->Header.IdType        = FDCAN_STANDARD_ID;
    FDCAN_TxFrame->Header.TxFrameType   = FDCAN_DATA_FRAME;
    FDCAN_TxFrame->Header.DataLength    = FDCAN_DLC_BYTES_8;
    FDCAN_TxFrame->Header.FDFormat      = FDCAN_CLASSIC_CAN;
    FDCAN_TxFrame->Header.BitRateSwitch = FDCAN_BRS_OFF;
    for (i = 0; i < len; i++) FDCAN_TxFrame->Data[i] = data[i];
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
 * @brief  Transmit an extended-ID CAN frame (used for position/current/rpm modes).
 *         Builds the extended ID from TxIdentifier | (packet_cmd << 8).
 */
static void CM_CAN_Transmit_EID(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                                CM_Motor_Info_Typedef *CM_Motor,
                                CAN_PACKET_ID packet_cmd,
                                const uint8_t *data, uint8_t len)
{
    uint8_t i;
    FDCAN_TxFrame->Header.Identifier    = CM_Motor->FDCANFrame.TxIdentifier |
                                          ((uint32_t)packet_cmd << 8);
    FDCAN_TxFrame->Header.IdType        = FDCAN_EXTENDED_ID;
    FDCAN_TxFrame->Header.TxFrameType   = FDCAN_DATA_FRAME;
    FDCAN_TxFrame->Header.DataLength    = (len <= 4) ? FDCAN_DLC_BYTES_4 : FDCAN_DLC_BYTES_8;
    FDCAN_TxFrame->Header.FDFormat      = FDCAN_CLASSIC_CAN;
    FDCAN_TxFrame->Header.BitRateSwitch = FDCAN_BRS_OFF;
    for (i = 0; i < len; i++) FDCAN_TxFrame->Data[i] = data[i];
    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

// ============================================================================
//  CM_Motor_Command  (mirrors DM_Motor_Command)
// ============================================================================

/**
 * @brief  Transmit enable / disable / save-zero-position command.
 * @param  *FDCAN_TxFrame  pointer to the FDCAN TX frame
 * @param  *CM_Motor       pointer to the CubeMars motor struct
 * @param  CMD             command (CM_Motor_CMD_Type_e)
 */
void CM_Motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                      CM_Motor_Info_Typedef *CM_Motor,
                      uint8_t CMD)
{
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;

    switch (CMD) {
        case CM_Motor_Enable:
            data[7] = 0xFC;
            break;
        case CM_Motor_Disable:
            data[7] = 0xFD;
            break;
        case CM_Motor_Save_Zero_Position:
            data[7] = 0xFE;
            break;
        default:
            return;
    }

    CM_CAN_Transmit_SID(FDCAN_TxFrame, CM_Motor, data, 8);
}

// ============================================================================
//  CM_Motor_CAN_TxMessage  (mirrors DM_Motor_CAN_TxMessage)
// ============================================================================

/**
 * @brief  Transmit MIT / Position / Current / RPM control frame.
 *         Dispatches based on CM_Motor->Control_Mode, exactly like
 *         DM_Motor_CAN_TxMessage dispatches on DM_Motor->Control_Mode.
 *
 * @param  *FDCAN_TxFrame  pointer to the FDCAN TX frame
 * @param  *CM_Motor       pointer to the CubeMars motor struct
 * @param  Position        target position  (rad)           — MIT mode
 * @param  Velocity        target velocity  (rad/s)         — MIT mode
 * @param  KP              position gain                    — MIT mode
 * @param  KD              velocity gain                    — MIT mode
 * @param  Torque          feed-forward torque (Nm)         — MIT mode
 */
void CM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                            CM_Motor_Info_Typedef *CM_Motor,
                            float Position, float Velocity,
                            float KP, float KD, float Torque)
{
    if (CM_Motor->Control_Mode == CM_MIT_MODE)
    {
        /* --- Clamp to param ranges --- */
        Position = fminf(fmaxf(-CM_Motor->Param_Range.P_MAX, Position), CM_Motor->Param_Range.P_MAX);
        Velocity = fminf(fmaxf(-CM_Motor->Param_Range.V_MAX, Velocity), CM_Motor->Param_Range.V_MAX);
        KP       = fminf(fmaxf(0.0f, KP), CM_Motor->Param_Range.KP_MAX);
        KD       = fminf(fmaxf(0.0f, KD), CM_Motor->Param_Range.KD_MAX);
        Torque   = fminf(fmaxf(-CM_Motor->Param_Range.T_MAX, Torque), CM_Motor->Param_Range.T_MAX);

        /* --- Convert floats to unsigned ints --- */
        uint16_t Position_Tmp = float_to_uint(Position, -CM_Motor->Param_Range.P_MAX, CM_Motor->Param_Range.P_MAX, 16);
        uint16_t Velocity_Tmp = float_to_uint(Velocity, -CM_Motor->Param_Range.V_MAX, CM_Motor->Param_Range.V_MAX, 12);
        uint16_t KP_Tmp       = float_to_uint(KP, 0.0f, CM_Motor->Param_Range.KP_MAX, 12);
        uint16_t KD_Tmp       = float_to_uint(KD, 0.0f, CM_Motor->Param_Range.KD_MAX, 12);
        uint16_t Torque_Tmp   = float_to_uint(Torque, -CM_Motor->Param_Range.T_MAX, CM_Motor->Param_Range.T_MAX, 12);

        /* --- Pack into CAN buffer (CubeMars MIT format) --- */
        uint8_t data[8];
        data[0] = (uint8_t)(Position_Tmp >> 8);                                         // Position High 8
        data[1] = (uint8_t)(Position_Tmp & 0xFF);                                       // Position Low 8
        data[2] = (uint8_t)(Velocity_Tmp >> 4);                                         // Velocity High 8
        data[3] = (uint8_t)(((Velocity_Tmp & 0xF) << 4) | (KP_Tmp >> 8));               // Velocity Low 4 | KP High 4
        data[4] = (uint8_t)(KP_Tmp & 0xFF);                                             // KP Low 8
        data[5] = (uint8_t)(KD_Tmp >> 4);                                               // KD High 8
        data[6] = (uint8_t)(((KD_Tmp & 0xF) << 4) | (Torque_Tmp >> 8));                 // KD Low 4 | Torque High 4
        data[7] = (uint8_t)(Torque_Tmp & 0xFF);                                         // Torque Low 8

        CM_CAN_Transmit_SID(FDCAN_TxFrame, CM_Motor, data, 8);
    }
    else if (CM_Motor->Control_Mode == CM_POSITION_MODE)
    {
        /* Position via extended CAN ID */
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)(Position * 10000.0f), &send_index);
        CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_POS, buffer, (uint8_t)send_index);
    }
    else if (CM_Motor->Control_Mode == CM_POSITION_SPEED_MODE)
    {
        /* Position + speed + acceleration via extended CAN ID */
        int32_t send_index = 0;
        uint8_t buffer[8];
        buffer_append_int32(buffer, (int32_t)(Position * 10000.0f), &send_index);
        buffer_append_int16(buffer, (int16_t)(Velocity / 10.0f), (int16_t*)&send_index);
        buffer_append_int16(buffer, (int16_t)(Torque / 10.0f), (int16_t*)&send_index);  /* Torque arg repurposed as acceleration */
        CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_POS_SPD, buffer, (uint8_t)send_index);
    }
    else if (CM_Motor->Control_Mode == CM_CURRENT_MODE)
    {
        /* Current (mA) via extended CAN ID */
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)(Torque), &send_index);  /* Torque arg repurposed as current */
        CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_CURRENT, buffer, (uint8_t)send_index);
    }
    else if (CM_Motor->Control_Mode == CM_RPM_MODE)
    {
        /* RPM via extended CAN ID */
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)(Velocity), &send_index);  /* Velocity arg as RPM */
        CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_RPM, buffer, (uint8_t)send_index);
    }
}

// ============================================================================
//  CM_Motor_Info_Update  (mirrors DM_Motor_Info_Update)
// ============================================================================

/**
 * @brief  Update CubeMars motor feedback from CAN RX data.
 * @param  Identifier  pointer to the received CAN identifier
 * @param  Rx_Buf      pointer to the 8-byte CAN receive buffer
 * @param  CM_Motor    pointer to the CubeMars motor struct
 *
 * CubeMars MIT feedback format (8 bytes):
 *   [0]       Motor ID
 *   [1..2]    Position   (16-bit)
 *   [3..4]    Velocity   (12-bit) | Torque high (4-bit)
 *   [4..5]    Torque     (12-bit)
 *   [6]       Temperature
 */
void CM_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, CM_Motor_Info_Typedef *CM_Motor)
{
    if (*Identifier != CM_Motor->FDCANFrame.RxIdentifier) return;

    /* Unpack raw ints (CubeMars MIT feedback layout) */
    CM_Motor->Data.P_int = ((uint16_t)Rx_Buf[1] << 8) | (uint16_t)Rx_Buf[2];
    CM_Motor->Data.V_int = ((uint16_t)Rx_Buf[3] << 4) | ((uint16_t)Rx_Buf[4] >> 4);
    CM_Motor->Data.T_int = (((uint16_t)Rx_Buf[4] & 0x0F) << 8) | (uint16_t)Rx_Buf[5];

    /* Convert to floats using struct param ranges */
    CM_Motor->Data.Position = uint_to_float(CM_Motor->Data.P_int,
                                            -CM_Motor->Param_Range.P_MAX,
                                             CM_Motor->Param_Range.P_MAX, 16);
    CM_Motor->Data.Velocity = uint_to_float(CM_Motor->Data.V_int,
                                            -CM_Motor->Param_Range.V_MAX,
                                             CM_Motor->Param_Range.V_MAX, 12);
    CM_Motor->Data.Torque   = uint_to_float(CM_Motor->Data.T_int,
                                            -CM_Motor->Param_Range.T_MAX,
                                             CM_Motor->Param_Range.T_MAX, 12);

    CM_Motor->Data.Temperature = (int8_t)Rx_Buf[6];

	
		CM_Motor->Data.Error = (int8_t)Rx_Buf[7];
		
    CM_Motor->Data.Initlized = true;
}

// ============================================================================
//  Convenience wrappers (extended-ID modes, also take motor ptr)
// ============================================================================

/**
 * @brief  Send current command via extended CAN ID.
 */
void CM_Motor_CAN_TxCurrent(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                            CM_Motor_Info_Typedef *CM_Motor,
                            float current_ampere)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current_ampere), &send_index);
    CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_CURRENT, buffer, (uint8_t)send_index);
}

/**
 * @brief  Send position command via extended CAN ID.
 */
void CM_Motor_CAN_TxPosition(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                             CM_Motor_Info_Typedef *CM_Motor,
                             float position)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(position * 10000.0f), &send_index);
    CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_POS, buffer, (uint8_t)send_index);
}

/**
 * @brief  Send RPM command via extended CAN ID.
 */
void CM_Motor_CAN_TxRPM(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                         CM_Motor_Info_Typedef *CM_Motor,
                         float rpm)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_RPM, buffer, (uint8_t)send_index);
}

/**
 * @brief  Send position + speed + acceleration command via extended CAN ID.
 */
void CM_Motor_CAN_TxPosSpdAcc(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                              CM_Motor_Info_Typedef *CM_Motor,
                              float position, int16_t max_speed, int16_t acceleration)
{
    int32_t send_index = 0;
    uint8_t buffer[8];
    buffer_append_int32(buffer, (int32_t)(position * 10000.0f), &send_index);
    buffer_append_int16(buffer, (int16_t)(max_speed / 10.0f), (int16_t*)&send_index);
    buffer_append_int16(buffer, (int16_t)(acceleration / 10.0f), (int16_t*)&send_index);
    CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_POS_SPD, buffer, (uint8_t)send_index);
}

/**
 * @brief  Send set-origin command via extended CAN ID.
 */
void CM_Motor_CAN_TxSetOrigin(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                              CM_Motor_Info_Typedef *CM_Motor)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(1), &send_index);
    CM_CAN_Transmit_EID(FDCAN_TxFrame, CM_Motor, CAN_PACKET_SET_ORIGIN_HERE, buffer, (uint8_t)send_index);
}