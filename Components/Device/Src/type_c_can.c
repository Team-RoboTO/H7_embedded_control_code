#include "type_c_can.h"

#include "fdcan.h"
#include "bsp_can.h"

#include <string.h>

extern FDCAN_TxFrame_TypeDef FDCAN3_TxFrame;

void H7_Can_Send(float value1, float value2)
{
    /* pack 2 floats into 8 bytes */
    memcpy(&FDCAN3_TxFrame.Data[0], &value1, 4);
    memcpy(&FDCAN3_TxFrame.Data[4], &value2, 4);

    /* set ID and send */
    FDCAN3_TxFrame.Header.Identifier  = H7_TO_DEWE_CAN_ID;
    FDCAN3_TxFrame.Header.IdType      = FDCAN_STANDARD_ID;
    FDCAN3_TxFrame.Header.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN3_TxFrame.Header.DataLength  = FDCAN_DLC_BYTES_8;

    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN3_TxFrame);
}

void Type_C_Can_Update(uint8_t *Data)
{
    memcpy(&Type_C_Can.value1, &Data[0], 4);
    memcpy(&Type_C_Can.value2, &Data[4], 4);
    Type_C_Can.rx_count++;
    Type_C_Can.Initlized = true;
}