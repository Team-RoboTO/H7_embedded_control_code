#ifndef TYPE_C_CAN_H
#define TYPE_C_CAN_H

#include "stdint.h"
#include "stdbool.h"

#define TYPE_C_CAN_ID    0x100
#define H7_TO_DEWE_CAN_ID    0x101


typedef struct {
    float    value1;
    float    value2;
    uint32_t rx_count;
    bool     Initlized;
} Type_C_Can_t;

extern Type_C_Can_t Type_C_Can;

void Type_C_Can_Update(uint8_t *Data);

void H7_Can_Send(float value1, float value2);


#endif