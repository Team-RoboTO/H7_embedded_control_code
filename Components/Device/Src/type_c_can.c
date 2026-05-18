#include "type_c_can.h"
#include <string.h>

void Type_C_Can_Update(uint8_t *Data)
{
    memcpy(&Type_C_Can.value1, &Data[0], 4);
    memcpy(&Type_C_Can.value2, &Data[4], 4);
    Type_C_Can.rx_count++;
    Type_C_Can.Initlized = true;
}