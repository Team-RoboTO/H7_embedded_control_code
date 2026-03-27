#include "rtt_log.h"
#include "SEGGER_RTT.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define RTT_LOG_INTERVAL_MS 2
#define RTT_MAX_VALUES 15  // max floats per packet (60 bytes + 4 timestamp = 64)

#define RTT_LOG_MAGIC 0xAA

void RTT_Log(float *values, uint8_t count)
{
    static uint32_t last_log_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_log_time < RTT_LOG_INTERVAL_MS) return;
    last_log_time = current_time;

    if (count > RTT_MAX_VALUES) count = RTT_MAX_VALUES;

    uint8_t packet[2 + 4 + RTT_MAX_VALUES * 4];
    packet[0] = RTT_LOG_MAGIC;  // 0xAA
    packet[1] = count;
    memcpy(&packet[2], &current_time, 4);
    for (uint8_t i = 0; i < count; i++) {
        memcpy(&packet[6 + i * 4], &values[i], 4);
    }
    SEGGER_RTT_Write(0, packet, 2 + 4 + count * 4);
}