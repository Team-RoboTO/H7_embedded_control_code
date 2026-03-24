#ifndef INA228DRIVER_H
#define INA228DRIVER_H

#include "stm32h7xx_hal.h"

// I2C address (A0=A1=GND)
#define INA228_I2C_ADDR (0x40 << 1)  // = 0x80 (HAL uses 8-bit address)

// Register Addresses
#define INA228_REG_CONFIG      0x00
#define INA228_REG_ADC_CONFIG  0x01
#define INA228_REG_SHUNT_CAL   0x02
#define INA228_REG_VSHUNT      0x04 // 24-bit
#define INA228_REG_VBUS        0x05 // 24-bit
#define INA228_REG_DIETEMP     0x06 // 16-bit
#define INA228_REG_CURRENT     0x07 // 24-bit
#define INA228_REG_POWER       0x08 // 24-bit
#define INA228_REG_ENERGY      0x09 // 40-bit
#define INA228_REG_CHARGE      0x0A // 40-bit
#define INA228_REG_DIAG_ALRT   0x0B
#define INA228_REG_ID          0x3F // Should read 0x2280 (16-bit)

// LSB constants for INA228
#define INA228_VBUS_LSB       0.0001953125f   // 195.3125 µV per bit
#define INA228_VSHUNT_LSB     0.0000003125f   // 312.5 nV per bit (ADCRANGE = 0)

// User Configurable settings
#define INA228_RSHUNT         0.005f          // 2 mOhm shunt
#define INA228_CURRENT_LSB    0.0001f          // 1000 µA per bit

// Power LSB is fixed at 3.2 * Current LSB for INA228
#define INA228_POWER_LSB      (INA228_CURRENT_LSB * 3.2f)

// Calibration value formula: CAL = 13107.2 * 10^6 * Current_LSB * Rshunt
// For 1mA LSB and 2mOhm shunt, CAL = 26214 (0x6666)
#define INA228_CAL_VALUE      ((uint16_t)(13107.2f * 1000000.0f * INA228_CURRENT_LSB * INA228_RSHUNT))

extern I2C_HandleTypeDef hi2c2;

// API
uint8_t INA228_Init(void);
uint8_t INA228_IsConnected(void);
float    INA228_ReadBusVoltage(void);
float    INA228_ReadShuntVoltage(void);
float    INA228_ReadCurrent(void);
float    INA228_ReadPower(void);

#endif // INA228DRIVER_H