#include "INA228.h"
#include "string.h"

// NOTE: hi2c2 must be defined elsewhere (e.g. main.c / ioc)
extern I2C_HandleTypeDef hi2c2;

// Helper: read 2 bytes (MSB first) from register (16-bit registers)
static HAL_StatusTypeDef INA228_ReadReg16(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, INA228_I2C_ADDR, reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
    if (st != HAL_OK) return st;
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return HAL_OK;
}

// Helper: write 2 bytes (MSB first) to register (16-bit registers)
static HAL_StatusTypeDef INA228_WriteReg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
    return HAL_I2C_Mem_Write(&hi2c2, INA228_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
}

// Helper: read 3 bytes (MSB first) from register (24-bit registers)
static HAL_StatusTypeDef INA228_ReadReg24(uint8_t reg, uint32_t *out)
{
    uint8_t buf[3];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, INA228_I2C_ADDR, reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, 3, HAL_MAX_DELAY);
    if (st != HAL_OK) return st;
    *out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return HAL_OK;
}

uint8_t INA228_IsConnected(void)
{
    // 3 trials, 100 ms timeout each
    return (HAL_I2C_IsDeviceReady(&hi2c2, INA228_I2C_ADDR, 3, 100) == HAL_OK) ? 1 : 0;
}

uint8_t INA228_Init(void)
{
    if (!INA228_IsConnected()) return 1;

    // 1. CONFIG Register (Default)
    if (INA228_WriteReg16(INA228_REG_CONFIG, 0x0000) != HAL_OK) return 2;

    // 2. ADC_CONFIG Register - TUNE YOUR AVERAGE HERE
    uint16_t mode    = 0xB;  // Continuous Shunt, Bus, Temp
    uint16_t vbus_ct = 0x3;  // VBUS conversion time = 1052us
    uint16_t vsh_ct  = 0x3;  // Shunt conversion time = 1052us
    uint16_t vt_ct   = 0x3;  // Temp conversion time = 1052us
    
    // CHANGE THIS VARIABLE TO TUNE THE AVERAGE (MEDIA):
    uint16_t avg     = 0x3;  // 0x3 = 64 samples average
    
    // Combine them into a single 16-bit word
    uint16_t adcConfigValue = (mode << 12) | (vbus_ct << 9) | (vsh_ct << 6) | (vt_ct << 3) | avg;

    if (INA228_WriteReg16(INA228_REG_ADC_CONFIG, adcConfigValue) != HAL_OK) return 3;

    // 3. Write Calibration
    uint16_t calValue = INA228_CAL_VALUE;
    if (INA228_WriteReg16(INA228_REG_SHUNT_CAL, calValue) != HAL_OK) return 4;

    return 0; // OK
}

float INA228_ReadBusVoltage(void)
{
    uint32_t raw24 = 0;
    if (INA228_ReadReg24(INA228_REG_VBUS, &raw24) != HAL_OK) return -1.0f;
    
    // VBUS is a 20-bit unsigned value, shifted by 4 bits (data in bits 23:4)
    uint32_t val20 = raw24 >> 4;
    return ((float)val20) * INA228_VBUS_LSB;
}

float INA228_ReadShuntVoltage(void)
{
    uint32_t raw24 = 0;
    if (INA228_ReadReg24(INA228_REG_VSHUNT, &raw24) != HAL_OK) return -1.0f;
    
    // VSHUNT is a 20-bit signed two's complement value (data in bits 23:4)
    int32_t val20 = raw24 >> 4;
    
    // Sign extend from the 20th bit (bit 19)
    if (val20 & 0x080000) {
        val20 |= 0xFFF00000;
    }
    
    return ((float)val20) * INA228_VSHUNT_LSB;
}

float INA228_ReadCurrent(void)
{
    uint32_t raw24 = 0;
    if (INA228_ReadReg24(INA228_REG_CURRENT, &raw24) != HAL_OK) return -1.0f;
    
    // CURRENT is a 20-bit signed two's complement value (data in bits 23:4)
    int32_t val20 = raw24 >> 4;
    
    // Sign extend from the 20th bit (bit 19)
    if (val20 & 0x080000) {
        val20 |= 0xFFF00000;
    }
    
    return ((float)val20) * INA228_CURRENT_LSB;
}

float INA228_ReadPower(void)
{
    uint32_t raw24 = 0;
    if (INA228_ReadReg24(INA228_REG_POWER, &raw24) != HAL_OK) return -1.0f;
    
    // POWER is a 24-bit unsigned value, uses all bits [23:0], no shifting needed.
    return ((float)raw24) * INA228_POWER_LSB;
}