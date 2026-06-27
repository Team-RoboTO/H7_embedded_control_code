#include "INA228.h"

float err = 0;

// I2C handle - must be defined elsewhere (main.c / CubeMX)
extern I2C_HandleTypeDef hi2c2;

/* ---------- Low-level register helpers ---------- */

static HAL_StatusTypeDef INA228_ReadReg16(uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, INA228_I2C_ADDR, reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, 2, 2);
    if (st != HAL_OK) return st;
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return HAL_OK;
}

static HAL_StatusTypeDef INA228_WriteReg16(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
    return HAL_I2C_Mem_Write(&hi2c2, INA228_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2, 2);
}

static HAL_StatusTypeDef INA228_ReadReg24(uint8_t reg, uint32_t *out)
{
    uint8_t buf[3];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c2, INA228_I2C_ADDR, reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, 3, 2);
    if (st != HAL_OK) return st;
    *out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return HAL_OK;
}

/* ---------- Public API ---------- */

uint8_t INA228_IsConnected(void)
{
    return (HAL_I2C_IsDeviceReady(&hi2c2, INA228_I2C_ADDR, 3, 100) == HAL_OK) ? 1 : 0;
}

uint8_t INA228_Init(void)
{
    if (!INA228_IsConnected()) return 1;

    // 1. CONFIG register: reset then default
    //    Bit 15 = 1 triggers a software reset; wait briefly then write defaults
    if (INA228_WriteReg16(INA228_REG_CONFIG, 0x8000) != HAL_OK) return 2;
    HAL_Delay(2);  // allow reset to complete

    if (INA228_WriteReg16(INA228_REG_CONFIG, 0x0000) != HAL_OK) return 2;

    // 2. ADC_CONFIG register
    uint16_t mode    = 0xB;  // Continuous: shunt + bus + temp
    uint16_t vbus_ct = 0x3;  // VBUS conversion time  = 1052 us
    uint16_t vsh_ct  = 0x3;  // Shunt conversion time = 1052 us
    uint16_t vt_ct   = 0x3;  // Temp conversion time  = 1052 us
    uint16_t avg     = 0x3;  // 64-sample averaging

    uint16_t adcCfg = (mode << 12) | (vbus_ct << 9) | (vsh_ct << 6) | (vt_ct << 3) | avg;
    if (INA228_WriteReg16(INA228_REG_ADC_CONFIG, adcCfg) != HAL_OK) return 3;

    // 3. Calibration register
    if (INA228_WriteReg16(INA228_REG_SHUNT_CAL, INA228_CAL_VALUE) != HAL_OK) return 4;

    return 0;  // success
}

float INA228_ReadBusVoltage(void)
{
    uint32_t raw24 = 0;
    if (INA228_ReadReg24(INA228_REG_VBUS, &raw24) != HAL_OK) return -1.0f;

    // 20-bit unsigned, bits [23:4]
    uint32_t val20 = raw24 >> 4;
    return ((float)val20) * INA228_VBUS_LSB;
}

float INA228_ReadShuntVoltage(void)
{
    uint32_t raw24 = 0;
    if (INA228_ReadReg24(INA228_REG_VSHUNT, &raw24) != HAL_OK) return -1.0f;

    // 20-bit signed two's complement, bits [23:4]
    int32_t val20 = (int32_t)(raw24 >> 4);
    if (val20 & 0x80000)
        val20 |= (int32_t)0xFFF00000;

    return ((float)val20) * INA228_VSHUNT_LSB;
}

float INA228_ReadCurrent(void)
{
    uint32_t raw24 = 0;
    if (INA228_ReadReg24(INA228_REG_CURRENT, &raw24) != HAL_OK) return -1.0f;

    // 20-bit signed two's complement, bits [23:4]
    int32_t val20 = (int32_t)(raw24 >> 4);
    if (val20 & 0x80000)
        val20 |= (int32_t)0xFFF00000;

    return ((float)val20) * INA228_CURRENT_LSB;
}

float INA228_ReadPower(void)
{
    uint32_t raw24 = 0;
    HAL_StatusTypeDef result = INA228_ReadReg24(INA228_REG_POWER, &raw24);
    
    if (result != HAL_OK) {
        // What error is it exactly?
        err = HAL_I2C_GetError(&hi2c2);
        // Put a breakpoint here and check `err`:
        // HAL_I2C_ERROR_TIMEOUT  = 0x20  ? still a timeout issue
        // HAL_I2C_ERROR_AF       = 0x04  ? bus locked, slave not ACKing  
        // HAL_I2C_ERROR_BERR     = 0x01  ? electrical noise/corruption
        return -1.0f;
    }
    return ((float)raw24) * INA228_POWER_LSB;
}

float INA228_ReadDieTemp(void)
{
    uint16_t raw16 = 0;
    if (INA228_ReadReg16(INA228_REG_DIETEMP, &raw16) != HAL_OK) return -999.0f;

    // DIETEMP: 16-bit signed, bits [15:4] hold the temperature, LSB = 7.8125 mC
    int16_t val12 = (int16_t)(raw16) >> 4;
    return ((float)val12) * 0.0078125f;
}

uint16_t INA228_ReadID(void)
{
    uint16_t id = 0;
    INA228_ReadReg16(INA228_REG_ID, &id);
    return id;  // expect 0x2280 for INA228
}