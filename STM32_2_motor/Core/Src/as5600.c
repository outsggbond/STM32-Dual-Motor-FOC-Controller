#include "as5600.h"

#define AS5600_TIMEOUT       50U
#define AS5600_ANGLE_SCALE   6.283185307f   /* 2π / 4096 */

/* 读取原始角度（0 ~ 4095） */
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t *raw)
{
    uint8_t data[2];
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1), AS5600_REG_RAW_ANGLE,
                          I2C_MEMADD_SIZE_8BIT, data, 2, AS5600_TIMEOUT);
    if (st != HAL_OK)
    {
        return st;
    }

    *raw = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    *raw &= 0x0FFFu;
    return HAL_OK;
}

/* 读取机械角度（弧度，0 ~ 2π） */
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint8_t addr, float *angle)
{
    uint16_t raw;
    HAL_StatusTypeDef st = AS5600_ReadRawAngle(hi2c, addr, &raw);

    if (st != HAL_OK)
    {
        return st;
    }

    *angle = (float)raw / 4096.0f * AS5600_ANGLE_SCALE;
    return HAL_OK;
}
