#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_REG_RAW_ANGLE   0x0C    /* 原始角度寄存器（12 位） */

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t *raw);
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint8_t addr, float *angle);

#ifdef __cplusplus
}
#endif

#endif /* __AS5600_H */
