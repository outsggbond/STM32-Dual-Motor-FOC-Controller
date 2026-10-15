#ifndef __CURRENT_SENSE_H
#define __CURRENT_SENSE_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ADC 电流采样扩展模块：当前硬件无采样，仅保留接口，不初始化、不读取、不参与控制 */

typedef struct
{
    uint16_t adc_raw_u;
    uint16_t adc_raw_v;
    uint16_t adc_raw_w;

    float current_u;
    float current_v;
    float current_w;

    float ia;
    float ib;
    float ic;

    float current_offset_u;
    float current_offset_v;
    float current_offset_w;

    uint8_t current_valid;
} CurrentSense_t;

HAL_StatusTypeDef CurrentSense_Init(void);
HAL_StatusTypeDef CurrentSense_Read(CurrentSense_t *sense);
void CurrentSense_CalibrateOffset(CurrentSense_t *sense);

#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_SENSE_H */
