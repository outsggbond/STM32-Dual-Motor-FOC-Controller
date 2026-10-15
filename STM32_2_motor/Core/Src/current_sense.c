#include "current_sense.h"

/* 预留实现：当前版本 ADC 不配置、不采样，函数不被调用 */

HAL_StatusTypeDef CurrentSense_Init(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef CurrentSense_Read(CurrentSense_t *sense)
{
    (void)sense;
    return HAL_OK;
}

void CurrentSense_CalibrateOffset(CurrentSense_t *sense)
{
    (void)sense;
}
