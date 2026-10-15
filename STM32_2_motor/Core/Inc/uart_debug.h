#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "stm32f1xx_hal.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART_Debug_Init(void);
void UART_Debug_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_DEBUG_H */
