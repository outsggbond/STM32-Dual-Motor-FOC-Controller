#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "motor.h"
#include "as5600.h"
#include "foc.h"

#ifdef __cplusplus
extern "C" {
#endif

void Motor_System_Init(void);
void Motor_System_Task(void);
void Motor_Control_Loop(void);
void Motor_Update(Motor_t *motor);
float Motor_GetElectricalAngle(Motor_t *motor);
void Motor_Calibrate(Motor_t *motor);

/* 开环电压软启动斜坡（按电机操作）。
   放在这里而不是 motor_config.h：motor_config.h 只包含 main.h，看不到 Motor_t。
   起始时刻在实现里记录，避免把内部状态暴露出去。 */
void Motor_RampStart(Motor_t *motor, float volt, float elec_vel, uint32_t ramp_ms);
void Motor_RampStop(Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
