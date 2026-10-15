#include "motor.h"
#include "motor_config.h"   /* 控制周期等配置 */
#include <string.h>

Motor_t motor1;
Motor_t motor2;

/* 电机对象初始化：保存硬件资源、设置默认参数、状态 = INIT、PWM/EN 关闭 */
void Motor_Init(Motor_t *motor,
                TIM_HandleTypeDef *timer,
                uint32_t ch_u, uint32_t ch_v, uint32_t ch_w,
                GPIO_TypeDef *en_port, uint16_t en_pin,
                I2C_HandleTypeDef *i2c, uint8_t encoder_addr)
{
    memset(motor, 0, sizeof(Motor_t));

    motor->pwm_timer = timer;
    motor->pwm_u_channel = ch_u;
    motor->pwm_v_channel = ch_v;
    motor->pwm_w_channel = ch_w;
    motor->en_port = en_port;
    motor->en_pin = en_pin;
    motor->i2c = i2c;
    motor->encoder_addr = encoder_addr;

    /* 默认参数（实际值由 Motor_ApplyConfig 覆盖） */
    motor->pole_pairs = 7.0f;
    motor->direction = 1.0f;
    motor->voltage_limit = 12.0f;
    motor->bus_voltage = 12.0f;

    /* 编码器计数方向由 Motor_ApplyConfig 按电机覆盖；这里给一个安全默认值 */
    motor->encoder_sign = 1.0f;

    PID_Init(&motor->position_pid, 0.0f, 0.0f, 0.0f, -20.0f, 20.0f);
    PID_Init(&motor->velocity_pid, 0.0f, 0.0f, 0.0f, -12.0f, 12.0f);

    motor->control_mode = MOTOR_CONTROL_POSITION;
    motor->state = MOTOR_STATE_INIT;

    /* EN 极性默认高有效。★ 必须在这里就置好：下面这行 WritePin 和
       Motor_ApplyConfig 都会读它，而 Motor_Init 跑在 ApplyConfig 之前。
       原来这行写死 GPIO_PIN_RESET，"低有效"的板子恰好蒙对、高有效的板子
       一上电就把驱动使能了（电机在初始化阶段就通电）。 */
    motor->en_active_high = 1;
    Motor_WriteEN(motor, 0);
    Motor_SetPWM(motor, 0.0f, 0.0f, 0.0f);
}

/* EN 引脚的唯一写入口。enable=1 表示「使能驱动」。
   不同驱动板的有效极性不同（见 motor_config.h 的 EN 极性说明）：
   高有效时使能=拉高、关断=拉低，低有效时反过来。
   极性搞反的后果极隐蔽：CCER/MOE/占空比/PWM 波形全都正常，但驱动板实际
   处于关断态，电机零转矩 —— 所以每一处 EN 操作都必须走这个函数，
   绝不能各自写死 GPIO_PIN_SET/RESET。 */
void Motor_WriteEN(Motor_t *motor, uint8_t enable)
{
    GPIO_PinState on  = motor->en_active_high ? GPIO_PIN_SET : GPIO_PIN_RESET;
    GPIO_PinState off = motor->en_active_high ? GPIO_PIN_RESET : GPIO_PIN_SET;

    HAL_GPIO_WritePin(motor->en_port, motor->en_pin, enable ? on : off);
}

/* 启动三相 PWM */
void Motor_PWM_Start(Motor_t *motor)
{
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_u_channel);
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_v_channel);
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_w_channel);
}

/* 设置三相占空比（0.0 ~ 1.0），CCR = duty × ARR */
void Motor_SetPWM(Motor_t *motor, float duty_u, float duty_v, float duty_w)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);

    if (duty_u > 1.0f) duty_u = 1.0f; else if (duty_u < 0.0f) duty_u = 0.0f;
    if (duty_v > 1.0f) duty_v = 1.0f; else if (duty_v < 0.0f) duty_v = 0.0f;
    if (duty_w > 1.0f) duty_w = 1.0f; else if (duty_w < 0.0f) duty_w = 0.0f;

    motor->duty_u = duty_u;
    motor->duty_v = duty_v;
    motor->duty_w = duty_w;

    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_u_channel, (uint32_t)(duty_u * (float)arr));
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_v_channel, (uint32_t)(duty_v * (float)arr));
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_w_channel, (uint32_t)(duty_w * (float)arr));
}

/* 使能：EN = 使能电平，进入 RUNNING（仅 READY 可启动） */
void Motor_Enable(Motor_t *motor)
{
    if (motor->state != MOTOR_STATE_READY)
        return;

    Motor_WriteEN(motor, 1);
    motor->driver_enabled = 1;
    motor->state = MOTOR_STATE_RUNNING;

    /* 复位测速与堵转检测窗口。不复位的话，堵转计时会沿用上一次运行留下的
       起始时刻，一启动就可能立刻判堵转（误报），或反之延迟报警。 */
    motor->velocity_window_stamp = motor->encoder_stamp;
    motor->velocity_window_angle = motor->mechanical_angle;
    motor->velocity              = 0.0f;
    motor->stall_tick            = HAL_GetTick();
    motor->stall_angle           = motor->mechanical_angle;
    motor->stall_last_angle      = motor->mechanical_angle;
    motor->stall_travel          = 0.0f;

    /* 每次启动清零热累积，但若上次是热跳闸则拒绝启动 ——
       必须让电机真的冷下来，不能靠"再启动一次"把它清掉。 */
    if (motor->thermal_tripped)
    {
        motor->state = MOTOR_STATE_ERROR;
        motor->error = MOTOR_ERROR_THERMAL;
        Motor_WriteEN(motor, 0);
        motor->driver_enabled = 0;
        return;
    }
    motor->thermal_i2t = 0.0f;
}

/* 关闭：PWM = 0，EN = 关断电平，进入 READY。
   ★ 必须清速度环积分。原来的 STOP 只断输出、不动积分器，于是积分把停机前
     的误差一直记着；下次 SPD/POS 一使能，那份陈旧积分会在第一个周期就全额
     加到输出上 —— 实测 STOP 后再启动有一次约 165 度的甩尾。放在这里而不是
     放在 STOP 处理器里，是因为 STOP / STOPALL / 校准收尾 / 出错停机都走
     这个函数，一处就全覆盖了。 */
void Motor_Disable(Motor_t *motor)
{
    Motor_SetPWM(motor, 0.0f, 0.0f, 0.0f);
    Motor_WriteEN(motor, 0);
    motor->driver_enabled = 0;
    motor->state = MOTOR_STATE_READY;
    motor->target_voltage = 0.0f;
    motor->velocity_pid.integral = 0.0f;
    motor->vd = 0.0f;
    motor->vq = 0.0f;
}

/* 错误统一入口：记录错误 → ERROR → PWM 0 → EN 关断。
   ★ EN 必须按极性写：原来这里写死 GPIO_PIN_RESET，在"低有效"的板子上
     等于把驱动【使能】了 —— 报警停机反而通电，是最不该出错的地方。 */
void Motor_SetError(Motor_t *motor, uint32_t error)
{
    motor->error |= error;
    motor->state = MOTOR_STATE_ERROR;
    Motor_SetPWM(motor, 0.0f, 0.0f, 0.0f);
    Motor_WriteEN(motor, 0);
    motor->driver_enabled = 0;
    /* 与 Motor_Disable 同理：停机就要把积分器归零，否则自动恢复（编码器那条
       路径会自己回到 READY）之后，第一个周期会带着故障前的积分重新起转。 */
    motor->velocity_pid.integral = 0.0f;
}
