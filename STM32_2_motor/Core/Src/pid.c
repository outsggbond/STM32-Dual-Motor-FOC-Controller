#include "pid.h"

/* 通用 PID：当前用于位置环、速度环；电流环接口未来扩展 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output_min = min;
    pid->output_max = max;
}

float PID_Update(PID_t *pid, float target, float feedback, float dt)
{
    float error = target - feedback;
    float output;
    float d_term = 0.0f;

    if (dt > 0.0f)
        d_term = pid->kd * (error - pid->previous_error) / dt;

    /* 抗积分饱和：仅 ki>0 时积分，且把积分项限幅到剩余输出范围内 */
    if (pid->ki > 0.0f)
    {
        pid->integral += error * dt;

        float i_max = (pid->output_max - pid->kp * error - d_term) / pid->ki;
        float i_min = (pid->output_min - pid->kp * error - d_term) / pid->ki;
        if (i_max < i_min) { float t = i_max; i_max = i_min; i_min = t; }
        if (pid->integral > i_max) pid->integral = i_max;
        if (pid->integral < i_min) pid->integral = i_min;
    }

    output = pid->kp * error + pid->ki * pid->integral + d_term;

    pid->previous_error = error;

    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

/* 预置输出：反算积分项，使「当前误差下」的本次输出恰好等于 out。
   为什么需要：开环拖动 → 闭环速度 的交接瞬间，速度环的积分还是 0、输出还是 0，
   而此刻开环正施加着 align_max_volt —— 一交接电压立刻塌到 0，转子马上减速，
   要等误差重新长起来才补得回去。表现为交接处"掉一下速"，重则直接失速、
   被堵转保护掐断。预置之后输出电压在交接瞬间是连续的。

   注意 ki<=0 时无积分可预置，只能靠 P 项（而交接时误差≈0，P 项也≈0）——
   这种配置下交接必然塌陷，所以 ALIGN 模式要求 ki>0。
   单位与 PID_Update 一致：积分量是 ∫e dt，输出 = kp·e + ki·integral + kd·de/dt。 */
void PID_PresetOutput(PID_t *pid, float out, float error)
{
    if (pid->ki <= 0.0f)
        return;

    if (out > pid->output_max) out = pid->output_max;
    if (out < pid->output_min) out = pid->output_min;

    /* 不考虑微分项：交接发生在速度误差首次出现的时刻，
       de/dt 由 `previous_error` 决定，这里把它对齐成当前误差即可（d 项为 0）。 */
    pid->integral       = (out - pid->kp * error) / pid->ki;
    pid->previous_error = error;
}
