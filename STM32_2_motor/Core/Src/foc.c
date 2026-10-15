#include "foc.h"
#include <math.h>

#define SQRT3_OVER_2   0.8660254038f
#define ONE_OVER_SQRT3 0.5773502692f

void FOC_Init(void)
{
    /* 无全局状态 */
}

/* Clarke：三相电流 → αβ（幅值不变），当前预留
   标准式 α = (2ia - ib - ic)/3，β = (ib - ic)/√3。
   利用三相平衡 ia+ib+ic=0 化简后即下式：α = ia，β = (ia + 2ib)/√3。
   注意该化简只在三相平衡且零序分量为 0 时成立。 */
void FOC_Clarke(float ia, float ib, float ic, float *alpha, float *beta)
{
    (void)ic;
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * ONE_OVER_SQRT3;
}

/* Park：αβ → dq，当前预留 */
void FOC_Park(float alpha, float beta, float angle, float *d, float *q)
{
    float c = cosf(angle);
    float s = sinf(angle);
    *d =  alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

/* 逆 Park：dq → αβ */
void FOC_InvPark(float d, float q, float angle, float *alpha, float *beta)
{
    float c = cosf(angle);
    float s = sinf(angle);
    *alpha = d * c - q * s;
    *beta  = d * s + q * c;
}

/* SVPWM：αβ（标幺，1.0 = 母线电压）→ 三相占空比（0 ~ 1） */
void FOC_SVPWM(float alpha, float beta, float *duty_u, float *duty_v, float *duty_w)
{
    float va = alpha;
    float vb = -0.5f * alpha + SQRT3_OVER_2 * beta;
    float vc = -0.5f * alpha - SQRT3_OVER_2 * beta;
    float vmin = va, vmax = va, offset;

    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;

    /* 零序注入，居中调制 */
    offset = 0.5f * (vmin + vmax);
    va -= offset;
    vb -= offset;
    vc -= offset;

    *duty_u = 0.5f + va;
    *duty_v = 0.5f + vb;
    *duty_w = 0.5f + vc;

    if (*duty_u > 1.0f) *duty_u = 1.0f; else if (*duty_u < 0.0f) *duty_u = 0.0f;
    if (*duty_v > 1.0f) *duty_v = 1.0f; else if (*duty_v < 0.0f) *duty_v = 0.0f;
    if (*duty_w > 1.0f) *duty_w = 1.0f; else if (*duty_w < 0.0f) *duty_w = 0.0f;
}

/* 电流环 PI：预留，当前不实现、不调用 */
void FOC_CurrentControl(float target_id, float target_iq,
                        float actual_id, float actual_iq,
                        float dt, float *vd, float *vq)
{
    (void)target_id; (void)target_iq;
    (void)actual_id; (void)actual_iq;
    (void)dt;
    *vd = 0.0f;
    *vq = 0.0f;
}
