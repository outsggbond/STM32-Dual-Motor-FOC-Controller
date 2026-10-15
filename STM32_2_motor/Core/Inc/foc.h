#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

void FOC_Init(void);

/* 当前版本使用 */
void FOC_InvPark(float d, float q, float angle, float *alpha, float *beta);
void FOC_SVPWM(float alpha, float beta, float *duty_u, float *duty_v, float *duty_w);

/* ADC 电流采样扩展预留（当前不调用） */
void FOC_Clarke(float ia, float ib, float ic, float *alpha, float *beta);
void FOC_Park(float alpha, float beta, float angle, float *d, float *q);
void FOC_CurrentControl(float target_id, float target_iq,
                        float actual_id, float actual_iq,
                        float dt, float *vd, float *vq);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
