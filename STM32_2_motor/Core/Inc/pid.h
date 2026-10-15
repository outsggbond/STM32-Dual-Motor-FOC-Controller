#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_error;

    float output_min;
    float output_max;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max);
float PID_Update(PID_t *pid, float target, float feedback, float dt);
void  PID_PresetOutput(PID_t *pid, float out, float error);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
