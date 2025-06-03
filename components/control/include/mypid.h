#ifndef MY_PID_H
#define MY_PID_H

#include <stdio.h>
#include <stdbool.h>

typedef struct
{
    float setpoint;
    float u[2];
    float e[3];
    float kp;
    float ki;
    float kd;
    float ts; 
} pid_data_t;

typedef struct
{
    uint32_t u_max_output;
    uint32_t u_min_output;
    uint32_t pwm_max_value;
    uint32_t pwm_min_value;
    uint8_t direction;
    uint32_t u;
} pid_outputs_t;


void pid_u(pid_data_t *pid, pid_outputs_t *pid_outputs);
void initialize_e_u(pid_data_t *pid);
uint32_t convert_u_to_pwm(pid_outputs_t *pid_outputs);



#endif