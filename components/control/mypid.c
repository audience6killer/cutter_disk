#include "mypid.h"

void pid_u(pid_data_t *pid, pid_outputs_t *pid_outputs)
{
    // pid parts 
    float e0 = 0;
    float e1 = 0;
    float e2 = 0;

    // operations 
    e0 = (pid->kp + ((pid->ki * pid->ts)/2) + (pid->kd/pid->ts)) * pid->e[0];
    e1 = (-pid->kp + ((pid->ki * pid->ts)/2) - ((2 * pid->kd)/pid->ts)) * pid->e[1];
    e2 = (pid->kd/pid->ts) * pid->e[2];
    
    // discrete pid 
    pid->u[0] = pid->u[1] + e0 + e1 + e2;

    if(pid->u[0] >= pid_outputs->u_max_output)
    {
        pid->u[0] = pid_outputs->u_max_output;
    }else if(pid->u[0] <= pid_outputs->u_min_output)
    {
        pid->u[0] = pid_outputs->u_min_output;
    }

    pid_outputs->u = (uint32_t)pid->u[0];

}

void initialize_e_u(pid_data_t *pid)
{
    pid->u[1] = pid->u[0];
    pid->e[1] = pid->e[0];
    pid->e[2] = pid->e[1];
}

uint32_t convert_u_to_pwm(pid_outputs_t *pid_outputs)
{
    uint32_t output = 0;
    output = (pid_outputs->u - pid_outputs->u_min_output) * (pid_outputs->pwm_max_value - pid_outputs->pwm_min_value)/(pid_outputs->u_max_output - pid_outputs->u_min_output) + pid_outputs->pwm_min_value;
    return output;
}
