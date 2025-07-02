#ifndef SEED_DISPENSER_TASK_COMMON_H
#define SEED_DISPENSER_TASK_COMMON_H

#define SEED_DISPENSER_CORE_ID          1
#define SEED_DISPENSER_STACK_SIZE       1024 * 4
#define SEED_DISPENSER_TASK_PRIORITY    10

#define BDC_MCPWM_TIMER_RESOLUTION_HZ   10000000    // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ               20000       // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_ENCODER_PCNT_HIGH_LIMIT     1000
#define BDC_ENCODER_PCNT_LOW_LIMIT      -1000
#define BDC_PID_LOOP_PERIOD_MS          100         // calculate the motor speed every 100ms

#define SEED_DISPENSER_PWM_A            41 
#define SEED_DISPENSER_PWM_B            42

#define SEED_DISPENSER_ENCODER_A        40 
#define SEED_DISPENSER_ENCODER_B        39

// 11.25 pulses @100ms -> 7.5RPM
// 90 pulses @100ms -> 60RPM
#define SEED_DISPENSER_DESIRED_SPEED    2  // Pulses
#endif