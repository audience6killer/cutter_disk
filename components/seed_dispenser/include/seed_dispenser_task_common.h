#ifndef SEED_DISPENSER_TASK_COMMON_H
#define SEED_DISPENSER_TASK_COMMON_H

#define SEED_DISPENSER_CORE_ID          1
#define SEED_DISPENSER_STACK_SIZE       1024 * 2
#define SEED_DISPENSER_TASK_PRIORITY    10

#define BDC_MCPWM_TIMER_RESOLUTION_HZ   10000000    // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ               20000       // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_ENCODER_PCNT_HIGH_LIMIT     1000
#define BDC_ENCODER_PCNT_LOW_LIMIT      -1000
#define BDC_PID_LOOP_PERIOD_MS          100 // calculate the motor speed every 100ms

#define SEED_DISPENSER_PWM_A            7
#define SEED_DISPENSER_PWM_B            15

#define SEED_DISPENSER_ENCODER_A        36
#define SEED_DISPENSER_ENCODER_B        35


#endif