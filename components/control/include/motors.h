#ifndef MOTORS_H
#define MOTORS_H

#include "bdc_motor.h"
#include "driver/gpio.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ   10000000    // 10Mhz
#define BDC_MCPWM_FREQ_HZ               20000       // 20Mhz              
#define BDC_MCPWM_DUTY_TICK_MAX         (((BDC_MCPWM_TIMER_RESOLUTION_HZ) / (BDC_MCPWM_FREQ_HZ)) - 1)
#define BDC_MCPWM_DUTY_CYCLE            100
#define BDC_DUTY_CYCLE(X)               (((X) * (BDC_MCPWM_DUTY_TICK_MAX)) / (BDC_MCPWM_DUTY_CYCLE))


/**
 * @brief Structure that contain motor information
 * 
 * This structure have information related to
 * 
 * Enable pins 
 * PWM pins
 * BDC group pwm id
 * motor handler
 */
typedef struct
{
    gpio_num_t ena_A;
    gpio_num_t ena_B;
    gpio_num_t pwm_A;
    gpio_num_t pwm_B;
    int group_id;
    bdc_motor_handle_t motor_handler;
}motors_vars_t;


/** 
 * @brief Initialize bts7960 enable pins
 * 
 * @param[in] enable_A
 * @param[in] enable_B
 */
void initialize_enable_pins_bts7960(gpio_num_t enable_A, gpio_num_t enable_B);

/**
 * @brief Enable bts7960 pins
 * 
 * @param[in] enable_A
 * @param[in] enable_B
 */
void enable_pins_bts7960(gpio_num_t enable_A, gpio_num_t enable_B);

/**
 * @brief Disable bts7960 pins
 * 
 * @param[in] enable_A
 * @param[in] enable_B
 */
void disable_pins_bts7960(gpio_num_t enable_A, gpio_num_t enable_B);

/**
 * @brief Init motor
 *
 * Init motor with bdc_motor.
 * 
 * @param[in] motor_to_init  
 */
void init_motor(motors_vars_t *motor_to_init);

/**
 * @brief Init relay sensors
 * 
 * Set pins to input state and put pullup resistors.
 * 
 * @param[in] gpio_dw
 * @param[in] gpio_up
 */
void initialize_relay_switch_sensors_ml(gpio_num_t gpio_dw, gpio_num_t gpio_up);

#endif