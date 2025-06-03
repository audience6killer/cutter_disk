#include "motors.h"
#include <stdio.h>

void initialize_enable_pins_bts7960(gpio_num_t enable_A, gpio_num_t enable_B)
{
    // enable A pin
    ESP_ERROR_CHECK(gpio_set_direction(enable_A, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_pulldown_en(enable_A));

    // enable B pin
    ESP_ERROR_CHECK(gpio_set_direction(enable_B, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_pulldown_en(enable_B));

    // disable pins
    ESP_ERROR_CHECK(gpio_set_level(enable_A, 0));
    ESP_ERROR_CHECK(gpio_set_level(enable_B, 0));
}

void enable_pins_bts7960(gpio_num_t enable_A, gpio_num_t enable_B)
{
    ESP_ERROR_CHECK(gpio_set_level(enable_A, 1));
    ESP_ERROR_CHECK(gpio_set_level(enable_B, 1));
}

void disable_pins_bts7960(gpio_num_t enable_A, gpio_num_t enable_B)
{
    ESP_ERROR_CHECK(gpio_set_level(enable_A, 0));
    ESP_ERROR_CHECK(gpio_set_level(enable_B, 0));
}

void init_motor(motors_vars_t *motor_to_init)
{
   initialize_enable_pins_bts7960(motor_to_init->ena_A, motor_to_init->ena_B);
   disable_pins_bts7960(motor_to_init->ena_A, motor_to_init->ena_B);

   // bdc init
   bdc_motor_config_t motor_config = {
    .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    .pwma_gpio_num = motor_to_init->pwm_A,
    .pwmb_gpio_num = motor_to_init->pwm_B
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = motor_to_init->group_id,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor_to_init->motor_handler));

}

void initialize_relay_switch_sensors_ml(gpio_num_t gpio_dw, gpio_num_t gpio_up)
{
    // input pin
    ESP_ERROR_CHECK(gpio_set_direction(gpio_dw, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_pulldown_en(gpio_dw));

    // input pin
    ESP_ERROR_CHECK(gpio_set_direction(gpio_up, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_pulldown_en(gpio_up));
}