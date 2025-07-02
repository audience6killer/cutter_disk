
#ifndef CONTROL_H
#define CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "as5600_sensor.h"
#include "motors.h"
// #include "mypid.h"


#define STACK_SIZE_CONTROL          1024
#define TASK_CONTROL_PRIORITY       5
#define TASK_CONTROL_CORE           0

#define PID_LOOP_PERIOD_MS          50 * 1000

#define PWM_OFF                     0
#define PWM_LINEAR_MOTOR            50
#define PWM_CUTTER_MOTOR_TEST       100

#define CUTTER_PWM_A                GPIO_NUM_4//GPIO_NUM_18//GPIO_NUM_4
#define CUTTER_PWM_B                GPIO_NUM_5//GPIO_NUM_19//GPIO_NUM_5
#define CUTTER_ENA_A                GPIO_NUM_6//GPIO_NUM_20//GPIO_NUM_6
#define CUTTER_ENA_B                GPIO_NUM_7//GPIO_NUM_21//GPIO_NUM_7
                                    
#define LINEAR_MOTOR_PWM_A          GPIO_NUM_17//GPIO_NUM_22//GPIO_NUM_17
#define LINEAR_MOTOR_PWM_B          GPIO_NUM_18//GPIO_NUM_23//GPIO_NUM_18
#define LINEAR_MOTOR_ENA_A          GPIO_NUM_8//GPIO_NUM_25//GPIO_NUM_8//GPIO_NUM_10
#define LINEAR_MOTOR_ENA_B          GPIO_NUM_9//GPIO_NUM_26//GPIO_NUM_9//GPIO_NUM_11
#define LINEAR_SENSOR_UP            GPIO_NUM_10//GPIO_NUM_27//GPIO_NUM_10//GPIO_NUM_12
#define LINEAR_SENSOR_DW            GPIO_NUM_11//GPIO_NUM_32//GPIO_NUM_11//GPIO_NUM_13

#define CONTROL_TEST
// #define ENABLE_LINEAR_MOTOR


/**
 * @brief Init cutter tasks
 * 
 * This function initialize tasks and sensors
 * 
 * Velocity task
 * PID block task
 * PID timer 
 * Motors (Cutter and linear motor)
 * 
 */
esp_err_t init_cutter_task();

/**
 * @brief Stop cutter tasks
 * 
 * This function disable the cutter tasks
 * 
 * PID block task
 * PID timer 
 * 
 */
esp_err_t stop_cutter_motor();

/**
 * @brief Start cutter tasks
 * 
 * This function enable the cutter tasks.
 * 
 * PID block task
 * PID timer
 */
esp_err_t start_cutter_motor();

/**
 * @brief Set linear motor up
 * 
 * This function set the linear motor in up position.
 */
esp_err_t linear_motor_up();

/**
 * @brief Set linear motor down
 * 
 * This function set the linear motor in down position.
 */
esp_err_t linear_motor_down();

/**
 * @brief Set linear motor stop
 * 
 * This function disable the linear motor.
 */
esp_err_t linear_motor_stop();

/**
 * @brief Read linear motor sensors 
 *
 * This function read the linear motor sensors and return a value 
 */
uint8_t is_linear_motor_up_down();

/**
 * @brief Test rpms
 * 
 * This function test the motor at PWM level. 
 */
esp_err_t test_rpms();

/**
 * @brief Stop test rpms
 * 
 * This function stop the motor (put in PWM to zero).
 */
esp_err_t stop_test_rpms();

#endif