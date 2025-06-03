#ifndef VELOCITY_TASK_H
#define VELOCITY_TASK_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define AS5600_PIN_SCL              GPIO_NUM_15
#define AS5600_PIN_SDA              GPIO_NUM_16
#define STACK_SIZE_VELOCITY         1024
#define TASK_VELOCITY_PRIORITY      3
#define TASK_VELOCITY_CORE          0

#define DISABLE_SENSOR
#define RPM_SENSOR_TEST

/**
 * @brief Start task for sense velocity
 * 
 * This function initializes the velocity sensor task
 * 
 * @return ESP_OK
 */
esp_err_t init_velocity_sense_task();

/**
 * @brief Get the queue handler that contains velocity information.
 * 
 * This function retrieves the queue handler and returns it by reference.
 * 
 * @param[in] handle Identifier or context used to locate the queue.
 */
esp_err_t get_velocity_task_queue_handle(QueueHandle_t *queue);


#endif