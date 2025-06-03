#ifndef SENSOR_TASKS_H
#define SENSOR_TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"

#include "driver/gpio.h"

// #define SENSORS_ON
#define STACK_SIZE_SENSOR       1024
#define TASK_SENSOR_PRIORITY    1
#define TASK_SENSOR_CORE        0
#define PIN_TRIGGER             GPIO_NUM_12
#define PIN_ECHO                GPIO_NUM_13
#define PIN_AM2308              GPIO_NUM_14


/** 
 * @brief Enumeration for sensors
 */
typedef enum
{
    distance_sensor = 0,
    humedity_sensor, 
    temperature_sensor,
    rpm_sensor,
}sensors_e;

/**
 * @brief Get the queue handler related to distance. 
 * 
 * This function retrieves the queue handler and returns it by reference.
 * 
 * @param[in] handle Identifier or context used to locate the queue.
 */
esp_err_t get_distance_queue(QueueHandle_t *queue_handle);

/**
 * @brief Get the queue handler related to humbedity. 
 * 
 * This function retrieves the queue handler and returns it by reference.
 * 
 * @param[in] handle Identifier or context used to locate the queue.
 */
esp_err_t get_humedity_queue(QueueHandle_t *queue_handle);

/**
 * @brief Get the queue handler related to temperature. 
 * 
 * This function retrieves the queue handler and returns it by reference.
 * 
 * @param[in] handle Identifier or context used to locate the queue.
 */
esp_err_t get_temperature_queue(QueueHandle_t *queue_handle);

/**
 * @brief Init sensors
 */
esp_err_t init_sensors();


#endif
