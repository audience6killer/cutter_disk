#ifndef ROBOT_TASKS_H
#define ROBOT_TASKS_H

#include "esp_err.h"

#define TASK_EVENT_CORE             0
#define TASK_EVENT_PRIORITY         3
#define TASK_EVENT_STACK_SIZE       1024
#define ESP32_DATA_LENGTH           200

// #define TEST_RPMS


/**
 * @brief init tasks
 * 
 * Init tasks
 * 
 * PID control
 * sensors
 * robot_tasks
 */
esp_err_t init_tasks();


#endif