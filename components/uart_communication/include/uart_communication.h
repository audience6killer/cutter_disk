#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// Task configuration
#define ESP32_UART_TASK_CORE_ID                 1
#define ESP32_UART_TASK_RECEIVE_PRIORITY        4
#define ESP32_UART_TASK_TRANSFER_PRIORITY       2
#define ESP32_UART_TASK_STACK_SIZE              1024 * 2

// UART configuration
#define ESP32_UART_BUFFER_SIZE                  256
#define ESP32_UART_QUEUE_SIZE                   5
#define ESP32_UART_PORT                         UART_NUM_2// UART_NUM_0
#define ESP32_UART_TX_PIN                       GPIO_NUM_16// GPIO_NUM_43
#define ESP32_UART_RX_PIN                       GPIO_NUM_17//  GPIO_NUM_44
#define ESP32_UART_RTS_PIN                      UART_PIN_NO_CHANGE
#define ESP32_UART_CTS_PIN                      UART_PIN_NO_CHANGE
#define ESP32_UART_BAUDRATE_RF                  115200
#define ESP32_UART_DATA_BITS                    UART_DATA_8_BITS
#define ESP32_DATA_LENGTH                       200     // In bytes

#define WAIT_QUEUE_SEND_ESP32                   100


/**
 * @brief Enumeration for ESP32 commands.
 * 
 * This enum defines the different commands that can be managed by the ESP32
 * to control various components such as the cutter, linear motors and dispenser.
 * 
 */
typedef enum 
{
    CMD_EMPTY = 0,           /**< No command. */ 
    CMD_STOP_CUTTER,         /**< Stop the cutter. */
    CMD_START_CUTTER,        /**< Start the cutter. */
    CMD_DISTANCE,            /**< Send the distance measure. */
    CMD_RPMS,                /**< Send RPMs measure. */
    CMD_LINEAR_MOTOR_UP,     /**< Put the linear motors in up state. */
    CMD_LINEAR_MOTOR_DW,     /**< Put the linear motors in down state. */
    CMD_START_DISPENSER,     /**< Start the dispenser. */
    CMD_STOP_DISPENSER,      /**< Stop the dispenser. */
    CMD_ERROR,              /**< Command no identified. */
}state_cmd_esp32_e;


/**
 * @brief Structure for received commands
 * 
 * This structure have the commands and value if it's necessary for ESP32.
 */
typedef struct
{
    state_cmd_esp32_e code; /**< Command.*/
    float arg;              /**< Arguments. */
} data_center_esp32_t;


/** 
 * @brief enumeration for ESP32 states.
 */
typedef enum 
{
    EMPTY_CMD = 0,      /**< Empty cmd */
    ERROR_CMD,          /**< Error CMD. */
    CUTTER_STARTED,     /**< Cutter started. */
    CUTTER_STOPPED,     /**< Cutter stopped. */
    CUTTER_UP,          /**< Linear motors set in up position. */
    CUTTER_DOWN,        /**< Linear motors set in down position. */
    SEED_DISTANCE,      /**< Distance measure. */
    DISPENSER_STARTED,  /**< Dispenser started. */
    DISPENSER_STOPPED,  /**< Dispenser stopped. */
    TEMPERATURE_MEASURE, /**< Temperature measure */
    HUMEDITY_MEASURE,   /**< Humedity measure */
    CUTTER_RPM_MEASURE, /**< Cutter rpm measure */
}cutter_disk_events_cmd_e;

/**
 * @brief enumeration for ESP32 errors.
 */
typedef enum
{
    NONE = 0,               /**< No error.*/
    CUTTER_ERROR,           /**< Error in cutter.*/
    CUTTER_MOV_ERROR,       /**< Error in linear motors.*/
    DISPENCER_ERROR,        /**< Error in dispenser.*/
    SEED_DISTANCE_ERROR,    /**< Error in distance measure.*/
    HUMEDITY_ERROR,         /**< Error in humedity measure. */
    TEMPERATURE_ERROR,      /**< Error in temperature measure. */

}cutter_disk_error_e;


/**
 * @brief Structure that contains information related to events and errors.
 */
typedef struct 
{
    cutter_disk_events_cmd_e event;     /**< ESP32 event. */ 
    cutter_disk_error_e error;          /**< Error code. */
    float arg;                          /**< Arg info. */
}cutter_disk_event_t; 


/**
 * @brief Get the queue handler to send information. 
 * 
 * This function retrieves the queue handler and returns it by reference.
 * 
 * @param[in] handle Identifier or context used to locate the queue.
 *  
 */
esp_err_t esp32_uart_get_queue_data2send(QueueHandle_t *handle);


/**
 * @brief Get the queue handler related to received information. 
 * 
 * This function retrieves the queue handler and returns it by reference.
 * 
 * @param[in] handle Identifier or context used to locate the queue.
 *  
 */
esp_err_t esp32_uart_get_queue_data_received(QueueHandle_t *handle);


/**
 * @brief Starts the RF communication task. 
 *  
 */
void esp32_uart_task_start(void);


#endif