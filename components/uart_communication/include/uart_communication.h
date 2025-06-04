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
    SOWER_CMD_EMPTY = 0,           /**< No command. */ 
    SOWER_CMD_STOP_CUTTER,         /**< Stop the cutter. */
    SOWER_CMD_START_CUTTER,        /**< Start the cutter. */
    SOWER_CMD_DISTANCE,            /**< Send the distance measure. */
    SOWER_CMD_RPMS,                /**< Send RPMs measure. */
    SOWER_CMD_LINEAR_MOTOR_UP,     /**< Put the linear motors in up state. */
    SOWER_CMD_LINEAR_MOTOR_DOWN,     /**< Put the linear motors in down state. */
    SOWER_CMD_START_DISPENSER,     /**< Start the dispenser. */
    SOWER_CMD_STOP_DISPENSER,      /**< Stop the dispenser. */
    SOWER_CMD_ECHO_SOWER,
    SOWER_CMD_ERROR,              /**< Command no identified. */
} sower_cmd_e;


/**
 * @brief Structure for received commands
 * 
 * This structure have the commands and value if it's necessary for ESP32.
 */
typedef struct
{
    sower_cmd_e code; /**< Command.*/
    float arg;              /**< Arguments. */
} sower_cmd_t;


/** 
 * @brief enumeration for ESP32 states.
 */
typedef enum 
{
    SOWER_EVENT_EMPTY_CMD = 0,      /**< Empty cmd */
    SOWER_EVENT_ERROR,          /**< Error CMD. */
    SOWER_EVENT_CUTTER_STARTED,     /**< Cutter started. */
    SOWER_EVENT_CUTTER_STOPPED,     /**< Cutter stopped. */
    SOWER_EVENT_CUTTER_UP,          /**< Linear motors set in up position. */
    SOWER_EVENT_CUTTER_DOWN,        /**< Linear motors set in down position. */
    SOWER_EVENT_SEED_DISTANCE,      /**< Distance measure. */
    SOWER_EVENT_DISPENSER_STARTED,  /**< Dispenser started. */
    SOWER_EVENT_DISPENSER_STOPPED,  /**< Dispenser stopped. */
    SOWER_EVENT_TEMPERATURE_MEASURE, /**< Temperature measure */
    SOWER_EVENT_HUMEDITY_MEASURE,   /**< Humedity measure */
    SOWER_EVENT_CUTTER_RPM_MEASURE, /**< Cutter rpm measure */
    SOWER_EVENT_ECHO_MSG,
} sower_events_e;

/**
 * @brief enumeration for ESP32 errors.
 */
typedef enum
{
    SOWER_ERROR_NONE = 0,               /**< No error.*/
    SOWER_ERROR_CUTTER_DONT_START,
    SOWER_ERROR_CUTTER_DONT_STOP,
    SOWER_ERROR_LMOTOR_DONT_RISE,
    SOWER_ERROR_LMOTOR_DONT_DESCEND,
    SOWER_ERROR_UNKNOWN_CMD,
    SOWER_ERROR_DISPENCER_ERROR,        /**< Error in dispenser.*/
    SOWER_ERROR_SEED_DISTANCE_ERROR,    /**< Error in distance measure.*/
    SOWER_ERROR_HUMEDITY_ERROR,         /**< Error in humedity measure. */
    SOWER_ERROR_TEMPERATURE_ERROR,      /**< Error in temperature measure. */

} sower_error_e;

/**
 * @brief Get the string name of a sower_cmd_e value.
 *
 * @param cmd The sower_cmd_e value.
 * @return The string name of the command.
 */
static inline const char* sower_cmd_name(sower_cmd_e cmd)
{
    switch (cmd) {
        case SOWER_CMD_EMPTY:            return "SOWER_CMD_EMPTY";
        case SOWER_CMD_STOP_CUTTER:      return "SOWER_CMD_STOP_CUTTER";
        case SOWER_CMD_START_CUTTER:     return "SOWER_CMD_START_CUTTER";
        case SOWER_CMD_DISTANCE:         return "SOWER_CMD_DISTANCE";
        case SOWER_CMD_RPMS:             return "SOWER_CMD_RPMS";
        case SOWER_CMD_LINEAR_MOTOR_UP:  return "SOWER_CMD_LINEAR_MOTOR_UP";
        case SOWER_CMD_LINEAR_MOTOR_DOWN:return "SOWER_CMD_LINEAR_MOTOR_DOWN";
        case SOWER_CMD_START_DISPENSER:  return "SOWER_CMD_START_DISPENSER";
        case SOWER_CMD_STOP_DISPENSER:   return "SOWER_CMD_STOP_DISPENSER";
        case SOWER_CMD_ECHO_SOWER:       return "SOWER_CMD_ECHO_SOWER";
        case SOWER_CMD_ERROR:            return "SOWER_CMD_ERROR";
        default:                         return "UNKNOWN_SOWER_CMD";
    }
}

/**
 * @brief Structure that contains information related to events and errors.
 */
typedef struct 
{
    sower_events_e event;     /**< ESP32 event. */ 
    sower_error_e error;          /**< Error code. */
    float arg;                          /**< Arg info. */
} sower_event_t; 


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