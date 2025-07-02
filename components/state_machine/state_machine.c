#include <stdio.h>
#include <string.h>
#include "state_machine.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "velocity_task.h"
#include "control.h"
#include "sensor_tasks.h"
#include "uart_communication.h"
#include "seed_dispenser.h"

#include "esp_log.h"

static const char TAG[] = "state_machine";
static QueueHandle_t esp32_queue_received = NULL;
static QueueHandle_t esp32_queue_send = NULL;

static QueueHandle_t distance_queue = NULL;
static QueueHandle_t humedity_queue = NULL;
static QueueHandle_t temperature_queue = NULL;
static QueueHandle_t rpm_queue = NULL;

static QueueHandle_t g_seed_dispenser_cmd_queue = NULL;
static QueueHandle_t g_seed_dispenser_data_queue = NULL;

static void sower_event_handler_loop(void *args);
static esp_err_t sensor_info_2_uart(sensors_e sensor);

esp_err_t init_tasks()
{
    // Init velocity task
    // init_velocity_sense_task();
    // get_velocity_task_queue_handle(&rpm_queue);

    // Start dispenser task
    seed_dispenser_task_start();

    // Init sensors
    //init_sensors();
    //get_humedity_queue(&humedity_queue);
    //get_temperature_queue(&temperature_queue);
    //get_distance_queue(&distance_queue);

    //// Init PID
    //init_cutter_task();

    //// Init task cmd
    //esp32_uart_task_start();
    //esp32_uart_get_queue_data_received(&esp32_queue_received);
    //esp32_uart_get_queue_data2send(&esp32_queue_send);

    /* Get queues */
    while (seed_dispenser_get_cmd_queue(&g_seed_dispenser_cmd_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get seed_dispenser_cmd_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while (seed_dispenser_get_data_queue(&g_seed_dispenser_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get seed_dispenser_data_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    seed_dispenser_cmd_e cmd = SD_CMD_START;
    if(xQueueSend(g_seed_dispenser_cmd_queue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        ESP_LOGI(TAG, "Dispenser started correctly");
    }
    else
    {
        ESP_LOGE(TAG, "Error: Cannot start dispenser");
    }
    

    ESP_LOGI(TAG, "receive_cmd task is initializing...");
    xTaskCreatePinnedToCore(
        sower_event_handler_loop,
        "cmd event task",
        TASK_EVENT_STACK_SIZE * 4,
        NULL,
        TASK_EVENT_PRIORITY,
        NULL,
        TASK_EVENT_CORE);

    return ESP_OK;
}

esp_err_t sower_send_to_uart_transmit_queue(sower_event_t event)
{
    if (xQueueSend(esp32_queue_send, &event, pdMS_TO_TICKS(10)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Failed to send to transmit queue.");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Event handlers*/
esp_err_t sower_stop_cutter_event_handler(void)
{
    sower_event_t event = {
        .arg = 0.0f,
        .error = SOWER_ERROR_NONE,
        .event = SOWER_EVENT_EMPTY_CMD};

#ifdef TEST_RPMS
    stop_test_rpms();
#else
    if (stop_cutter_motor() == ESP_OK)
    {
        ESP_LOGI(TAG, "Event: Cutter disk stopped successfully");

        event.event = SOWER_EVENT_CUTTER_STOPPED;
        event.error = SOWER_ERROR_NONE;
        sower_send_to_uart_transmit_queue(event);
    }
    else
    {
        ESP_LOGE(TAG, "Error: Failed to stop cutter disk!");

        event.event = SOWER_EVENT_ERROR;
        event.error = SOWER_ERROR_CUTTER_DONT_STOP;
        sower_send_to_uart_transmit_queue(event);

        return ESP_FAIL;
    }
#endif
    return ESP_OK;
}

esp_err_t sower_start_cutter_event_handler(void)
{
    sower_event_t event = {
        .arg = 0.0f,
        .error = SOWER_ERROR_NONE,
        .event = SOWER_EVENT_EMPTY_CMD};

#ifdef TEST_RPMS
    test_rpms();
#else
    if (start_cutter_motor() == ESP_OK)
    {
        event.event = SOWER_EVENT_CUTTER_STARTED;
        event.error = SOWER_ERROR_NONE;
        sower_send_to_uart_transmit_queue(event);
    }
    else
    {
        event.event = SOWER_EVENT_ERROR;
        event.error = SOWER_ERROR_CUTTER_DONT_START;
        sower_send_to_uart_transmit_queue(event);
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

esp_err_t sower_linear_motor_rise_event_handler(void)
{
    sower_event_t event = {
        .arg = 0.0f,
        .error = SOWER_ERROR_NONE,
        .event = SOWER_EVENT_EMPTY_CMD};

    if (linear_motor_up() == ESP_OK)
    {
        event.event = SOWER_EVENT_CUTTER_UP;
        event.error = SOWER_ERROR_NONE;
        sower_send_to_uart_transmit_queue(event);
    }
    else
    {
        event.event = SOWER_EVENT_ERROR;
        event.error = SOWER_ERROR_LMOTOR_DONT_RISE;
        sower_send_to_uart_transmit_queue(event);

        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sower_linear_motor_descend_event_handler(void)
{
    sower_event_t event = {
        .arg = 0.0f,
        .error = SOWER_ERROR_NONE,
        .event = SOWER_EVENT_EMPTY_CMD};

    if (linear_motor_down() == ESP_OK)
    {
        event.event = SOWER_EVENT_CUTTER_DOWN;
        event.error = SOWER_ERROR_NONE;
        sower_send_to_uart_transmit_queue(event);
    }
    else
    {
        event.event = SOWER_EVENT_ERROR;
        event.error = SOWER_ERROR_LMOTOR_DONT_RISE;
        sower_send_to_uart_transmit_queue(event);

        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sower_start_dispenser_event_handler(void)
{
    seed_dispenser_cmd_e cmd = SD_CMD_START;

    if (xQueueSend(g_seed_dispenser_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Error: Cannot send start cmd to dispenser. Retrying...");
        return ESP_FAIL;
    }

    seed_dispenser_state_e state;
    if (xQueueReceive(g_seed_dispenser_data_queue, &state, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (state == SD_STATE_STARTED)
        {
            sower_event_t event = {
                .arg = 0.0f,
                .error = SOWER_ERROR_NONE,
                .event = SOWER_EVENT_DISPENSER_STARTED};

            sower_send_to_uart_transmit_queue(event);

            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG, "Error: invalid response for SD_CMD_START");
            sower_event_t event = {
                .arg = 0.0f,
                .error = SOWER_ERROR_DISPENCER_ERROR,
                .event = SOWER_EVENT_ERROR};

            sower_send_to_uart_transmit_queue(event);
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error: response timeout for SD_CMD_START");
        sower_event_t event = {
            .arg = 0.0f,
            .error = SOWER_ERROR_DISPENCER_ERROR,
            .event = SOWER_EVENT_ERROR};

        sower_send_to_uart_transmit_queue(event);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sower_stop_dispenser_event_handler(void)
{
    seed_dispenser_cmd_e cmd = SD_CMD_STOP;

    if (xQueueSend(g_seed_dispenser_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Error: Cannot send stop cmd to dispenser. Retrying...");
        return ESP_FAIL;
    }

    seed_dispenser_state_e state;
    if (xQueueReceive(g_seed_dispenser_data_queue, &state, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (state == SD_STATE_STARTED)
        {
            sower_event_t event = {
                .arg = 0.0f,
                .error = SOWER_ERROR_NONE,
                .event = SOWER_EVENT_DISPENSER_STOPPED};

            sower_send_to_uart_transmit_queue(event);

            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG, "Error: invalid response for SD_CMD_STOP");
            sower_event_t event = {
                .arg = 0.0f,
                .error = SOWER_ERROR_DISPENCER_ERROR,
                .event = SOWER_EVENT_ERROR};

            sower_send_to_uart_transmit_queue(event);
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error: response timeout for SD_CMD_STOP");
        sower_event_t event = {
            .arg = 0.0f,
            .error = SOWER_ERROR_DISPENCER_ERROR,
            .event = SOWER_EVENT_ERROR};

        sower_send_to_uart_transmit_queue(event);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t sower_echo_event_handler()
{
    sower_event_t event = {
        .arg = 0.0f,
        .error = SOWER_ERROR_NONE,
        .event = SOWER_EVENT_ECHO_MSG};

    sower_send_to_uart_transmit_queue(event);

    return ESP_OK;
}

esp_err_t sower_default_event_handler(void)
{
    sower_event_t event = {
        .arg = 0.0f,
        .error = SOWER_ERROR_UNKNOWN_CMD,
        .event = SOWER_EVENT_ERROR};

    // sower_send_to_uart_transmit_queue(event);

    return ESP_OK;
}

void sower_event_handler_loop(void *args)
{
    ESP_LOGI(TAG, "Receive_cmd task started...");

    sower_cmd_t received_data = {
        .arg = 0.0,
        .code = SOWER_CMD_EMPTY,
    };

    while (true)
    {
        /*if (xQueueReceive(esp32_queue_received, &received_data, pdMS_TO_TICKS(100)) == pdPASS)
        {
            switch (received_data.code)
            {
            case SOWER_CMD_STOP_CUTTER:
                ESP_LOGI(TAG, "STOP CUTTER EVENT");
                sower_stop_cutter_event_handler();
                break;

            case SOWER_CMD_START_CUTTER:
                ESP_LOGI(TAG, "START CUTTER EVENT");
                sower_start_cutter_event_handler();
                break;

            case SOWER_CMD_DISTANCE:
                ESP_LOGI(TAG, "DISTANCE EVENT");
                sensor_info_2_uart(distance_sensor);
                break;

            case SOWER_CMD_RPMS:
                ESP_LOGI(TAG, "RPMS EVENT");
                sensor_info_2_uart(rpm_sensor);
                break;

            case SOWER_CMD_LINEAR_MOTOR_UP:
                ESP_LOGI(TAG, "LINEAR MOTOR UP EVENT");
                sower_linear_motor_rise_event_handler();
                break;

            case SOWER_CMD_LINEAR_MOTOR_DOWN:
                ESP_LOGI(TAG, "LINEAR MOTOR DESCEND EVENT");
                sower_linear_motor_descend_event_handler();
                break;

            case SOWER_CMD_START_DISPENSER:
                ESP_LOGI(TAG, "START DISPENSER EVENT");
                sower_start_dispenser_event_handler();
                break;

            case SOWER_CMD_STOP_DISPENSER:
                ESP_LOGI(TAG, "STOP DISPENSER EVENT");

                sower_stop_dispenser_event_handler();
                break;

            case SOWER_CMD_ECHO_SOWER:
                ESP_LOGI(TAG, "ECHO SOWER EVENT");

                sower_echo_event_handler();
                break;

            default:
                ESP_LOGW(TAG, "ERROR CMD EVENT");
                sower_default_event_handler();
                break;
            }
        }*/
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t sensor_info_2_uart(sensors_e sensor)
{
    sower_event_t event = {
        .arg = 0.0,
        .error = 0,
        .event = 0,
    };

    switch (sensor)
    {
    case distance_sensor:
        xQueuePeek(distance_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = SOWER_EVENT_SEED_DISTANCE;

        break;

    case humedity_sensor:
        xQueuePeek(humedity_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = SOWER_EVENT_HUMEDITY_MEASURE;

        break;

    case temperature_sensor:
        xQueuePeek(temperature_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = SOWER_EVENT_TEMPERATURE_MEASURE;

        break;

    case rpm_sensor:
        xQueuePeek(rpm_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = SOWER_EVENT_CUTTER_RPM_MEASURE;

        break;
    }

    sower_send_to_uart_transmit_queue(event);

    return ESP_OK;
}
