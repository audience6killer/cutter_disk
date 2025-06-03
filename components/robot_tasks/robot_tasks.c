#include <stdio.h>
#include <string.h>
#include "robot_tasks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#include "velocity_task.h"
#include "control.h"
#include "sensor_tasks.h"
#include "uart_communication.h"


#include "esp_log.h"

static const char TAG[] = "task event";
static QueueHandle_t esp32_queue_received = NULL;
static QueueHandle_t esp32_queue_send = NULL;

static QueueHandle_t distance_queue = NULL;
static QueueHandle_t humedity_queue = NULL;
static QueueHandle_t temperature_queue = NULL;
static QueueHandle_t rpm_queue = NULL;


static void cmd_events(void *args);
static esp_err_t sensor_info_2_uart(sensors_e sensor);

esp_err_t init_tasks()
{
    // Init velocity task
    init_velocity_sense_task();
    get_velocity_task_queue_handle(&rpm_queue);

    // Init sensors
    init_sensors();
    get_humedity_queue(&humedity_queue);
    get_temperature_queue(&temperature_queue);
    get_distance_queue(&distance_queue);

    // Init PID
    init_cutter_task();
    
    // Init task cmd
    esp32_uart_task_start();
    esp32_uart_get_queue_data_received(&esp32_queue_received);
    esp32_uart_get_queue_data2send(&esp32_queue_send);
    
    ESP_LOGI(TAG, "receive_cmd task is initializing...");
    xTaskCreatePinnedToCore(
        cmd_events,
        "cmd event task",
        TASK_EVENT_STACK_SIZE * 4,
        NULL,
        TASK_EVENT_PRIORITY,
        NULL,
        TASK_EVENT_CORE
    );

    return ESP_OK;
}

void cmd_events(void *args)
{
    ESP_LOGI(TAG, "Receive_cmd task started...");
    
    data_center_esp32_t received_data = {
        .arg = 0.0,
        .code = 0,
    };

    cutter_disk_event_t events = {
        .arg = 0.0,
        .event = 0,
        .error = 0,
    };
    
    while(true)
    {
        if(xQueueReceive(esp32_queue_received, &received_data, pdMS_TO_TICKS(100)) == pdPASS)
        {
            switch (received_data.code)
            {
                case CMD_EMPTY:
                    ESP_LOGW(TAG, "EMPTY_CMD EVENT");
                    events.event = EMPTY_CMD;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));
                break;
         
                case CMD_STOP_CUTTER:
                    ESP_LOGI(TAG, "STOP CUTTER EVENT");

#ifdef TEST_RPMS
                    stop_test_rpms();
#else
                    stop_cutter_motor();
#endif
                    events.event = CUTTER_STOPPED;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));

                break;

                case CMD_START_CUTTER:
                    ESP_LOGI(TAG, "START CUTTER EVENT");
#ifdef TEST_RPMS     
                    test_rpms();
#else              
                    start_cutter_motor();
#endif

                    events.event = CUTTER_STARTED;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));
                break;

                case CMD_DISTANCE:
                    ESP_LOGI(TAG, "DISTANCE EVENT");
                    sensor_info_2_uart(distance_sensor);

                break;

                case CMD_RPMS:
                    ESP_LOGI(TAG, "RPMS EVENT");
                    sensor_info_2_uart(rpm_sensor);
                break;

                case CMD_LINEAR_MOTOR_UP:
                    ESP_LOGI(TAG, "LINEAR MOTOR UP EVENT");
                    linear_motor_up();

                    events.event = CUTTER_UP;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));
                break;

                case CMD_LINEAR_MOTOR_DW:
                    ESP_LOGI(TAG, "LINEAR MOTOR DOWN EVENT");
                    linear_motor_down();

                    events.event = CUTTER_DOWN;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));
                break;

                case CMD_START_DISPENSER:
                    ESP_LOGI(TAG, "START DISPENSER EVENT");

                    events.event = DISPENSER_STARTED;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));
                break;

                case CMD_STOP_DISPENSER:
                    ESP_LOGI(TAG, "STOP DISPENSER EVENT");

                    events.event = DISPENSER_STOPPED;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));
                break;

                default:
                    ESP_LOGW(TAG, "ERROR CMD EVENT");
                    events.event = ERROR_CMD;
                    events.error = NONE;
                    xQueueSend(esp32_queue_send, &events, pdMS_TO_TICKS(5));
                break;
        
            }
            
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t sensor_info_2_uart(sensors_e sensor)
{
    cutter_disk_event_t event = {
        .arg = 0.0,
        .error = 0,
        .event = 0,
    };

    switch (sensor)
    {
    case distance_sensor:
        xQueuePeek(distance_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = SEED_DISTANCE;
        
        break;
    
    case humedity_sensor:
        xQueuePeek(humedity_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = HUMEDITY_MEASURE;

        break;
    
    case temperature_sensor:
        xQueuePeek(temperature_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = TEMPERATURE_MEASURE;

        break;

    case rpm_sensor:
        xQueuePeek(rpm_queue, &event.arg, pdMS_TO_TICKS(10));
        event.event = CUTTER_RPM_MEASURE;

        break;        
    }

    xQueueSend(esp32_queue_send, &event, pdMS_TO_TICKS(10));

    return ESP_OK;
}
