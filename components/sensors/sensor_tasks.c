#include <stdio.h>

#include "sensor_tasks.h"

#include "esp_log.h"
#include "esp_check.h"
#include "ultrasonic.h"
#include "dht.h"

static const char TAG[] = "Sensors task";
static QueueHandle_t distance_queue = NULL;
static QueueHandle_t humedity_queue = NULL;
static QueueHandle_t temperature_queue = NULL;

static void hcsr04_task(void *arg);
static void am2301_task(void *arg);

esp_err_t get_distance_queue(QueueHandle_t *queue_handle)
{
    ESP_RETURN_ON_FALSE(distance_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *queue_handle = distance_queue;
    return ESP_OK;
}

esp_err_t get_humedity_queue(QueueHandle_t *queue_handle)
{
    ESP_RETURN_ON_FALSE(humedity_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *queue_handle = humedity_queue;
    return ESP_OK;
}

esp_err_t get_temperature_queue(QueueHandle_t *queue_handle)
{
    ESP_RETURN_ON_FALSE(temperature_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *queue_handle = temperature_queue;
    return ESP_OK;
}

esp_err_t init_sensors()
{

    ESP_LOGI(TAG, "sensor tasks are initializing...");

    distance_queue = xQueueCreate(1, sizeof(float));
    humedity_queue = xQueueCreate(1, sizeof(float));
    temperature_queue = xQueueCreate(1, sizeof(float));

    // ultrasonic sensor 
    xTaskCreatePinnedToCore(
        hcsr04_task,       // task's name
        "ultrasonic sensor task",            // debug name
        STACK_SIZE_SENSOR * 2, // stack size
        NULL,                // no  value
        TASK_SENSOR_PRIORITY,
        NULL,
        TASK_SENSOR_CORE
    );

    // temperature humidity sensor 
    xTaskCreatePinnedToCore(
        am2301_task,       // task's name
        "Hum & temp sensor task",            // debug
        STACK_SIZE_SENSOR * 3, // stack size
        NULL,                // no value
        TASK_SENSOR_PRIORITY,
        NULL,
        TASK_SENSOR_CORE
    );

    return ESP_OK;
}

static void hcsr04_task(void *arg)
{
    ESP_LOGI(TAG, "Ultrasonic sensor task started...");
#ifdef SENSORS_ON
    ultrasonic_sensor_t ultrasonic_device = {
        .trigger_pin = PIN_TRIGGER,
        .echo_pin = PIN_ECHO
    };
    
    ESP_ERROR_CHECK(ultrasonic_init(&ultrasonic_device));

    const uint16_t ultrasonic_measure_time_ms = 1000;
    const float max_distance = 1;
    float distance = 0.0;

    esp_err_t ultrasonic_response;
    while(true){

        vTaskDelay(pdMS_TO_TICKS(ultrasonic_measure_time_ms));
        ultrasonic_response = ultrasonic_measure(&ultrasonic_device, max_distance, &distance);
        switch (ultrasonic_response)
        {
        case ESP_OK:
            distance = distance * 100;
            // ESP_LOGI(TAG, "OK");
            break;
        case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
            ESP_LOGW(TAG, "So far");
            distance = -1;
            break;
        
        case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
            ESP_LOGE(TAG, "Device is not responding");
            distance = -2;
            break;

        case ESP_ERR_ULTRASONIC_PING:
            ESP_LOGW(TAG, "Previous ping not ended");
            distance = -3;
            break;
        }
        
        xQueueOverwrite(distance_queue, &distance);
    }
#else 
    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
}

static void am2301_task(void *arg)
{
    ESP_LOGI(TAG, "Hum & Temp sensor task started...");
#ifdef SENSORS_ON
    const uint16_t am2301_measure_time_ms = 1000;
   // const gpio_num_t pin_am2308 = GPIO_NUM_14;
   float data_sensor[2];

    esp_err_t dht_response;
 
    vTaskDelay(pdMS_TO_TICKS(3000));
    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(am2301_measure_time_ms));
        dht_response = dht_read_float_data(DHT_TYPE_AM2301,
                                            PIN_AM2308,
                                            &data_sensor[0],
                                            &data_sensor[1]
                                                        );

        if(dht_response != ESP_OK)
        {
            ESP_LOGE(TAG, "DHT failed");
            data_sensor[0] = 0;
            data_sensor[1] = 0;
        }
        
        xQueueOverwrite(humedity_queue, &data_sensor[0]);
        xQueueOverwrite(temperature_queue, &data_sensor[1]);      
        
    }
#else 
    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
}
