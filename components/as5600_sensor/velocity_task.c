#include "velocity_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_check.h"

#include "as5600_sensor.h"


static const char TAG[] = "velocity task";

i2c_master_bus_handle_t bus_handle_as5600 = NULL;
i2c_master_dev_handle_t dev_handle_as5600 = NULL;
QueueHandle_t velocity_info_queue = NULL;

static void velocity_task(void *args);

esp_err_t init_velocity_sense_task()
{
    ESP_LOGI(TAG, "velocity task is initializing...");

#ifdef DISABLE_SENSOR
    ESP_LOGI(TAG, "test only...");
#else
    set_i2c_as5600(AS5600_PIN_SCL, AS5600_PIN_SDA, &dev_handle_as5600, &bus_handle_as5600);
    
    velocity_info_queue = xQueueCreate(1, sizeof(float));
    xTaskCreatePinnedToCore(
        velocity_task,
        "velocity_task",
        STACK_SIZE_VELOCITY * 4,
        NULL,
        TASK_VELOCITY_PRIORITY,
        NULL,
        TASK_VELOCITY_CORE
    );

#endif

    return ESP_OK;
}

esp_err_t get_velocity_task_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(queue != NULL, ESP_ERR_INVALID_STATE,  TAG, "Queue is NULL");
    *queue = velocity_info_queue;
    return ESP_OK;
}

static void velocity_task(void *args)
{   
    ESP_LOGI(TAG, "velocity task started...");
    const int32_t period = 10000;
    float rpms = 0.0;
    float rpms_filtered = 0.0;
    float alpha = 0.5;

    while(true)
    {

#ifdef DISABLE_SENSOR
        rpms_filtered = 0.0; 
#else
        rpms = velocity_rpms(period, &dev_handle_as5600);
        rpms_filtered = ema_filter(alpha, rpms, rpms_filtered);
#endif


#ifdef RPM_SENSOR_TEST
        printf("/*%.2f, %.2f*/ \r\n", rpms_filtered, rpms_filtered/3.40);
#else
        xQueueOverwrite(velocity_info_queue, &rpms_filtered);
#endif
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    
}