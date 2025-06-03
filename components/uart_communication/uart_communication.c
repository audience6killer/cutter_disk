#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#include "uart_communication.h"

static QueueHandle_t communication_esp32_queue = NULL; 
static QueueHandle_t g_queue_data2send = NULL;
static QueueHandle_t g_queue_data_received = NULL;   

static const char TAG[] = "ESP32 uart";

esp_err_t esp32_uart_get_queue_data2send(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_queue_data2send != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_queue_data2send;
    return ESP_OK;
}

esp_err_t esp32_uart_get_queue_data_received(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_queue_data_received != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_queue_data_received;
    return ESP_OK;
}

static void esp32_uart_receive_task(void *pvParameters)
{
    uart_event_t event_uart_rx;

    uint8_t *data = (uint8_t *)malloc(ESP32_UART_BUFFER_SIZE);
    data_center_esp32_t cmd = {
        .arg = 0.0,
        .code = CMD_EMPTY,
    };

    char message_to_queue[ESP32_DATA_LENGTH];
    uart_flush(ESP32_UART_PORT);

    for(;;)
    {
        if (xQueueReceive(communication_esp32_queue, (void *)&event_uart_rx, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Clean buffer.
            memset(data, 0, ESP32_UART_BUFFER_SIZE);

            switch (event_uart_rx.type)
            {
            case UART_DATA:
                // read from uart
                uart_read_bytes(ESP32_UART_PORT, (char *)&cmd, sizeof(data_center_esp32_t), pdMS_TO_TICKS(100));

                printf("%d, %0.2f \r\n", cmd.code, cmd.arg);

                // send queue
                if(xQueueSend(g_queue_data_received, message_to_queue, pdMS_TO_TICKS(100)) == pdFAIL)
                {
                    ESP_LOGE(TAG, "Error sending data to queue");
                }

                // Clean input.
                uart_flush(ESP32_UART_PORT);
                break;

            default:
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(data);
    data = NULL;
}

static void esp32_uart_transmit_task(void *pvParameters)
{
    BaseType_t status_queue;

    cutter_disk_event_t received_data = {
        .arg = 0.0,
        .error = 0,
        .event = 0,
    };

    for(;;)
    {
        // If it detects a queue.
        status_queue = xQueueReceive(g_queue_data2send, &received_data, pdMS_TO_TICKS(WAIT_QUEUE_SEND_ESP32));

        // Manages the information and send to master.
        if (status_queue == pdPASS)
        {
            // Writes information to the UART port
            uart_write_bytes(ESP32_UART_PORT, (char *)&received_data, sizeof(received_data));

            // Cleans result.
            received_data.arg = 0.0;
            received_data.error = 0;
            received_data.event = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t esp32_uart_task_init(void)
{
    ESP_LOGI(TAG, "ESP32 UART is initializing...");
    uart_config_t uart_RF_configuration = {
        .baud_rate = ESP32_UART_BAUDRATE_RF,
        .data_bits = ESP32_UART_DATA_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    // Creates UART port.
    ESP_ERROR_CHECK(uart_param_config(ESP32_UART_PORT, &uart_RF_configuration));

    // Configurates PINS for UART.
    ESP_ERROR_CHECK(uart_set_pin(ESP32_UART_PORT, ESP32_UART_TX_PIN, ESP32_UART_RX_PIN, ESP32_UART_RTS_PIN, ESP32_UART_CTS_PIN));

    // Prepares UART.
    ESP_ERROR_CHECK(uart_driver_install(ESP32_UART_PORT,
                                        ESP32_UART_BUFFER_SIZE,
                                        ESP32_UART_BUFFER_SIZE,
                                        ESP32_UART_QUEUE_SIZE,
                                        &communication_esp32_queue,
                                        ESP_INTR_FLAG_LEVEL3));

    return ESP_OK;
}

void esp32_uart_task_start(void)
{
    ESP_LOGI(TAG, "ESP32/cutter communication tasks started...");

    ESP_ERROR_CHECK(esp32_uart_task_init());

    g_queue_data2send = xQueueCreate(5, sizeof(cutter_disk_event_t));
    g_queue_data_received = xQueueCreate(5, sizeof(data_center_esp32_t));

    // Creates RF task to receive information.
    xTaskCreatePinnedToCore(esp32_uart_receive_task,
                            "esp32_receive_task",
                            ESP32_UART_TASK_STACK_SIZE,
                            NULL,
                            ESP32_UART_TASK_RECEIVE_PRIORITY,
                            NULL,
                            ESP32_UART_TASK_CORE_ID);

    // Creates RF task to transmit information.
    xTaskCreatePinnedToCore(esp32_uart_transmit_task,
                            "esp32_transmit_task",
                            ESP32_UART_TASK_STACK_SIZE,
                            NULL,
                            ESP32_UART_TASK_TRANSFER_PRIORITY,
                            NULL,
                            ESP32_UART_TASK_CORE_ID);
}

// tipo de suelo 