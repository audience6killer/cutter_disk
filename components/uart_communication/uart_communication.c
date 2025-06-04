#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#include "uart_communication.h"

static QueueHandle_t g_esp32_uart_queue = NULL;
static QueueHandle_t g_uart_data_to_transmit_queue = NULL;
static QueueHandle_t g_uart_data_received_queue = NULL;

static const char TAG[] = "esp32_uart";

// Protocol constants
#define SOWER_FRAME_START 0xAA
#define SOWER_FRAME_END 0x55
#define SOWER_SERIALIZED_SIZE 8 // 4 bytes for enum + 4 bytes for float

// Serialized frame structure
typedef struct
{
    uint8_t start_byte;
    uint8_t length;
    uint8_t data[SOWER_SERIALIZED_SIZE];
    uint8_t checksum;
    uint8_t end_byte;
} sower_frame_t;

esp_err_t esp32_uart_get_queue_data2send(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_uart_data_to_transmit_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_uart_data_to_transmit_queue;
    return ESP_OK;
}

esp_err_t esp32_uart_get_queue_data_received(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_uart_data_received_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_uart_data_received_queue;
    return ESP_OK;
}

/**
 * @brief Calculate XOR checksum
 */
uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Deserialize byte array into sower_cmd_t
 * @param buffer Input buffer
 * @param cmd Output command struct
 * @return true if successful, false otherwise
 */
bool deserialize_sower_cmd(const uint8_t *buffer, sower_cmd_t *cmd)
{
    if (!buffer || !cmd)
        return false;

    size_t offset = 0;

    // Deserialize command code
    uint32_t cmd_code;
    memcpy(&cmd_code, buffer + offset, sizeof(uint32_t));
    cmd->code = (sower_cmd_e)cmd_code;
    offset += sizeof(uint32_t);

    // Deserialize argument
    memcpy(&cmd->arg, buffer + offset, sizeof(float));
    offset += sizeof(float);

    // Validate command code
    if (cmd->code > SOWER_CMD_ERROR)
    {
        cmd->code = SOWER_CMD_ERROR;
        return false;
    }

    return true;
}

/**
 * @brief Receive sower command from UART with protocol framing
 * @param uart_num UART port number
 * @param cmd Output command struct
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t receive_sower_cmd(uart_port_t uart_num, sower_cmd_t *cmd, uint32_t timeout_ms)
{
    sower_frame_t frame;
    uint8_t byte;

    // Wait for start byte
    while (true)
    {
        int len = uart_read_bytes(uart_num, &byte, 1, timeout_ms / portTICK_PERIOD_MS);
        if (len <= 0)
            return ESP_ERR_TIMEOUT;
        if (byte == SOWER_FRAME_START)
            break;
    }
    frame.start_byte = byte;

    // Read length
    int len = uart_read_bytes(uart_num, &frame.length, 1, timeout_ms / portTICK_PERIOD_MS);
    if (len <= 0)
        return ESP_ERR_TIMEOUT;

    // Validate length
    if (frame.length != SOWER_SERIALIZED_SIZE)
        return ESP_ERR_INVALID_SIZE;

    // Read data
    len = uart_read_bytes(uart_num, frame.data, frame.length, timeout_ms / portTICK_PERIOD_MS);
    if (len != frame.length)
        return ESP_ERR_TIMEOUT;

    // Read checksum
    len = uart_read_bytes(uart_num, &frame.checksum, 1, timeout_ms / portTICK_PERIOD_MS);
    if (len <= 0)
        return ESP_ERR_TIMEOUT;

    // Read end byte
    len = uart_read_bytes(uart_num, &frame.end_byte, 1, timeout_ms / portTICK_PERIOD_MS);
    if (len <= 0)
        return ESP_ERR_TIMEOUT;

    // Validate frame
    if (frame.end_byte != SOWER_FRAME_END)
        return ESP_ERR_INVALID_RESPONSE;

    uint8_t calculated_checksum = calculate_checksum(frame.data, frame.length);
    if (frame.checksum != calculated_checksum)
        return ESP_ERR_INVALID_CRC;

    // Deserialize command
    if (!deserialize_sower_cmd(frame.data, cmd))
        return ESP_ERR_INVALID_ARG;

    return ESP_OK;
}

static void esp32_uart_receive_task(void *pvParameters)
{
    uart_event_t event_uart_rx;

    sower_cmd_t cmd = {
        .arg = 0.0,
        .code = SOWER_CMD_EMPTY,
    };

    for (;;)
    {
        if (xQueueReceive(g_esp32_uart_queue, (void *)&event_uart_rx, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            esp_err_t result;
            switch (event_uart_rx.type)
            {
            case UART_DATA:
                sower_cmd_t received_cmd;
                result = receive_sower_cmd(ESP32_UART_PORT, &received_cmd, 1000); // 1 second timeout
                if (result == ESP_OK)
                {
                    const char *cmd_name = sower_cmd_name(received_cmd.code);
                    printf("Received command: %s, arg: %.2f\n", cmd_name, received_cmd.arg);

                    if(xQueueSend(g_uart_data_received_queue, &received_cmd, pdMS_TO_TICKS(100)) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Error: Failed to send received cmd to queue");
                    }
                }
            default:
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void esp32_uart_transmit_task(void *pvParameters)
{
    sower_event_t received_data = {
        .arg = 0.0,
        .error = 0,
        .event = 0,
    };

    for (;;)
    {
        // Manages the information and send to master.
        if (xQueueReceive(g_uart_data_to_transmit_queue, &received_data, pdMS_TO_TICKS(WAIT_QUEUE_SEND_ESP32)) == pdPASS)
        {
            //printf("Writing data to uart!\n");
            // Writes information to the UART port
            const char *event_name = sower_event_name(received_data.event);
            printf("Event to send to uart: %s\n", event_name);
            uart_write_bytes(ESP32_UART_PORT, (uint8_t *)&received_data, sizeof(received_data));

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
        .source_clk = UART_SCLK_DEFAULT};

    // Creates UART port.
    ESP_ERROR_CHECK(uart_param_config(ESP32_UART_PORT, &uart_RF_configuration));

    // Configurates PINS for UART.
    ESP_ERROR_CHECK(uart_set_pin(ESP32_UART_PORT, ESP32_UART_TX_PIN, ESP32_UART_RX_PIN, ESP32_UART_RTS_PIN, ESP32_UART_CTS_PIN));

    // Prepares UART.
    ESP_ERROR_CHECK(uart_driver_install(ESP32_UART_PORT,
                                        ESP32_UART_BUFFER_SIZE,
                                        ESP32_UART_BUFFER_SIZE,
                                        ESP32_UART_QUEUE_SIZE,
                                        &g_esp32_uart_queue,
                                        ESP_INTR_FLAG_LEVEL3));

    return ESP_OK;
}

void esp32_uart_task_start(void)
{
    ESP_LOGI(TAG, "ESP32/cutter communication tasks started...");

    ESP_ERROR_CHECK(esp32_uart_task_init());

    g_uart_data_to_transmit_queue = xQueueCreate(5, sizeof(sower_event_t));
    g_uart_data_received_queue = xQueueCreate(5, sizeof(sower_cmd_t));

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