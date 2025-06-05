#ifndef SEED_DISPENSER_H
#define SEED_DISPENSER_H

typedef enum
{
    SD_CMD_EMPTY = 0,
    SD_CMD_START,
    SD_CMD_STOP,
} seed_dispenser_cmd_e;

typedef enum {
    SD_STATE_STOPPED = 0,
    SD_STATE_STARTED,
} seed_dispenser_state_e;

esp_err_t seed_dispenser_get_cmd_queue(QueueHandle_t *handle);

esp_err_t seed_dispenser_get_data_queue(QueueHandle_t *handle);

void seed_dispenser_task_start();

#endif