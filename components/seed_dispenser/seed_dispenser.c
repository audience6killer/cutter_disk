#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "driver/gptimer.h"

#include "bdc_motor.h"
#include "pid_ctrl.h"

#include "seed_dispenser_task_common.h"
#include "seed_dispenser.h"

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
    int desired_speed;      // In pulses
} motor_control_context_t;

static const char *TAG = "seed_dispenser";
static motor_control_context_t *g_seed_dispenser_handle = NULL;
static QueueHandle_t g_seed_dispenser_isr_data_queue = NULL;
static QueueHandle_t g_seed_dispenser_cmd_queue = NULL;
static QueueHandle_t g_seed_dispenser_data_queue = NULL;
static gptimer_handle_t g_seed_dispenser_pid_gptimer;
static seed_dispenser_state_e g_seed_dispenser_state = SD_STATE_STOPPED;
static bool g_is_timer_running = false;

esp_err_t seed_dispenser_update_state(seed_dispenser_cmd_e state)
{
    g_seed_dispenser_state = state;
    seed_dispenser_state_e s = state;

    if(xQueueSend(g_seed_dispenser_data_queue, &s, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Error: Failed to send state to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t seed_dispenser_get_cmd_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_seed_dispenser_cmd_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "g_seed_dispenser_cmd_queue is null");

    *handle = g_seed_dispenser_cmd_queue;

    return ESP_OK;
}

esp_err_t seed_dispenser_get_data_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_seed_dispenser_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "g_seed_dispenser_data_queue is null");

    *handle = g_seed_dispenser_data_queue;

    return ESP_OK;
    
}

static bool IRAM_ATTR seed_dispenser_pid_isr_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_ctx)
{
    static int seed_dispenser_last_pulse_count = 0;

    /* Read current encoder counts */
    int seed_dispenser_cur_pulse_count = 0;
    pcnt_unit_get_count(g_seed_dispenser_handle->pcnt_encoder, &seed_dispenser_cur_pulse_count);

    /* Calculate pulses since the last callback */
    int seed_dispenser_real_pulses = seed_dispenser_cur_pulse_count - seed_dispenser_last_pulse_count;

    seed_dispenser_last_pulse_count = seed_dispenser_cur_pulse_count;

    /* Send data to the queue (use ISR-safe function) */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_seed_dispenser_isr_data_queue, &seed_dispenser_real_pulses, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return true;
}

static void seed_dispenser_update_task(void *pvParameters)
{
    int seed_dispenser_real_pulses;

    for(;;)
    {
        if(xQueueReceive(g_seed_dispenser_isr_data_queue, &seed_dispenser_real_pulses, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            float seed_dispenser_new_speed = 0.0f;

            float seed_dispenser_error = g_seed_dispenser_handle->desired_speed - abs(seed_dispenser_real_pulses);

            pid_compute(g_seed_dispenser_handle->pid_ctrl, seed_dispenser_error, &seed_dispenser_new_speed);

            bdc_motor_set_speed(g_seed_dispenser_handle->motor, (uint32_t)seed_dispenser_new_speed);

            g_seed_dispenser_handle->report_pulses = seed_dispenser_real_pulses;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Event handlers */
esp_err_t seed_dispenser_start_event_handler(void)
{
    if(!g_is_timer_running)
    {
        ESP_ERROR_CHECK(gptimer_start(g_seed_dispenser_pid_gptimer));
        seed_dispenser_update_state(SD_STATE_STARTED);
        ESP_LOGI(TAG, "Event: Timer is running");
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "Warning: Timer is already started");
        return ESP_FAIL;
    }
}

esp_err_t seed_dispenser_stop_event_handler(void)
{
    if(g_is_timer_running)
    {
        ESP_ERROR_CHECK(gptimer_stop(g_seed_dispenser_pid_gptimer));
        seed_dispenser_update_state(SD_STATE_STOPPED);
        ESP_LOGI(TAG, "Event: Timer is stopped");
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "Warning: Timer is already stopped");
        return ESP_FAIL;
    }
}

void seed_dispenser_event_loop(void)
{
    seed_dispenser_cmd_e cmd;
    if(xQueueReceive(g_seed_dispenser_cmd_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        switch (cmd)
        {
        case SD_CMD_EMPTY:
            ESP_LOGI(TAG, "CMD Received: Empty");
            break;
        case SD_CMD_START:
            ESP_LOGI(TAG, "CMD Received: Start");
            seed_dispenser_start_event_handler();
            break;
        case SD_CMD_STOP:
            ESP_LOGI(TAG, "CMD Received: Stop");
            seed_dispenser_stop_event_handler();
            break;
        default:
            break;
        }
    }
}

static void seed_dispenser_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing task");

    g_seed_dispenser_handle = (motor_control_context_t *)malloc(sizeof(motor_control_context_t));

    /* Configure BDC_MOTOR */
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = SEED_DISPENSER_PWM_A,
        .pwmb_gpio_num = SEED_DISPENSER_PWM_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    g_seed_dispenser_handle->motor = motor;

    /* Configure encoder pcnt counter */
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = SEED_DISPENSER_ENCODER_A,
        .level_gpio_num = SEED_DISPENSER_ENCODER_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = SEED_DISPENSER_ENCODER_B,
        .level_gpio_num = SEED_DISPENSER_ENCODER_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    g_seed_dispenser_handle->pcnt_encoder = pcnt_unit;

    /* Configure PID Control*/
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    g_seed_dispenser_handle->pid_ctrl = pid_ctrl;

    /* Configure GPTimer */
    gptimer_config_t gptimer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000000,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&gptimer_config, &g_seed_dispenser_pid_gptimer));

    gptimer_event_callbacks_t gptimer_callbacks = {
        .on_alarm = seed_dispenser_pid_isr_cb,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_seed_dispenser_pid_gptimer, &gptimer_callbacks, NULL));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = BDC_PID_LOOP_PERIOD_MS * 1000,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(g_seed_dispenser_pid_gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(g_seed_dispenser_pid_gptimer));

    ESP_ERROR_CHECK(bdc_motor_forward(g_seed_dispenser_handle->motor));
    ESP_ERROR_CHECK(bdc_motor_enable(g_seed_dispenser_handle->motor));

    for(;;)
    {
        seed_dispenser_event_loop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void seed_dispenser_task_start()
{
    ESP_LOGI(TAG, "Staring task");

    g_seed_dispenser_isr_data_queue = xQueueCreate(5, sizeof(int));
    g_seed_dispenser_data_queue = xQueueCreate(5, sizeof(seed_dispenser_state_e));

    xTaskCreatePinnedToCore(seed_dispenser_task, "seed_dispenser", SEED_DISPENSER_STACK_SIZE, NULL, SEED_DISPENSER_TASK_PRIORITY, NULL, SEED_DISPENSER_CORE_ID);
    xTaskCreatePinnedToCore(seed_dispenser_update_task, "seed_update", 2048, NULL, 15, NULL, 1);
}
