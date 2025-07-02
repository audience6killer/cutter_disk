#include "control.h"

#include "esp_log.h"
#include "esp_check.h"
#include "velocity_task.h"
#include "pid_ctrl.h"

static const char TAG[] = "PID task";
static TaskHandle_t pid_cutter_handle = NULL;
static esp_timer_handle_t pid_loop_timer = NULL;
static QueueHandle_t rpm_queue = NULL;



static pid_ctrl_block_handle_t cutter_pid_handler = NULL;

static pid_ctrl_parameter_t cutter_pid = {
    .ki = 5.0,//1.0,// 0.75,//0.9, 0.1,
    .kd = 3.0,//0.20,//1E-10,
    .kp = 1.5,//2.0,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .max_output = BDC_MCPWM_DUTY_TICK_MAX,
    .min_output = 0,
    .max_integral = 1000,
    .min_integral = -1000,
};


motors_vars_t cutter_motor = {
    .ena_A = CUTTER_ENA_A,
    .ena_B = CUTTER_ENA_B,
    .group_id = 0,
    .pwm_A = CUTTER_PWM_A,
    .pwm_B = CUTTER_PWM_B,
};

motors_vars_t linear_motor = {
    .ena_A = LINEAR_MOTOR_ENA_A,
    .ena_B = LINEAR_MOTOR_ENA_B,
    .group_id = 1,
    .pwm_A = LINEAR_MOTOR_PWM_A,
    .pwm_B = LINEAR_MOTOR_PWM_B,
};

static void wakeup_pid_block(void *args);
static void cutter_pid_task(void *args);

esp_err_t init_cutter_task()
{
    ESP_LOGI(TAG, "cutter task is initializing...");
    // -- motor cutter --
    init_motor(&cutter_motor);
    ESP_ERROR_CHECK(bdc_motor_enable(cutter_motor.motor_handler));
    ESP_ERROR_CHECK(bdc_motor_forward(cutter_motor.motor_handler));
    ESP_ERROR_CHECK(bdc_motor_set_speed(cutter_motor.motor_handler, (uint32_t)BDC_DUTY_CYCLE(PWM_OFF)));


    // -- linear motor --
    init_motor(&linear_motor);
    ESP_ERROR_CHECK(bdc_motor_enable(linear_motor.motor_handler));
    ESP_ERROR_CHECK(bdc_motor_forward(linear_motor.motor_handler));
    ESP_ERROR_CHECK(bdc_motor_set_speed(linear_motor.motor_handler, (uint32_t)BDC_DUTY_CYCLE(PWM_OFF)));

    initialize_relay_switch_sensors_ml(LINEAR_SENSOR_DW, LINEAR_SENSOR_UP);

    // Task init
    get_velocity_task_queue_handle(&rpm_queue);

    pid_ctrl_config_t cutter_pid_config = {
        .init_param = cutter_pid,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&cutter_pid_config, &cutter_pid_handler));

    xTaskCreatePinnedToCore(
        cutter_pid_task,
        "cutter pid",
        STACK_SIZE_CONTROL * 4,
        NULL,
        TASK_CONTROL_PRIORITY,
        &pid_cutter_handle,
        TASK_CONTROL_CORE
    );

    // create timer for PID block
    const esp_timer_create_args_t periodic_timer_args_pid = {
        .callback = wakeup_pid_block,
        .name = "pid loop timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args_pid, &pid_loop_timer));
    
    return ESP_OK;
}

esp_err_t start_cutter_motor()
{
    if(pid_loop_timer == NULL)
    {
        ESP_LOGE(TAG, "PID timer is NULL");
        return ESP_OK;
    }

    if(esp_timer_is_active(pid_loop_timer))
    {
        ESP_LOGW(TAG, "PID timer is already started");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting PID timer");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, PID_LOOP_PERIOD_MS));
    enable_pins_bts7960(cutter_motor.ena_A, cutter_motor.ena_B);
    enable_pins_bts7960(linear_motor.ena_A, linear_motor.ena_B);

    return ESP_OK;
}

esp_err_t stop_cutter_motor()
{
    if(pid_loop_timer == NULL)
    {
        ESP_LOGE(TAG, "PID timer is NULL");
        return ESP_OK;
    }

    if(!esp_timer_is_active(pid_loop_timer))
    {
        ESP_LOGW(TAG, "PID loop is already stopped");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping PID loop");
    ESP_ERROR_CHECK(esp_timer_stop(pid_loop_timer));

    // wait 100 ms
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(bdc_motor_set_speed(cutter_motor.motor_handler, (uint32_t)BDC_DUTY_CYCLE(PWM_OFF)));
    vTaskDelay(pdMS_TO_TICKS(5));
    disable_pins_bts7960(cutter_motor.ena_A, cutter_motor.ena_B);

    // reverse motor 
    ESP_LOGI(TAG, "Put linear motor in up state");
    enable_pins_bts7960(linear_motor.ena_A, linear_motor.ena_B);
    ESP_ERROR_CHECK(bdc_motor_reverse(linear_motor.motor_handler));
    ESP_ERROR_CHECK(bdc_motor_set_speed(linear_motor.motor_handler, BDC_DUTY_CYCLE(PWM_LINEAR_MOTOR)));
    
    // up linear motor motor

    /*
    uint8_t is_up = 0;
    while(is_up)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        is_up = gpio_get_level(LINEAR_SENSOR_UP);
    }
    */

    vTaskDelay(pdMS_TO_TICKS(5000));

    disable_pins_bts7960(linear_motor.ena_A, linear_motor.ena_B);
    ESP_ERROR_CHECK(bdc_motor_set_speed(linear_motor.motor_handler, BDC_DUTY_CYCLE(PWM_OFF)));
    ESP_LOGI(TAG, "linear motor in up state");
    return ESP_OK;
}

esp_err_t linear_motor_up()
{
    ESP_LOGI(TAG, "Linear motor up state...");
    enable_pins_bts7960(linear_motor.ena_A, linear_motor.ena_B);
    ESP_ERROR_CHECK(bdc_motor_forward(linear_motor.motor_handler));
    ESP_ERROR_CHECK(bdc_motor_set_speed(linear_motor.motor_handler, BDC_DUTY_CYCLE(PWM_LINEAR_MOTOR)));

    return ESP_OK;
}

esp_err_t linear_motor_down()
{
    ESP_LOGI(TAG, "Linear motor down state...");
    enable_pins_bts7960(linear_motor.ena_A, linear_motor.ena_B);
    ESP_ERROR_CHECK(bdc_motor_reverse(linear_motor.motor_handler));
    ESP_ERROR_CHECK(bdc_motor_set_speed(linear_motor.motor_handler, BDC_DUTY_CYCLE(PWM_LINEAR_MOTOR)));

    return ESP_OK;
}

esp_err_t linear_motor_stop()
{
    ESP_LOGI(TAG, "Linear motor stop state...");
    disable_pins_bts7960(linear_motor.ena_A, linear_motor.ena_B);
    ESP_ERROR_CHECK(bdc_motor_set_speed(linear_motor.motor_handler, BDC_DUTY_CYCLE(PWM_OFF)));

    return ESP_OK;
}

uint8_t is_linear_motor_up_down()
{
    uint8_t state_up = 0;
    uint8_t state_dw = 0;


    state_up = gpio_get_level(LINEAR_SENSOR_UP);
    state_dw = gpio_get_level(LINEAR_SENSOR_DW);

    if(state_up == 1 && state_dw == 0)
    {
        return 0;
    }else if(state_up == 0 && state_dw == 1)
    {
        return 1;
    }else if(state_up == 1 && state_dw == 1)
    {
        disable_pins_bts7960(linear_motor.ena_A, linear_motor.ena_B);
        return 3;
    }else if(state_up == 0 && state_dw == 0)
    {
        return 2;
    }else
    {
        return 3;
    }
}

esp_err_t test_rpms()
{
    ESP_LOGI(TAG, "Starting RPM test..");
    enable_pins_bts7960(cutter_motor.ena_A, cutter_motor.ena_B);
    ESP_ERROR_CHECK(bdc_motor_set_speed(cutter_motor.motor_handler, BDC_DUTY_CYCLE(PWM_CUTTER_MOTOR_TEST)));

    return ESP_OK;
}

esp_err_t stop_test_rpms()
{
    ESP_LOGI(TAG, "Stoping RPM test...");
    disable_pins_bts7960(cutter_motor.ena_A, cutter_motor.ena_B);
    ESP_ERROR_CHECK(bdc_motor_set_speed(cutter_motor.motor_handler, BDC_DUTY_CYCLE(PWM_CUTTER_MOTOR_TEST)));
    
    return ESP_OK;
}

static void wakeup_pid_block(void *args)
{
    xTaskNotifyGive(pid_cutter_handle);
}

static void cutter_pid_task(void *args)
{
    ESP_LOGI(TAG, "cutter pid started...");
    float rpms = 0.0;
    float rpms_desired = 800;//800;//400;
    float error = 0.0;
    float new_speed = 0.0;

    rpm_queue = xQueueCreate(1, sizeof(float));

    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xQueuePeek(rpm_queue, &rpms, pdMS_TO_TICKS(5));

        error = rpms_desired - rpms;
        pid_compute(cutter_pid_handler, error, &new_speed);
        bdc_motor_set_speed(cutter_motor.motor_handler, (uint32_t)new_speed);

#ifdef CONTROL_TEST
        //printf("/*%0.2f, 400.0, %0.2f, %0.2f*/\r\n", rpms, new_speed, error);
#endif

#ifdef ENABLE_LINEAR_MOTOR
        // put down cutter
        if(rpms >= (rpms_desired - 20) && rpms <= (rpms_desired + 20))
        {

            uint8_t is_down = gpio_get_level(LINEAR_SENSOR_DW);
            if(is_down == 0)
            {
                //esp_event_post_to(loop_task_handle, TASKS_EVENTS, MOTOR_LINEAR_DOWN, NULL, 0, pdMS_TO_TICKS(10));
                //up_down_motor_linear(&motor_linear, true);
                //ESP_ERROR_CHECK(bdc_motor_enable(motor_linear.motor_handler));
                ESP_ERROR_CHECK(bdc_motor_reverse(linear_motor.motor_handler));
                ESP_ERROR_CHECK(bdc_motor_set_speed(linear_motor.motor_handler, BDC_DUTY_CYCLE(50)));
            }
            
        }
#endif

    }

}
