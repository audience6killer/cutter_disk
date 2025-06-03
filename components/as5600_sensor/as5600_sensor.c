#include "as5600_sensor.h"

#define ADDRESS_AS5600_I2C      0x36
#define SCL_SPEED_HZ_I2C        400000
#define ADDRESS_LENGTH          I2C_ADDR_BIT_7
#define ENABLE_INTERNAL_PULLUP  true
#define GLITCH_IGNORE_CNT       7
#define I2C_PORT_AS5600         I2C_NUM_0
#define CLK_SOURCE_I2C          I2C_CLK_SRC_DEFAULT


void set_i2c_as5600(gpio_num_t scl, gpio_num_t sda, i2c_master_dev_handle_t *dev, i2c_master_bus_handle_t *bus)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = CLK_SOURCE_I2C,
        .i2c_port = I2C_PORT_AS5600,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .glitch_ignore_cnt = GLITCH_IGNORE_CNT,
        .flags.enable_internal_pullup = ENABLE_INTERNAL_PULLUP,
    };

    // Device configuration
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = ADDRESS_LENGTH,
        .device_address = ADDRESS_AS5600_I2C,
        .scl_speed_hz = SCL_SPEED_HZ_I2C,
    };

    // Create I2C bus 
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus, &dev_cfg, dev));
}

float as5600_measure_raw(i2c_master_dev_handle_t *dev)
{
    const uint8_t raw_position_high = 0x0C;
    uint8_t buffer_position[2];
    uint16_t position = 0;
    float degree_position = 0;
    const float conversion_degree = 0.087890625;

    // set pointer to 
    i2c_master_transmit_receive(*dev, &raw_position_high, sizeof(raw_position_high), buffer_position, sizeof(buffer_position), -1);
    
    // transform the information into degrees
    position = (buffer_position[0] << 8) | buffer_position[1];
    degree_position = position * conversion_degree;

    return degree_position;
}

void which_quadrant(float position_decoded, uint8_t *actual_quadrant)
{
    // define the quadrant for know the number of turns 
    if(position_decoded >= 0 && position_decoded <= 90){
        // quadrant 1
        *actual_quadrant = 1;
    }else if(position_decoded > 90 && position_decoded <= 180){
        // quadrant 2
        *actual_quadrant = 2;
    }else if(position_decoded > 180 && position_decoded <= 270){
        // quadrant 3
        *actual_quadrant = 3;
    }else if(position_decoded > 270 && position_decoded <= 360){
        // quadrant 4
        *actual_quadrant = 4;
    }
}

void number_of_turns(int32_t *turns, uint8_t *actual_quadrant, uint8_t *previous_quadrant)
{
    // detect complete turn
    if(*actual_quadrant == 1 && *previous_quadrant == 4){
        // clockwise
        (*turns)++;
    }else if(*actual_quadrant == 4 && *previous_quadrant == 1){
        // counterclockwise
        (*turns)--;
    }

    // update quadrant
    *previous_quadrant = *actual_quadrant;

}

float true_angle(int32_t turns, float corrected_measure)
{
    float total = 0;
    total = (turns * 360) + corrected_measure;

    return total;
}

float calculate_rpms(float dt, float pos1, float pos2)
{
    float time_period_seconds;
	float degree_number;
	float rpms;
	const uint8_t minute_second = 60;
	const uint16_t degree = 360;
	const float convertion_to_seconds = 1E6;

	time_period_seconds = dt/convertion_to_seconds; //1000000.0;
	degree_number = pos2 - pos1;
	rpms = (degree_number * minute_second)/(time_period_seconds * degree);

	return rpms;
}

float velocity_rpms(int32_t period_us, i2c_master_dev_handle_t *dev_as5600)
{
    // as5600 sensor
    as5600_vars_t temp_as5600_data = {
        .actual_quadrant = 0,
        .previous_quadrant = 0,
        .position_1 = 0,
        .position_2 = 0,
        .turns = 0
    };

    // Time data
    time_vars_t temp_time_as5600 = {
        .dt = 0,
        .period = period_us,
        .time_1 = 0,
        .time_2 = 0
    };

    // rpms 
    float rpms = 0.0;

    temp_time_as5600.time_1 = esp_timer_get_time();
    temp_as5600_data.position_1 = as5600_measure_raw(dev_as5600);
    which_quadrant(temp_as5600_data.position_1, &temp_as5600_data.previous_quadrant);

    do
    {
        temp_time_as5600.time_2 = esp_timer_get_time();
        temp_as5600_data.position_2 = as5600_measure_raw(dev_as5600);
        which_quadrant(temp_as5600_data.position_2, &temp_as5600_data.actual_quadrant);
        number_of_turns(&temp_as5600_data.turns, &temp_as5600_data.actual_quadrant, &temp_as5600_data.previous_quadrant);

        temp_time_as5600.dt = temp_time_as5600.time_2 - temp_time_as5600.time_1;
    } while (temp_time_as5600.dt <= temp_time_as5600.period);
        
    temp_as5600_data.position_2 = true_angle(temp_as5600_data.turns, temp_as5600_data.position_2);
    rpms = calculate_rpms(temp_time_as5600.period, temp_as5600_data.position_1, temp_as5600_data.position_2);
    return rpms;
}

float ema_filter(float alpha, float sample, float previous_sample_filtered)
{
    float sample_filtered= 0;

    sample_filtered = (alpha * sample) + ((1 - alpha) * previous_sample_filtered);
    return sample_filtered;
}