#ifndef AS5600_SENSOR_H
#define AS5600_SENSOR_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"

/**
 * @brief Structure that contains information related to as5600 information
 * 
 * This structure have information related with angle position, quadrant and number of turns for velocity calcultaion. 
 */
typedef struct
{
    float position_1;               /**< First sample.*/
    float position_2;               /**< Final sample.*/
    uint8_t actual_quadrant;        /**< Quadrant 1 used for turn calculation.*/
    uint8_t previous_quadrant;      /**< Quadrant 2 used for turn calculation.*/
    int32_t turns;                  /**< Number of turns.*/
}as5600_vars_t;

typedef struct
{
    int64_t time_1;     /**< First time sampled.*/
    int64_t time_2;     /**< Final time sampled.*/
    int64_t dt;         /**< Time diference between time_1 and time_2.*/
    int32_t period;     /**< Time period.*/
}time_vars_t;



/** 
 * @brief Set i2c comunication beetwen esp32 and as5600
 * 
 * This function initializes i2c communication between ESP32(master) and AS5600 sensor(slave)
 *
 * @param[in] scl pin scl 
 * @param[in] sda pin sda
 * @param[in] dev i2c handler used by esp32 to comunicate 
 * @param[in] bus i2c handler for initialize 
 */
void set_i2c_as5600(gpio_num_t scl, gpio_num_t sda, i2c_master_dev_handle_t *dev, i2c_master_bus_handle_t *bus);

/**
 * @brief Obtain the raw measure and convert it into angle position.
 * 
 * This function obtain the raw measure and convert it into angle position.
 * 
 * 
 * @param[in] dev i2c handler used by esp32 to comunicate.
 * @return degree 
 */
float as5600_measure_raw(i2c_master_dev_handle_t *dev);


/**
 * @brief Determine the quadrant with the angle position.
 * 
 * This function computes the quadrant position using the angle position.
 * 
 * 
 * @param[in] position_decoded
 * @param[in] actual_quadrant
 */
void which_quadrant(float position_decoded, uint8_t *actual_quadrant);

/**
 * @brief Determine the number of turns.
 * 
 * This function computes the number of turns using cuadrants.
 * 
 * @param[in] turns
 * @param[in] actual_quadrant
 * @param[in] previous_quadrant
 * 
 */
void number_of_turns(int32_t *turns, uint8_t *actual_quadrant, uint8_t *previous_quadrant);

/**
 * @brief Determine the real angle for velocity calculation.
 * 
 * This function computes the absolute angle, with the following expression:
 * 
 * (turns * 360) + corrected_measure
 * 
 * @param[in] turns
 * @param[in] corrected_measure
 * @return real_angle
 */
float true_angle(int32_t turns, float corrected_measure);

/**
 * @brief Calculate rpms using 2 position samples and a time period
 * 
 * This function calculates RPMS based on two angular position samples (degrees) and
 * the time elapsed between them.
 *  
 * @param[in] dt 
 * @param[in] pos1
 * @param[in] pos2
 * @return rpms 
 */
float calculate_rpms(float dt, float pos1, float pos2);

/**
 * @brief Calculate the rpms in a time period.
 * 
 * This function calculates the rpms using a time period beetwen position samples.
 * 
 * @param[in] period_us
 * @param[in] dev_as5600
 * @return rpms
 */
float velocity_rpms(int32_t period_us, i2c_master_dev_handle_t *dev_as5600);

/**
 * @brief EMA filter 
 * 
 * This function have an EMA filter.
 *  
 * @param[in] alpha 
 * @param[in] sample
 * @param[in] previous_sample_filtered
 * @return filtered_signal
 */
float ema_filter(float alpha, float sample, float previous_sample_filtered);

#endif