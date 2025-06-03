idf_component_register(SRCS "velocity_task.c" "as5600_sensor.c" 
                    INCLUDE_DIRS "include"
                    REQUIRES "driver"
                    REQUIRES "esp_timer"
                    )


