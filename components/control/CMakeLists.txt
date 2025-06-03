idf_component_register(SRCS "mypid.c" "motors.c" "control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES     "driver"
                    REQUIRES     "esp_timer"
                    REQUIRES     "as5600_sensor"
                    REQUIRES     "bdc_motor"
                    REQUIRES     "pid_ctrl")
