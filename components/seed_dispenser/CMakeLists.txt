idf_component_register(SRCS "seed_dispenser.c"
                    INCLUDE_DIRS "include"
                    REQUIRES pid_ctrl bdc_motor driver esp_timer)
