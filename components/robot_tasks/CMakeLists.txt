idf_component_register(SRCS "robot_tasks.c"
                    INCLUDE_DIRS "include"
                    REQUIRES    "sensors"
                    REQUIRES    "uart_communication" 
                    REQUIRES    "as5600_sensor"
                    REQUIRES    "control")