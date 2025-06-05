idf_component_register(SRCS "state_machine.c"
                    INCLUDE_DIRS "include"
                    REQUIRES    "sensors"
                    REQUIRES    "uart_communication" 
                    REQUIRES    "as5600_sensor"
                    REQUIRES    "control"
                    REQUIRES    seed_dispenser)