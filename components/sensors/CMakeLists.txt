idf_component_register(SRCS "sensor_tasks.c"
                    INCLUDE_DIRS    "include"
                    REQUIRES        "dht"
                    REQUIRES        "ultrasonic")
