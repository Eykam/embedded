cmake_minimum_required(VERSION 3.16)


set(component_srcs 
    "core/VL53L1X_api.c" 
    "platform/vl53l1_platform.c"
)

idf_component_register(
    SRCS ${component_srcs}
    INCLUDE_DIRS "include"
    REQUIRES driver
)
