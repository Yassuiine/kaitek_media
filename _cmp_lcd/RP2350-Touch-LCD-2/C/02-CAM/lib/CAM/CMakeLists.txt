
# Search for all source files in the current directory
# and save the names to the DIR_CAM_SRCS variable
aux_source_directory(. DIR_CAM_SRCS)

include_directories(../Config)
include_directories(../LCD)

# Generate link library
add_library(CAM STATIC ${DIR_CAM_SRCS})
target_link_libraries(CAM PUBLIC Config LCD pico_stdlib hardware_i2c hardware_pwm hardware_dma hardware_pio)
