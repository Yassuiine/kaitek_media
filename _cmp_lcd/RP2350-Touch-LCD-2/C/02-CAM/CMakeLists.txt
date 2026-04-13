# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(main)
pico_sdk_init()

# add compiled files subdirectory
add_subdirectory(lib/Config)
add_subdirectory(lib/LCD)
add_subdirectory(lib/Fonts)
add_subdirectory(lib/GUI)
add_subdirectory(lib/CAM)
add_subdirectory(examples)

# add header file directory
include_directories(examples)
include_directories(./lib/Config)
include_directories(./lib/GUI)
include_directories(./lib/LCD)
include_directories(./lib/CAM)

# generate executable file
add_executable(main 
main.c
)

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/lib/CAM/picampinos.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/CAM)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)

target_link_libraries(main 
    examples 
    LCD 
    GUI 
    Fonts 
    Config
    CAM
    pico_stdlib
    hardware_vreg 
    hardware_spi 
    hardware_i2c 
    hardware_pio
    hardware_dma
    hardware_adc
)
