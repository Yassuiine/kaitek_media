# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(03-FatFs C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Build helper library from src/
add_subdirectory(src)

add_executable(03-FatFs
    main.cpp
    config/hw_config.c
    src/command.cpp
    src/data_log_demo.c
    tests/app4-IO_module_function_checker.c
    tests/big_file_test.c
    tests/simple.c
    tests/bench.c
    tests/CreateAndVerifyExampleFiles.c
    tests/ff_stdio_tests_with_cwd.c
    lib/CAM/cam.c
    lib/CAM/ov5640.c
)

pico_generate_pio_header(03-FatFs
    ${CMAKE_CURRENT_LIST_DIR}/lib/CAM/picampinos.pio
)

pico_set_program_name(03-FatFs "03-FatFs")
pico_set_program_version(03-FatFs "0.1")

# Enable USB stdio by default for this project
pico_enable_stdio_uart(03-FatFs 0)
pico_enable_stdio_usb(03-FatFs 1)

target_compile_definitions(03-FatFs PRIVATE
    PARAM_ASSERTIONS_ENABLE_ALL=1
    PICO_MALLOC_PANIC=1
    PICO_USE_STACK_GUARDS=1
    USE_PRINTF
)

target_compile_options(03-FatFs PUBLIC
    -Wall
    -Wextra
    -Wshadow
    -Wstack-usage=2048
    -fanalyzer
)

target_include_directories(03-FatFs PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src/include
    ${CMAKE_CURRENT_LIST_DIR}/lib/CAM
)

target_link_libraries(03-FatFs PRIVATE
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
    hardware_clocks
    hardware_adc
    hardware_pwm
    hardware_i2c
)

pico_add_extra_outputs(03-FatFs)

