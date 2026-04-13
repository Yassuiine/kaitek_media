
# Search for all source files in the current directory
# and save the names to the DIR_examples_SRCS variable
aux_source_directory(. DIR_examples_SRCS)

include_directories(../lib/Config)
include_directories(../lib/GUI)
include_directories(../lib/LCD)
include_directories(../lib/CAM)

# Generate link library
add_library(examples ${DIR_examples_SRCS})
target_link_libraries(examples PUBLIC Config LCD CAM hardware_vreg hardware_pio hardware_adc)