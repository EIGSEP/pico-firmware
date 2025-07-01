cmake_minimum_required(VERSION 3.13)

# Include pico_sdk_import.cmake from switches app for now
include(apps/switches/pico_sdk_import.cmake)

# Set project name and language  
project(pico_multi C CXX ASM)

# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Main firmware executable
add_executable(pico_multi
    src/main.c
    src/motor_app.c
    src/switch_app.c
    src/usb_descriptors.c
    apps/motor/motor.c
)

# Include directories
target_include_directories(pico_multi PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/apps/motor
)

# Link libraries
target_link_libraries(pico_multi 
    pico_stdlib
    pico_unique_id
    hardware_watchdog
    hardware_gpio
    hardware_timer
    tinyusb_device
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(pico_multi 1)
pico_enable_stdio_uart(pico_multi 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(pico_multi)

# Set the output filename to pico_multi.uf2
set_target_properties(pico_multi PROPERTIES OUTPUT_NAME "pico_multi")