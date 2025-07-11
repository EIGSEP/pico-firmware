cmake_minimum_required(VERSION 3.13)

# include the Pico SDK
include(pico-sdk/external/pico_sdk_import.cmake)
project(pico_multi C CXX ASM)

# standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize SDK
pico_sdk_init()

# add executable
add_executable(pico_multi
    src/main.c
    src/motor.c
    src/rfswitch.c
    src/tempctrl.c
    src/tempmon.c
    src/temp_simple.c
    src/imu.cpp
    src/lidar.c
)


# include directories
add_library(cjson STATIC lib/cJSON/cJSON.c)
target_include_directories(cjson PUBLIC lib/cJSON)

add_library(eigsep_command STATIC lib/eigsep_command/eigsep_command.c)
target_include_directories(eigsep_command PUBLIC
    lib/eigsep_command
    lib/cJSON
)
target_link_libraries(eigsep_command
  PUBLIC
    cjson
)

add_library(onewire STATIC lib/onewire/onewire_library.c)
target_include_directories(onewire PUBLIC lib/onewire)
target_link_libraries(onewire PUBLIC
    pico_stdlib
    hardware_pio
)
# Generate PIO header for the onewire library
pico_generate_pio_header(onewire ${CMAKE_CURRENT_LIST_DIR}/lib/onewire/onewire_library.pio)

# Add BNO08x library
add_subdirectory(lib/BNO08x_Pico_Library)

target_include_directories(pico_multi PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/lib/BNO08x_Pico_Library/include
)

# link against Pico stdlib and required hardware libraries
target_link_libraries(pico_multi
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_pio
    pico_unique_id
    cjson
    eigsep_command
    onewire
    hardware_i2c
    BNO08x_Pico_Library
)

# enable USB CDC for stdio
pico_enable_stdio_usb(pico_multi 1)
pico_enable_stdio_uart(pico_multi 0)

# generate UF2, map file, etc.
pico_add_extra_outputs(pico_multi)

# set output name
set_target_properties(pico_multi PROPERTIES OUTPUT_NAME "pico_multi")

