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
    src/potmon.c
    src/temp_simple.c
    src/imu.c
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

target_include_directories(pico_multi PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# link against Pico stdlib and required hardware libraries
target_link_libraries(pico_multi
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_adc
    pico_unique_id
    pico_rand
    cjson
    eigsep_command
    hardware_i2c
    hardware_uart
)

# enable USB CDC for stdio
pico_enable_stdio_usb(pico_multi 1)
pico_enable_stdio_uart(pico_multi 0)
# Emit USB-CDC stdout whenever the device is USB-ready, not only while the host
# asserts DTR. flash-picos / the manager's discovery probe churn open/close (each
# close drops DTR); on a marginal hub a re-asserted DTR control transfer can be
# lost, which otherwise silently suppresses all status (the readback-flicker bug).
target_compile_definitions(pico_multi PRIVATE PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)

# generate UF2, map file, etc.
pico_add_extra_outputs(pico_multi)

# set output name
set_target_properties(pico_multi PROPERTIES OUTPUT_NAME "pico_multi")

# Test image: blinks LED in a heartbeat pattern and exposes USB-CDC so
# the Pico can subsequently be re-flashed with the production image via
# `flash-picos`.
add_executable(pico_test_blink
    src/test_blink.c
)
target_link_libraries(pico_test_blink
    pico_stdlib
    hardware_gpio
    pico_unique_id
)
pico_enable_stdio_usb(pico_test_blink 1)
pico_enable_stdio_uart(pico_test_blink 0)
# Match pico_multi: report status without requiring host DTR (see above).
target_compile_definitions(pico_test_blink PRIVATE PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)
pico_add_extra_outputs(pico_test_blink)
set_target_properties(pico_test_blink PROPERTIES OUTPUT_NAME "pico_test_blink")

# Recovery targets must keep USB stdio: without it, flash-picos cannot
# trigger BOOTSEL for the next remote reflash.
foreach(_recovery_target pico_multi pico_test_blink)
    get_target_property(_stdio_usb ${_recovery_target} PICO_TARGET_STDIO_USB)
    if(NOT _stdio_usb)
        message(FATAL_ERROR
            "${_recovery_target} built without USB stdio — would brick remote reflash. "
            "Restore `pico_enable_stdio_usb(${_recovery_target} 1)` in CMakeLists.txt.")
    endif()
endforeach()
