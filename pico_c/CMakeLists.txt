cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(switch_network C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(switch_network
    src/main.c
)

# Pull in common dependencies
target_link_libraries(switch_network
    pico_stdlib
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(switch_network 1)
pico_enable_stdio_uart(switch_network 0)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(switch_network)