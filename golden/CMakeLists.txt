# CMakeLists.txt for bootloader
cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_bootloader C CXX ASM)
pico_sdk_init()

add_executable(pico_bootloader main.c)
target_link_libraries(pico_bootloader pico_stdlib)
pico_add_extra_outputs(pico_bootloader)

