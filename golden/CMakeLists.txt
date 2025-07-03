# CMakeLists.txt for bootloader
cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_bootloader C CXX ASM)
pico_sdk_init()

add_executable(pico_bootloader main.c)
target_link_libraries(pico_bootloader 
	pico_stdlib
	pico_unique_id
)

pico_set_uf2_family(pico_bootloader rp2350-arm-s)
pico_embed_pt_in_binary(pico_bootloader "${CMAKE_CURRENT_LIST_DIR}/pt.json")

pico_enable_stdio_usb(pico_bootloader 1)
pico_enable_stdio_uart(pico_bootloader 0)
pico_add_extra_outputs(pico_bootloader)

