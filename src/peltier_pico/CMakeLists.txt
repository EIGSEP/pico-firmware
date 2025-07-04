cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico_T_ctrl C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()




add_executable(pico_T_ctrl 
    main.c 
    hbridge_peltier.c 
    read_temp.c
    runtime_cmd.c
    onewire_library.c # 1-wire driver needed for DS18B20 thermistor
)


# ------------------------------------------------------------------
# 1-Wire / DS18B20 driver (PIO version from pico-examples official RaspberyyPi GitHub)
# Copy these five files into the same directory as this CMakeLists.txt
#   onewire_library.c
#   onewire_library.h
#   onewire_library.pio
#   ow_rom.h
#   ds18b20.h
# ------------------------------------------------------------------

pico_generate_pio_header(
    pico_T_ctrl                                # target
    ${CMAKE_CURRENT_LIST_DIR}/onewire_library.pio
)


#---------  PIO driver needs hardware_pio and DMA --
target_link_libraries(pico_T_ctrl
        pico_stdlib
        hardware_pwm
        hardware_adc      # still used for internal pico temp sensor if needed
        pico_multicore
	hardware_pio
        hardware_dma
)
# ---------------------------------------------------
pico_enable_stdio_usb(pico_T_ctrl 1)
pico_enable_stdio_uart(pico_T_ctrl 0)

pico_add_extra_outputs(pico_T_ctrl)
