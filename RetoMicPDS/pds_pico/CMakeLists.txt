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
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pds_pico C CXX ASM)

pico_sdk_init()

add_executable(pds_pico
    src/main.c
    src/adc_capture.c
    src/goertzel.c
    src/tone_detect.c
    src/fft.c
)

target_include_directories(pds_pico PRIVATE src)
target_compile_options(pds_pico PRIVATE -O2 -Wall -Wextra)

target_link_libraries(pds_pico
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_irq
)

# Salida de consola por USB (no UART)
pico_enable_stdio_usb(pds_pico 1)
pico_enable_stdio_uart(pds_pico 0)

pico_set_program_name(pds_pico "pds_pico")
pico_add_extra_outputs(pds_pico)
