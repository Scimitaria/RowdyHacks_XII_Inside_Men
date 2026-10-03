cmake_minimum_required(VERSION 3.13)

# Pico 1 (RP2040, non-W). Must be set before importing the SDK.
set(PICO_BOARD pico CACHE STRING "Board type")

if (NOT DEFINED PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{HOME}/pico/pico-sdk)
endif()

include(pico_sdk_import.cmake)

project(pi_pico_slot_counter C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pi_pico_slot_counter main.c)

target_link_libraries(pi_pico_slot_counter pico_stdlib)

# printf over USB serial (/dev/ttyACM*), not UART
pico_enable_stdio_usb(pi_pico_slot_counter 1)
pico_enable_stdio_uart(pi_pico_slot_counter 0)

# Generate .uf2, .bin, .hex, etc.
pico_add_extra_outputs(pi_pico_slot_counter)
