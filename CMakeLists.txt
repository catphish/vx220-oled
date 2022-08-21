cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(display C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(display)
target_sources(display PRIVATE display.c)
target_link_libraries(display PRIVATE pico_stdlib hardware_spi)

# pico_enable_stdio_usb(display 1)
# pico_enable_stdio_uart(display 0)
pico_add_extra_outputs(display)
