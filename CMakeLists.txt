cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(TMC2209 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

add_executable(${PROJECT_NAME} TMC2209.c)

target_link_libraries(${PROJECT_NAME} pico_stdlib)
target_link_libraries(${PROJECT_NAME} hardware_uart)


pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

pico_add_extra_outputs(${PROJECT_NAME})
