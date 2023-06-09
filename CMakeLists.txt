cmake_minimum_required(VERSION 3.12)

# Comment out these lines if compiling directly on arm ... hello MBP Mx users :)

set(CMAKE_C_COMPILER "/usr/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/arm-none-eabi-g++")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)

pico_sdk_init()

link_directories(libmicroros)
link_directories(Vacuumizer)

add_executable(main
    main.c
    Vacuumizer/pico_uart_transport.c
    Vacuumizer/Vacuumizer.c
)

target_link_libraries(main
    pico_stdlib
    hardware_spi
    hardware_i2c
    microros
)

target_include_directories(main PUBLIC
    libmicroros/include
    Vacuumizer
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(main)
