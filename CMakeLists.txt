# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
#set(PICO_BOARD seeed_xiao_rp2040 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Set the project name
set(CMAKE_PROJECT_NAME MIDI2Toy)

project(${CMAKE_PROJECT_NAME} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(${CMAKE_PROJECT_NAME} main.cpp usb_descriptors.c
        libs/tusb_ump/ump_device.cpp
        libs/AM_MIDI2.0Lib/src/umpMessageCreate.cpp
        libs/AM_MIDI2.0Lib/src/midiCIMessageCreate.cpp
        libs/AM_MIDI2.0Lib/src/midiCIProcessor.cpp
        libs/AM_MIDI2.0Lib/src/umpProcessor.cpp
        libs/AM_MIDI2.0Lib/src/mcoded7.cpp
)

pico_set_program_name(${CMAKE_PROJECT_NAME} "${CMAKE_PROJECT_NAME}")
pico_set_program_version(${CMAKE_PROJECT_NAME} "0.1")

# Add the standard library to the build
target_link_libraries(${CMAKE_PROJECT_NAME} pico_stdlib
        pico_multicore
        hardware_gpio
        hardware_adc
        tinyusb_device tinyusb_board)

# Add the standard include files to the build
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/tusb_ump
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/AM_MIDI2.0Lib/include
)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 0)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
