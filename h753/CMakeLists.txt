cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H753UDP)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/comm/mavlink_udp.c
    App/hal/hardware_manager.c
    App/motors/servo_controller.c
    App/motors/dc_controller.c
    App/motors/robomaster_controller.c
    App/motors/rs485_controller.c
    App/motors/motor_registry.c
    App/config/parameter_manager.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App
    App/comm
    App/config
    App/hal
    App/motors
    Lib/mavlink/c_library_v2
    Lib/mavlink/c_library_v2/robomaster_motor  # Custom motor messages (includes common)
    Lib/mavlink/c_library_v2/common
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    # USE_NUCLEO_H753ZI is already defined in Core/Inc/stm32h7xx_nucleo_conf.h
)

# Suppress MAVLink warnings
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wno-unused-parameter
    -Wno-address-of-packed-member
    -Wno-packed-not-aligned
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
