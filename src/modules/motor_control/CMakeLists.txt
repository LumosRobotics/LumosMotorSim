# CMakeLists.txt for motor control library
cmake_minimum_required(VERSION 3.14)

# Define the library name
set(LIBRARY_NAME motor_control)

# Source files for the motor control library
set(SOURCE_FILES
    motor_control.cpp
)

# Header files for the motor control library
set(HEADER_FILES
    motor_control.h
)

# Create the static library
add_library(motor_control STATIC ${SOURCE_FILES} ${HEADER_FILES})

# Add include directories
target_include_directories(motor_control PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

# Add test subdirectory
add_subdirectory(test)
