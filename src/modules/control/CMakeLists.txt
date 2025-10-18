# CMakeLists.txt for control library
cmake_minimum_required(VERSION 3.14)

# Define the library name
set(LIBRARY_NAME control)

# Source files for the control library
set(SOURCE_FILES
    angle/angle_controller.cpp
    velocity/velocity_controller.cpp
)

# Header files for the control library
set(HEADER_FILES
    angle/angle_controller.h
    velocity/velocity_controller.h
)

# Create the static library
add_library(control STATIC ${SOURCE_FILES} ${HEADER_FILES})

# Add include directories
target_include_directories(control PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)

# Add test subdirectory
add_subdirectory(test)
