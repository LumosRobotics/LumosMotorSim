# CMakeLists.txt for simulator library
cmake_minimum_required(VERSION 3.14)

# Define the library name
set(LIBRARY_NAME simulator)

# Source files for the simulator library
set(SOURCE_FILES
    simulator.cpp
    bldc_motor.cpp
    sensors.cpp
)

# Header files for the simulator library
set(HEADER_FILES
    simulator.h
    bldc_motor.h
    sensors.h
)

# Create the static library
add_library(simulator STATIC ${SOURCE_FILES} ${HEADER_FILES})

# Add test subdirectory
add_subdirectory(test)
