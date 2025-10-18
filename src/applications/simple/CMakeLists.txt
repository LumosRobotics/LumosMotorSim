cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

set(CPP_SOURCE_FILES main.cpp)

add_executable(simple ${CPP_SOURCE_FILES})

# Link against the simulator library
target_link_libraries(simple simulator)

# Include directories for headers
target_include_directories(simple PRIVATE ${CMAKE_SOURCE_DIR}/src)
