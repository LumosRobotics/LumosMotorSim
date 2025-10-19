cmake_minimum_required(VERSION 3.14)

# Integration test executable for closed-loop control
add_executable(closed_loop_control_test closed_loop_control_test.cpp)

# Link against all required libraries
target_link_libraries(closed_loop_control_test
    simulator
    control
    motor_control
    ${GTEST_LIB_FILES}
)

# Include directories
target_include_directories(closed_loop_control_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party/googletest/googletest/include
)

# Add test to CTest
add_test(NAME ClosedLoopControlTest COMMAND closed_loop_control_test)
