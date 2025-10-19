cmake_minimum_required(VERSION 3.14)

# Test executable for motor control
add_executable(motor_control_test motor_control_test.cpp)

# Link against the motor control library and Google Test
target_link_libraries(motor_control_test
    motor_control
    ${GTEST_LIB_FILES}
)

# Include directories
target_include_directories(motor_control_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party/googletest/googletest/include
)

# Add test to CTest
add_test(NAME MotorControlTest COMMAND motor_control_test)
