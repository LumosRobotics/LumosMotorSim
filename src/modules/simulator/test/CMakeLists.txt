cmake_minimum_required(VERSION 3.14)

# Test executable for BLDC motor
add_executable(bldc_motor_test bldc_motor_test.cpp)

# Link against the simulator library and Google Test
target_link_libraries(bldc_motor_test
    simulator
    ${GTEST_LIB_FILES}
)

# Include directories
target_include_directories(bldc_motor_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party/googletest/googletest/include
)

# Add test to CTest
add_test(NAME BldcMotorTest COMMAND bldc_motor_test)

# Test executable for sensors
add_executable(sensors_test sensors_test.cpp)

# Link against the simulator library and Google Test
target_link_libraries(sensors_test
    simulator
    ${GTEST_LIB_FILES}
)

# Include directories
target_include_directories(sensors_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party/googletest/googletest/include
)

# Add test to CTest
add_test(NAME SensorsTest COMMAND sensors_test)

# Test executable for simulator
add_executable(simulator_test simulator_test.cpp)

# Link against the simulator library and Google Test
target_link_libraries(simulator_test
    simulator
    ${GTEST_LIB_FILES}
    pthread
)

# Include directories
target_include_directories(simulator_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party/googletest/googletest/include
)

# Add test to CTest
add_test(NAME SimulatorTest COMMAND simulator_test)
