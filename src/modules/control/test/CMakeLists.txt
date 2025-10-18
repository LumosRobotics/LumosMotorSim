cmake_minimum_required(VERSION 3.14)

# Test executable for angle controller
add_executable(angle_controller_test angle_controller_test.cpp)

# Link against the control library and Google Test
target_link_libraries(angle_controller_test
    control
    ${GTEST_LIB_FILES}
)

# Include directories
target_include_directories(angle_controller_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party/googletest/googletest/include
)

# Add test to CTest
add_test(NAME AngleControllerTest COMMAND angle_controller_test)

# Test executable for velocity controller
add_executable(velocity_controller_test velocity_controller_test.cpp)

# Link against the control library and Google Test
target_link_libraries(velocity_controller_test
    control
    ${GTEST_LIB_FILES}
)

# Include directories
target_include_directories(velocity_controller_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party/googletest/googletest/include
)

# Add test to CTest
add_test(NAME VelocityControllerTest COMMAND velocity_controller_test)
