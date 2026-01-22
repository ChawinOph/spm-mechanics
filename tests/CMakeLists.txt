cmake_minimum_required(VERSION 3.10)

# Simple test framework (you can replace with Google Test, Catch2, etc.)
add_executable(test_kinematics test_kinematics.cpp)
target_link_libraries(test_kinematics kinematics)

# Add test
add_test(NAME KinematicsTest COMMAND test_kinematics)
