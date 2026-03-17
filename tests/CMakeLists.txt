cmake_minimum_required(VERSION 3.10)

# Build the kinematics test executable from test_kinematics.cpp.
# 'spm' is the static library defined in the parent CMakeLists.txt;
# linking it automatically propagates the include/ directory so that
# test_kinematics.cpp can #include "mSPMModel.h", "mMath.h", etc.
add_executable(test_kinematics test_kinematics.cpp)
target_link_libraries(test_kinematics spm)

# Register with CTest so that "ctest" / "CMake: Run Tests" discovers this test.
# COMMAND is the executable to run; a non-zero exit code marks the test as failed.
add_test(NAME KinematicsTest COMMAND test_kinematics)
