cmake_minimum_required(VERSION 3.10)
project(SphericalParallelMech VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# For Raspberry Pi cross-compilation (optional)
# set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/raspberry-pi.cmake)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Source files
set(SOURCES
    src/kinematics.cpp
    src/forward_kinematics.cpp
    src/inverse_kinematics.cpp
    src/utils.cpp
)

# Create library
add_library(kinematics STATIC ${SOURCES})

# Create executable
add_executable(kinematics_demo src/main.cpp)
target_link_libraries(kinematics_demo kinematics)

# Enable testing
enable_testing()
add_subdirectory(tests)

# Installation
install(TARGETS kinematics kinematics_demo
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
