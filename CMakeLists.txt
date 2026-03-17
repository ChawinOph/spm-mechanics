# Minimum CMake version required — error if older version is installed
cmake_minimum_required(VERSION 3.10)

# Name the project and set its version.
# Auto-creates: PROJECT_NAME, PROJECT_VERSION, PROJECT_SOURCE_DIR
project(SphericalParallelMech VERSION 1.0)

# Compile all targets as C++17.
# REQUIRED = error if compiler doesn't support it (no silent fallback)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Append compiler flags for every target:
#   -Wall    enable common warnings
#   -Wextra  enable extra warnings beyond -Wall
#   -O2      optimisation level 2
# ${CMAKE_CXX_FLAGS} preserves any flags already set (e.g. by a toolchain file)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# Global include path — adds include/ to #include search for every target.
# (Superseded per-target by target_include_directories below, kept for safety)
include_directories(${PROJECT_SOURCE_DIR}/include)

# ── library ──────────────────────────────────────────────────────────────────

# Build a static library named 'spm' from mSPMModel.cpp.
# STATIC = linked directly into the executable at build time;
#          no separate .so/.dll needed at runtime  →  libspm.a / spm.lib
add_library(spm STATIC
    src/mSPMModel.cpp
)

# Attach include/ to the 'spm' target with PUBLIC visibility:
#   PUBLIC = both 'spm' itself and any target linking against it
#            inherit this include path automatically
#   (contrast: PRIVATE = spm only; INTERFACE = dependents only)
target_include_directories(spm PUBLIC ${PROJECT_SOURCE_DIR}/include)

# ── executable ───────────────────────────────────────────────────────────────

# Build the demo executable from main.cpp
add_executable(spm_demo src/main.cpp)

# Link 'spm' into 'spm_demo'.
# Also propagates spm's PUBLIC include directories, so main.cpp
# can #include "mSPMModel.h" without any extra include_directories call.
target_link_libraries(spm_demo spm)

# ── tests ─────────────────────────────────────────────────────────────────────

# Enable CTest integration (required before add_test() calls take effect).
enable_testing()

# Pull in tests/CMakeLists.txt.  The 'spm' target is already defined above,
# so test_kinematics can link against it without any extra configuration.
add_subdirectory(tests)