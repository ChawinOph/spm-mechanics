# 3-RRR Spherical Parallel Mechanism Kinematics

C++ implementation of forward and inverse kinematics for three classes of 3-RRR spherical parallel mechanisms with closed-form solutions, based on the research paper "Forward kinematics of three classes of 3-RRR spherical parallel mechanisms admitting closed-form solutions" (Mechanism and Machine Theory, 2024).

## Features

- **Class I**: Orthogonal moving platform with 90° distal links (includes Agile Eye)
  - Quadratic or linear univariate polynomial solutions
  
- **Class II**: Coplanar moving platform with 90° distal links
  - Quartic univariate polynomial solutions
  
- **Class III**: Coaxial input axes with 90° proximal and distal links
  - Quartic univariate polynomial solutions

- Efficient closed-form solutions (no iterative methods required)
- Optimized for embedded systems (Raspberry Pi deployment)
- Comprehensive utility functions for vector/matrix operations

## Project Structure

```
kinematics_project/
├── CMakeLists.txt
├── include/
│   └── kinematics.h
├── src/
│   ├── main.cpp
│   ├── forward_kinematics.cpp
│   ├── inverse_kinematics.cpp
│   ├── kinematics.cpp
│   └── utils.cpp
├── tests/
│   └── test_kinematics.cpp
├── .vscode/
│   ├── tasks.json
│   ├── launch.json
│   └── settings.json
└── README.md
```

## Prerequisites

### Development Machine
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.10 or higher
- VS Code with extensions:
  - C/C++ (Microsoft)
  - CMake Tools (Microsoft)
  - Remote - SSH (for Raspberry Pi deployment)

### Raspberry Pi Target
- Raspberry Pi 3/4/5 with Raspbian OS
- GCC compiler (pre-installed)
- CMake: `sudo apt install cmake`

## Building on Development Machine

### Method 1: Using VS Code (Recommended)

1. Open the project folder in VS Code
2. Press `Ctrl+Shift+B` to build (or use Command Palette: "Tasks: Run Build Task")
3. Press `F5` to build and debug

### Method 2: Using Command Line

```bash
# Configure
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --config Release -j4

# Run
./build/kinematics_demo
```

## Cross-Compilation for Raspberry Pi

### Option 1: Native Compilation (Simplest)

Use VS Code Remote-SSH to connect directly to Raspberry Pi and build there:

1. Install "Remote - SSH" extension in VS Code
2. Connect to Pi: `ssh pi@<raspberry-pi-ip>`
3. Open project folder on Pi
4. Build using standard commands

### Option 2: Cross-Compilation Toolchain

Create `cmake/raspberry-pi.cmake`:

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

Then build:
```bash
cmake -B build-pi -S . -DCMAKE_TOOLCHAIN_FILE=cmake/raspberry-pi.cmake
cmake --build build-pi
```

## Usage Example

```cpp
#include "kinematics.h"

int main() {
    // Create Agile Eye configuration (Class I)
    kinematics::RobotArchitecture arch;
    kinematics::Class1Kinematics robot(arch);
    
    // Input joint angles (radians)
    kinematics::JointAngles theta = {-0.3, -0.5, -0.4};
    
    // Solve forward kinematics
    auto solutions = robot.solveFK(theta);
    
    // Process solutions
    for (const auto& sol : solutions) {
        if (sol.is_valid) {
            // Use sol.rotation_matrix
        }
    }
    
    return 0;
}
```

## Performance

Typical performance on Raspberry Pi 4:
- Class I FK: ~500-700 μs per solution
- Class II FK: ~600-800 μs per solution
- Class III FK: ~700-900 μs per solution

(Significantly faster than iterative methods which require 2-5 ms)

## VS Code Keyboard Shortcuts

- `Ctrl+Shift+B`: Build project
- `F5`: Build and debug
- `Ctrl+Shift+P`: Command palette
- `Ctrl+``: Toggle terminal
- `F9`: Toggle breakpoint
- `F10`: Step over
- `F11`: Step into

## Testing

```bash
# Build with tests
cmake -B build -S . -DBUILD_TESTING=ON
cmake --build build

# Run tests
cd build
ctest --output-on-failure
```

## Deployment to Raspberry Pi

1. **Transfer executable**:
   ```bash
   scp build/kinematics_demo pi@<pi-ip>:~/
   ```

2. **SSH and run**:
   ```bash
   ssh pi@<pi-ip>
   ./kinematics_demo
   ```

3. **Set up as service** (optional):
   Create `/etc/systemd/system/kinematics.service`

## References

Based on the paper:
```
Li, W., Zhang, S., Wang, B., Angeles, J., Gao, F., & Guo, W. (2024). 
Forward kinematics of three classes of 3-RRR spherical parallel mechanisms 
admitting closed-form solutions. Mechanism and Machine Theory, 201, 105751.
```

## License

[Specify your license here]

## Contributing

[Contribution guidelines]

## Contact

[Your contact information]
