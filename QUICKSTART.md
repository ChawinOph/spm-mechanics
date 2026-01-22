# Quick Start Guide

## Setup from Scratch

### 1. Install Required Software

#### Windows:
```powershell
# Install MinGW-w64 or Visual Studio Build Tools
# Install CMake from https://cmake.org/download/
# Install VS Code from https://code.visualstudio.com/
# Install Git from https://git-scm.com/
```

#### Linux (Ubuntu/Debian):
```bash
sudo apt update
sudo apt install build-essential cmake git
```

#### macOS:
```bash
# Install Xcode Command Line Tools
xcode-select --install

# Install CMake
brew install cmake
```

### 2. Setup VS Code

1. **Install VS Code**: Download from https://code.visualstudio.com/

2. **Install Extensions** (Ctrl+Shift+X):
   - C/C++ (Microsoft)
   - CMake Tools (Microsoft)
   - CMake (twxs)
   - Remote - SSH (Microsoft) - for Raspberry Pi

3. **Open Project**:
   ```bash
   cd kinematics_project
   code .
   ```

### 3. Configure CMake in VS Code

Press `Ctrl+Shift+P` and select:
- "CMake: Configure" (or it may configure automatically)
- Select your compiler kit when prompted

### 4. Build Project

**Method 1**: Press `Ctrl+Shift+B`

**Method 2**: Use CMake Tools
- Press `Ctrl+Shift+P`
- Type "CMake: Build"
- Press Enter

**Method 3**: Terminal
```bash
cmake -B build -S .
cmake --build build
```

### 5. Run the Demo

**Method 1**: Press `F5` (builds and debugs)

**Method 2**: Terminal
```bash
./build/kinematics_demo
```

### 6. Run Tests

```bash
cd build
ctest --output-on-failure
```

## Raspberry Pi Deployment

### Method 1: Remote Development (Easiest)

1. **Setup SSH on Pi**:
   ```bash
   # On Raspberry Pi
   sudo raspi-config
   # Enable SSH in Interface Options
   ```

2. **Connect from VS Code**:
   - Press `Ctrl+Shift+P`
   - Type "Remote-SSH: Connect to Host"
   - Enter: `pi@<raspberry-pi-ip>`
   - Open the project folder on Pi
   - Build and run normally

### Method 2: Copy Executable

```bash
# Build for Raspberry Pi architecture
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Copy to Pi
scp build/kinematics_demo pi@<pi-ip>:~/

# Run on Pi
ssh pi@<pi-ip>
./kinematics_demo
```

## Troubleshooting

### CMake can't find compiler
```bash
# Verify compiler is installed
gcc --version
g++ --version

# If not installed (Linux):
sudo apt install build-essential
```

### Build fails with "cannot find -lstdc++"
```bash
# Linux:
sudo apt install libstdc++-dev

# Or specify C++ standard library
cmake -B build -DCMAKE_CXX_FLAGS="-stdlib=libc++"
```

### VS Code IntelliSense not working
1. Press `Ctrl+Shift+P`
2. Type "C/C++: Edit Configurations (UI)"
3. Set "Configuration provider" to "CMake Tools"

## Next Steps

1. Study `src/main.cpp` for usage examples
2. Review `include/kinematics.h` for API documentation
3. Modify architecture parameters in `RobotArchitecture`
4. Implement Class II and III FK (currently stubs)
5. Add your own test cases in `tests/`

## Performance Optimization for Embedded

### Compiler Flags
Edit CMakeLists.txt:
```cmake
# For speed
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

# For size (embedded)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")

# For Raspberry Pi specific
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a72 -mfloat-abi=hard")
```

### Static Linking (for standalone binary)
```cmake
set(CMAKE_EXE_LINKER_FLAGS "-static")
```

## Useful VS Code Commands

| Shortcut | Action |
|----------|--------|
| `Ctrl+Shift+B` | Build |
| `F5` | Debug |
| `Ctrl+F5` | Run without debugging |
| `F9` | Toggle breakpoint |
| `F10` | Step over |
| `F11` | Step into |
| `Shift+F11` | Step out |
| `Ctrl+Shift+P` | Command palette |
| `Ctrl+`` | Toggle terminal |

## Resources

- CMake Documentation: https://cmake.org/documentation/
- C++ Reference: https://en.cppreference.com/
- Raspberry Pi Documentation: https://www.raspberrypi.org/documentation/
