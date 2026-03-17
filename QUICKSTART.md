# Quick Start Guide — 3-RRR Spherical Parallel Mechanism

## Project Overview

This project implements the kinematics of the **Agile Eye-type 3-RRR Spherical Parallel Mechanism (SPM)**:

| Component | Description |
|-----------|-------------|
| `src/mSPMModel.cpp` | Core kinematics: IK, FK (Gauss-Newton dogleg), Velocity Jacobian |
| `include/mSPMModel.h` | `SPMModel` class and result structs (`IKResult`, `FKResult`, `VelJacobian`) |
| `include/mMatrix.h` / `mMath.h` / `mVect.h` | Lightweight linear algebra (no Eigen dependency) |
| `src/main.cpp` | Demo entry point |
| `tests/test_kinematics.cpp` | Kinematics verification tests |
| `notebooks/spm_kinematics.ipynb` | Symbolic derivation and visualization (Python) |

---

## C++ Build

### Prerequisites

#### Windows
```powershell
# Install MinGW-w64 or Visual Studio Build Tools
# Install CMake from https://cmake.org/download/
```

#### Linux (Ubuntu/Debian)
```bash
sudo apt update && sudo apt install build-essential cmake git
```

#### macOS
```bash
xcode-select --install
brew install cmake
```

### Build

```bash
cmake -B build -S .
cmake --build build
```

Or in VS Code: `Ctrl+Shift+P` → "CMake: Build" (or `Ctrl+Shift+B`).

### Run the Demo

```bash
./build/spm_demo
```

### Run Tests

```bash
# Run all tests (suppress output on pass)
cd build && ctest --output-on-failure

# Run with full printed output (timing, per-test results)
cd build && ctest -V

# Run only the kinematics test
cd build && ctest -V -R KinematicsTest

# Run the test executable directly (all printf/cout visible)
./build/tests/test_kinematics        # Linux / macOS
build\tests\test_kinematics.exe      # Windows
```

---

## Python Notebook

The `notebooks/` directory contains a Jupyter notebook for symbolic kinematics and visualization.

```bash
cd notebooks
pip install -r requirements.txt
jupyter notebook spm_kinematics.ipynb
```

---

## Using `SPMModel`

```cpp
#include "mSPMModel.h"

SPMModel spm;                   // default Coaxial Input SPM architecture

// Inverse Kinematics
Matrix33f R = /* rotation matrix */;
IKResult ik = spm.computeIK(R, /*update_config=*/true);

// Forward Kinematics (Gauss-Newton dogleg solver)
float theta[3] = { ik.theta[0][0], ik.theta[1][0], ik.theta[2][0] };
FKResult fk = spm.computeFK(theta, /*update_config=*/true, /*init_prev_sol=*/false);
// fk.R  — recovered rotation matrix (SO(3) via polar decomposition)

// Velocity Jacobian  (omega = J * theta_dot)
VelJacobian jac = spm.computeVelJacobian();
```

Custom architecture — populate `SPMArch` and pass to the constructor:

```cpp
SPMArch arch;
arch.gamma_i1[0] = /* ... */;
// ...
SPMModel spm(arch);
```

---

## VS Code Setup

1. **Install extensions** (`Ctrl+Shift+X`): C/C++ (Microsoft), CMake Tools (Microsoft)
2. **Open project**: `code .` from the repo root
3. **Configure**: `Ctrl+Shift+P` → "CMake: Configure", select your compiler kit
4. **Build**: `Ctrl+Shift+B`
5. **Debug**: `F5`

| Shortcut | Action |
|----------|--------|
| `Ctrl+Shift+B` | Build |
| `F5` | Debug |
| `Ctrl+F5` | Run without debugging |
| `F9` | Toggle breakpoint |
| `F10` / `F11` | Step over / Step into |
| `Ctrl+Shift+P` | Command palette |

---

## Troubleshooting

**CMake can't find compiler**
```bash
gcc --version   # verify installation
sudo apt install build-essential   # Linux fix
```

**VS Code IntelliSense not working**
`Ctrl+Shift+P` → "C/C++: Edit Configurations (UI)" → set "Configuration provider" to "CMake Tools"

---

## Next Steps

1. Review `include/mSPMModel.h` for full API documentation and constraint equations
2. Study `tests/test_kinematics.cpp` for IK↔FK round-trip verification examples
3. Customize architecture parameters via `SPMArch` (angles in radians)
4. Open `notebooks/spm_kinematics.ipynb` for symbolic derivations
