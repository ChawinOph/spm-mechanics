# 3-RRR Spherical Parallel Mechanism

**Author:** Chawin Ophaswongse

**Email:** oph.chawin@gmail.com

**Date Created:** January 22, 2026

**Last Modified:** March 17, 2026

**Repository:** https://github.com/ChawinOph/spm-mechanics

---

C++ implementation of forward and inverse kinematics for a Coaxial Input 3-RRR spherical parallel mechanism (Agile Eye-type), with a Python/Jupyter companion for symbolic derivation and numerical verification.

## Features

- **Inverse Kinematics** — closed-form per-leg solution via trigonometric equation solver (Weierstrass half-angle substitution); up to 2 solutions per leg
- **Forward Kinematics** — iterative trust-region dogleg solver minimising a 9-equation constraint system; recovers rotation matrix via polar decomposition
- **Velocity Jacobian** — analytical $J = A^{-1}B$ from the differentiated constraint equations
- **Embedded-friendly** — no STL containers in the solver, fixed-size float arrays, Arduino-compatible `mMath.h`

## Project Structure

```
spm_project/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── mMath.h          # Quadratic & trigonometric equation solvers
│   ├── mVect.h          # mVector2/3/4 templated vector types
│   ├── mMatrix.h        # Matrix22/33/44 templated matrix types
│   └── mSPMModel.h      # SPMModel class, SPMArch, IKResult, FKResult, VelJacobian
├── src/
│   ├── mSPMModel.cpp    # SPMModel implementation + internal FK dogleg solver
│   └── main.cpp         # Demo entry point
├── tests/
│   └── test_kinematics.cpp   # (legacy – pending update)
└── notebooks/
    ├── requirements.txt
    └── spm_kinematics.ipynb  # Symbolic derivation, IK/FK verification, visualisation
```

## Key API

```cpp
#include "mSPMModel.h"

SPMModel spm;                          // default Coaxial Input SPM geometry

// Inverse kinematics
Matrix33f R_des = ...;
IKResult ik = spm.computeIK(R_des, /*update_config=*/true);

// Select solution branch per leg (0 or 1)
spm.theta_sol_indices[0] = 0;

// Velocity Jacobian  (requires update_config IK first)
VelJacobian jac = spm.computeVelJacobian();

// Forward kinematics
FKResult fk = spm.computeFK(spm.theta_i,
                             /*update_config=*/true,
                             /*init_prev_sol=*/true);
// fk.R        — recovered rotation matrix R ∈ SO(3)
// fk.w[i]     — solved w_i vectors in world frame
// fk.success  — true if solver converged
// fk.cost     — final cost ½‖F(x*)‖² (should be ~0)
```

## Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.10 or higher

## Building

```bash
# Configure
cmake -B build -S .

# Build
cmake --build build

# Run demo
./build/spm_demo          # Linux / macOS
build\spm_demo.exe        # Windows
```

The build produces:
- `libspm.a` — static library (`SPMModel` + FK solver)
- `spm_demo` — demo executable

## Cross-Compilation for Raspberry Pi

### Option 1: Native (simplest)

SSH into the Pi and build directly:
```bash
sudo apt install cmake g++
git clone <repo>
cd spm_project
cmake -B build -S .
cmake --build build
```

### Option 2: Cross-compile toolchain

Create `cmake/raspberry-pi.cmake`:
```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER   arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```
Then:
```bash
cmake -B build-pi -S . -DCMAKE_TOOLCHAIN_FILE=cmake/raspberry-pi.cmake
cmake --build build-pi
scp build-pi/spm_demo pi@<pi-ip>:~/
```

## Python / Jupyter Notebook

The `notebooks/spm_kinematics.ipynb` notebook contains:

- Symbolic derivation of joint axes, proximal link parameterisation, and constraint equations
- IK and FK solvers (scipy dogleg) with numerical verification
- Velocity Jacobian derivation
- 3-D visualisation of SPM configurations

### Setup

```bash
cd notebooks
pip install -r requirements.txt
jupyter notebook spm_kinematics.ipynb
```

| Package | Purpose |
|---|---|
| `numpy` | Numerical computation |
| `scipy` | FK dogleg solver (`scipy.optimize.minimize`) |
| `sympy` | Symbolic derivation |
| `matplotlib` | 3-D visualisation |

## VS Code Shortcuts

| Shortcut | Action |
|---|---|
| `Ctrl+Shift+B` | Build project |
| `F5` | Build and debug |
| `Ctrl+Shift+P` | Command palette |
| `F9` | Toggle breakpoint |
| `F10` | Step over |
| `F11` | Step into |

## References

```
Tursynbek, Iliyas, and Almas Shintemirov. "Infinite Rotational Motion Generation and
Analysis of a Spherical Parallel Manipulator with Coaxial Input Axes."
Mechatronics 78 (October 2021): 102625.
https://doi.org/10.1016/j.mechatronics.2021.102625

S. Bai, "Optimum design of spherical parallel manipulators for a prescribed workspace,"
Mechanism and Machine Theory, vol. 45, no. 2, pp. 200–211, Feb. 2010.
https://doi.org/10.1016/j.mechmachtheory.2009.06.007

Li, Wei, Shuai Zhang, Binyan Wang, Jorge Angeles, Feng Gao, and Weizhong Guo. “Forward Kinematics of Three Classes of 3-RRR Spherical Parallel Mechanisms Admitting Closed-Form Solutions.” 
Mechanism and Machine Theory 201 (October 2024): 105751. https://doi.org/10.1016/j.mechmachtheory.2024.105751.
```

## License

Licensed under the [Apache License 2.0](LICENSE).
Copyright (c) 2026 Chawin Ophaswongse

## Contact

Chawin Ophaswongse — oph.chawin@gmail.com