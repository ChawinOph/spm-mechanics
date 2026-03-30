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

# Run tests — suppress output on pass
cd build && ctest --output-on-failure

# Run tests — full printed output (timing + per-test results)
cd build && ctest -V

# Run test executable directly (all stdout visible)
./build/tests/test_kinematics        # Linux / macOS
build\tests\test_kinematics.exe      # Windows
```

The build produces:
- `libspm.a` — static library (`SPMModel` + FK solver)
- `spm_demo` — demo executable
- `tests/test_kinematics` — kinematics test executable

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

## Link Surpass Detection (Python)

`link_surpass_occur` checks whether any coaxial input link has "surpassed" (overtaken) another on the shared axis, and optionally enforces a minimum angular separation (margin). It operates on the three adjusted input angles in `[0, 2π)`.

### Algorithm

The modulo operation used throughout is defined as:

$$a \bmod m = a - m\left\lfloor\frac{a}{m}\right\rfloor$$

which always returns a result in $[0, m)$ for positive $m$, correctly handling both negative angles and angles beyond $2\pi$.

1. **Wrap** each raw joint angle: `θ_adj[i] = (θ[i] + η[i]) % 2π`
2. **Compute CCW arcs** between consecutive links using `(θ2 - θ1) % 2π`
3. **Surpass test**: if the three CCW arcs sum to `2π`, the links are in valid CCW cyclic order (1→2→3). Any other sum means at least one link has surpassed another.
4. **Margin test**: shortest arc between each pair = `min(arc, 2π − arc)`. If any shortest arc is less than `margin`, a near-collision is flagged.

```python
def _ccw(self, t1, t2):
    """CCW arc from t1 to t2, result in [0, 2π)."""
    return (t2 - t1) % (2 * np.pi)

def link_surpass_occur(self, test_thetas, margin=0):
    if any(gamma != 0 for gamma in self.gamma_i1):
        return False  # only valid for coaxial case

    a = [self.wrap_angle(test_thetas[i] + self.eta_i1[i]) for i in range(3)]

    ccw12 = self._ccw(a[0], a[1])
    ccw23 = self._ccw(a[1], a[2])
    ccw31 = self._ccw(a[2], a[0])

    # surpass: links not in CCW cyclic order 1→2→3
    if not np.isclose(ccw12 + ccw23 + ccw31, 2 * np.pi):
        return True

    # margin: shortest arc between each pair
    min_arcs = [min(d, 2 * np.pi - d) for d in (ccw12, ccw23, ccw31)]
    if any(arc < abs(margin) for arc in min_arcs):
        return True

    return False
```

### Known Limitations

| Case | Behaviour |
|---|---|
| Two links at exactly the same angle | Reported as surpass (CCW arc = 0, sum ≠ 2π) |
| **Double surpass** (link 1 passes 2 **and** 2 passes 3 simultaneously) | **Not detected** — cyclic order can appear valid again (sum returns to 2π) |
| CW initial configuration (1→3→2) | Reports surpass immediately; no `expected_order` parameter |

For full correctness in the double-surpass case, pair-wise order tracking across timesteps (storing previous order state) is required.

### Test Cases

```python
DEG = np.deg2rad

# 1. No surpass — evenly spaced CCW
assert not link_surpass_occur([DEG(0),   DEG(120), DEG(240)])

# 2. Surpass — link 2 has passed link 3
assert     link_surpass_occur([DEG(0),   DEG(240), DEG(120)])

# 3. Degenerate — two links at same angle
assert     link_surpass_occur([DEG(0),   DEG(0),   DEG(180)])

# 4. Margin — links 1&2 only 5° apart, margin=10°
assert     link_surpass_occur([DEG(0),   DEG(5),   DEG(180)], margin=DEG(10))

# 5. Margin boundary — links 10° apart, margin=10° (not strictly less than)
assert not link_surpass_occur([DEG(0),   DEG(10),  DEG(180)], margin=DEG(10))

# 6. Wrap — angles outside [0, 2π] treated same as wrapped equivalent
assert not link_surpass_occur([DEG(360), DEG(480), DEG(600)])  # same as 0,120,240

# 7. Double surpass — KNOWN UNDETECTED CASE
# Originally 0,120,240; each link passes the next → 150,90,30
# assert link_surpass_occur([DEG(150), DEG(90), DEG(30)])  # fails: sum=2π but all surpassed
```

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