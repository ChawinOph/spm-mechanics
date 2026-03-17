# SPM Kinematics — Technical Reference

**Mechanism:** Agile Eye-type 3-RRR Spherical Parallel Mechanism (SPM)
**Implementation:** `src/mSPMModel.cpp` · `include/mSPMModel.h`

---

## Table of Contents

1. [Mechanism Overview](#1-mechanism-overview)
2. [Architecture Parameters](#2-architecture-parameters)
3. [Joint Axis Vectors](#3-joint-axis-vectors)
4. [Proximal Link Parameterisation](#4-proximal-link-parameterisation)
5. [Inverse Kinematics (IK)](#5-inverse-kinematics-ik)
6. [Velocity Jacobian](#6-velocity-jacobian)
7. [Forward Kinematics (FK)](#7-forward-kinematics-fk)
8. [Rotation Recovery — SVD Polar Decomposition](#8-rotation-recovery--svd-polar-decomposition)
9. [Math Utilities](#9-math-utilities)
10. [Default Architecture Values](#10-default-architecture-values)

---

## 1. Mechanism Overview

The 3-RRR SPM has three identical legs, each with three revolute joints whose axes intersect at a common point (the centre of the sphere). The platform orientation is fully described by a rotation matrix **R** ∈ SO(3).

Each leg `i` has three joint axes:

| Symbol | Description | Frame |
|--------|-------------|-------|
| $u_i$ | First joint axis (base) | World frame, fixed |
| $v_i(θ_i)$ | Second joint axis (proximal link tip) | World frame, depends on θ_i |
| $w_i$ | Third joint axis (moving platform side) | World frame, `= R · w_i0` |

$w_i0$ is the third joint axis expressed in the **moving platform body frame** (constant).

---

## 2. Architecture Parameters

Stored in `SPMArch`:

| Field | Symbol | Description |
|-------|--------|-------------|
| `eta_i1[i]` | η_{i1} | Azimuthal angle of the i-th base joint axis |
| `eta_i2[i]` | η_{i2} | Azimuthal angle of the i-th platform joint axis |
| `gamma_i1[i]` | γ_{i1} | Tilt angle of the i-th base joint axis |
| `gamma_i2[i]` | γ_{i2} | Tilt angle of the i-th platform joint axis |
| `alpha_i1[i]` | α_{i1} | Half-angle of the proximal link (base to v_i) |
| `alpha_i2[i]` | α_{i2} | Half-angle of the distal link (v_i to w_i) |

---

## 3. Joint Axis Vectors

All joint axes lie on the unit sphere and are computed by rotating the z-axis:

```
Rz(η) · Rx(γ) · [0, 0, 1]ᵀ  =  [ sin η · sin γ,  −cos η · sin γ,  cos γ ]ᵀ
```

Applied to each leg:

```
u_i   = Rz(η_{i1}) · Rx(γ_{i1}) · ẑ        (base,     fixed in world)
w_i0  = Rz(η_{i2}) · Rx(γ_{i2}) · ẑ        (platform, fixed in body frame)
w_i   = R · w_i0                            (platform, in world frame)
```

Implementation: `SPMModel::computeJointVector(eta, gamma)` → `mSPMModel.cpp:448`

---

## 4. Proximal Link Parameterisation

The second joint axis **v**_i traces a circular arc on the sphere as θ_i varies:

```
v_i(θ_i) = [ a1·cos θ + b1·sin θ + c1 ]
            [ a2·cos θ + b2·sin θ + c2 ]
            [ a3·cos θ           + c3  ]
```

Coefficients are precomputed at construction from (η_{i1}, γ_{i1}, α_{i1}):

```
a1 =  sin η_{i1} · cos γ_{i1} · sin α_{i1}
b1 =  cos η_{i1} · sin α_{i1}
c1 =  sin η_{i1} · sin γ_{i1} · cos α_{i1}

a2 = −cos η_{i1} · cos γ_{i1} · sin α_{i1}
b2 =  sin η_{i1} · sin α_{i1}
c2 = −cos η_{i1} · sin γ_{i1} · cos α_{i1}

a3 = −sin γ_{i1} · sin α_{i1}
c3 =  cos γ_{i1} · cos α_{i1}
```

The arc parameterisation satisfies the proximal link constraint:

```
u_i · v_i(θ_i) = cos α_{i1}   for all θ_i
```

Implementation: `SPMModel::computeVVector(...)` → `mSPMModel.cpp:456`

---

## 5. Inverse Kinematics (IK)

**Input:** Rotation matrix **R** ∈ SO(3)
**Output:** Joint angles θ_i for each leg (up to 2 solutions per leg)

### Derivation

The distal link constraint requires:

```
v_i(θ_i) · w_i = cos α_{i2}
```

Substituting `w_i = R · w_i0` and expanding `v_i(θ_i)` gives a linear
combination in `cos θ_i` and `sin θ_i`:

```
A_i · cos θ_i  +  B_i · sin θ_i  =  C_i
```

where:

```
A_i = wx·a1 + wy·a2 + wz·a3
B_i = wx·b1 + wy·b2             (b3 = 0 by construction)
C_i = cos α_{i2} − (wx·c1 + wy·c2 + wz·c3)

[wx, wy, wz]ᵀ = R · w_i0
```

### Solution

Solved via the **Weierstrass half-angle substitution** `t = tan(θ/2)`:

```
cos θ = (1 − t²) / (1 + t²)
sin θ = 2t / (1 + t²)
```

Substituting and multiplying through by `(1 + t²)` reduces the equation to a
quadratic in `t`:

```
(A_i + C_i) t²  −  2 B_i t  +  (C_i − A_i)  =  0
```

Solved by `solveQuadratic()`. A solution exists only when `A_i² + B_i² ≥ C_i²`.

Each leg yields **0, 1, or 2** solutions. The active branch per leg is tracked
by `theta_sol_indices[i]` (default `0`).

**Result structure:** `IKResult` — `theta[i][0..1]`, `has_solution[i]`, `num_solutions[i]`

Implementation: `SPMModel::computeIK(R, update_config)` → `mSPMModel.cpp:560`

---

## 6. Velocity Jacobian

**Input:** Current configuration (requires prior `computeIK(..., update_config=true)`)
**Output:** Forward Jacobian **J** such that `ω = J · θ̇`

### Derivation

Differentiating the constraint `v_i · w_i = cos α_{i2}` with respect to time:

```
v̇_i · w_i  +  v_i · ẇ_i  =  0
```

Using `ẇ_i = ω × w_i` (rigid body rotation):

```
v̇_i · w_i  +  (w_i × v_i) · ω  =  0
```

The first term relates to the joint velocity: `v̇_i · w_i = (u_i · (w_i × v_i)) · θ̇_i`

Stacking all three legs:

```
A · ω = B · θ̇
```

where:

```
A[i, :]  = w_i × v_i                    (row i of A_mat)
B[i, i]  = u_i · (w_i × v_i)            (diagonal of B_mat)
```

The forward Jacobian is:

```
J = A⁻¹ · B          (ω = J · θ̇)
```

`A` is inverted analytically via `Matrix33f::Invert`. If `|det A| < 1e-6` the
configuration is singular and `is_valid = false`.

**Result structure:** `VelJacobian` — `J`, `A_mat`, `B_mat`, `is_valid`

Implementation: `SPMModel::computeVelJacobian()` → `mSPMModel.cpp:615`

---

## 7. Forward Kinematics (FK)

**Input:** Joint angles `θ[0..2]`
**Output:** Third joint axis vectors **w**_i and rotation matrix **R**

### Constraint System

The 9 scalar constraints on `x = [w₀ | w₁ | w₂]` (9 unknowns):

| Index | Equation | Type |
|-------|----------|------|
| F₀ | `v₀(θ₀) · w₀ = cos α_{02}` | Distal alignment, leg 0 |
| F₁ | `v₁(θ₁) · w₁ = cos α_{12}` | Distal alignment, leg 1 |
| F₂ | `v₂(θ₂) · w₂ = cos α_{22}` | Distal alignment, leg 2 |
| F₃ | `w₁ · w₂ = w_{10} · w_{20}` | Platform inter-axis angle (1-2) |
| F₄ | `w₂ · w₀ = w_{20} · w_{00}` | Platform inter-axis angle (2-0) |
| F₅ | `w₀ · w₁ = w_{00} · w_{10}` | Platform inter-axis angle (0-1) |
| F₆ | `‖w₀‖ = 1` | Unit norm |
| F₇ | `‖w₁‖ = 1` | Unit norm |
| F₈ | `‖w₂‖ = 1` | Unit norm |

The inter-axis dot products `w_{i0} · w_{j0}` are constant (rigid platform),
so they are precomputed and stored in `c3[0..2]`.

### Analytical Jacobian of F

The 9×9 Jacobian `J_F = ∂F/∂x` has a sparse, structured form:

```
Rows 0–2  (alignment):   J_F[i, 3i : 3i+3] = v_i         (block diagonal)
Row  3    (angle 1-2):   J_F[3, 3:6] = w₂;  J_F[3, 6:9] = w₁
Row  4    (angle 2-0):   J_F[4, 6:9] = w₀;  J_F[4, 0:3] = w₂
Row  5    (angle 0-1):   J_F[5, 0:3] = w₁;  J_F[5, 3:6] = w₀
Rows 6–8  (unit norm):   J_F[6+i, 3i : 3i+3] = 2 w_i
```

### Gauss-Newton Dogleg Solver

The system `F(x) = 0` is solved by minimising `f(x) = ½ ‖F(x)‖²` using a
**trust-region dogleg** method.

**Normal equations** at each iteration:

```
B_GN = J_Fᵀ J_F    (Gauss-Newton Hessian approximation)
g    = J_Fᵀ F       (gradient of f)
```

`B_GN` is symmetric positive semi-definite and solved via **Cholesky
factorisation** (`B_GN = L Lᵀ`, forward/back substitution).

**Dogleg step** `p`:
1. **Newton step** `p_N`: solve `B_GN · p_N = −g`
2. If `‖p_N‖ ≤ Δ` (inside trust region) → `p = p_N`
3. **Cauchy step** `p_U = −(‖g‖² / (gᵀ B_GN g)) · g`
4. If `‖p_U‖ ≥ Δ` → `p = Δ · p_U / ‖p_U‖`
5. Otherwise → interpolate `p_U → p_N` to hit the trust-region boundary Δ

**Trust-region update** (ρ = actual/predicted reduction):

| ρ | Action |
|---|--------|
| < 0.25 | `Δ ← 0.25 Δ` (shrink) |
| > 0.75 and step on boundary | `Δ ← min(2Δ, Δ_max)` (expand) |
| > η (= 0.1) | Accept step |

**Convergence criteria** (any triggers success):

```
‖g‖ < 1e-6    (gradient tolerance)
‖p‖ < 1e-7    (step tolerance)
|Δf| < 1e-10  (function value tolerance)
```

Maximum iterations: 100. Declared converged when `f(x*) < 1e-6`.

**Initial guess options:**

| `init_prev_sol` | Starting point |
|-----------------|----------------|
| `false` (cold) | `w_i0` — body-frame vectors (neutral pose) |
| `true` (warm) | `w_i` — last accepted configuration |

Implementation: `fk_solve()` / `fk_residual_and_jac()` → `mSPMModel.cpp:59–255`

---

## 8. Rotation Recovery — SVD Polar Decomposition

After the FK solver converges, **R** is recovered from the solved `w_i` vectors.

### Construction of M

```
M = Σᵢ  w_i · w_{i0}ᵀ   =  W_sol · W₀ᵀ
```

where `W_sol = [w₀ | w₁ | w₂]` and `W₀ = [w_{00} | w_{10} | w_{20}]`
(3×3 matrices with column vectors).

When the FK has converged, `w_i = R · w_{i0}`, so:

```
M = R · W₀ · W₀ᵀ
```

The polar factor of `M` equals `R` since `W₀ · W₀ᵀ` is symmetric PSD.

### Why SVD, not Newton Iteration

The default SPM has `γ_{i2} ≈ 85°`, so all `w_{i0}` vectors have very small
z-components (`≈ cos 85° ≈ 0.087`). This makes `W₀ · W₀ᵀ` ill-conditioned:

```
σ_min(W₀ · W₀ᵀ) ≈ 0.023,   σ_max ≈ 1.49,   κ ≈ 65
```

The Newton polar iteration maps each singular value `σ → (σ + 1/σ) / 2`.
Starting from `σ = 0.023` takes ~8 steps to reach `σ = 1`. With only 5
iterations the result was `m₃₃ ≈ 1.6` instead of `1.0`.

SVD computes the polar factor exactly in one pass, independent of conditioning.

### Algorithm: Jacobi SVD

**Step 1** — Form `B = MᵀM` (symmetric PSD, 3×3)

**Step 2** — Jacobi diagonalisation of B (up to 30 sweeps):

Each sweep finds the largest off-diagonal entry `|B[p][q]|` and zeros it
with a Givens rotation `J(p, q, θ)`:

```
τ = (B[q][q] − B[p][p]) / (2 B[p][q])
t = sign(τ) / (|τ| + √(1 + τ²))        (tan θ, numerically stable form)
c = 1 / √(1 + t²),   s = t · c          (cos θ, sin θ)
```

Symmetric update `B ← JᵀBJ` (only 5 entries change):

```
B[p][p] ← B[p][p] − t · B[p][q]
B[q][q] ← B[q][q] + t · B[p][q]
B[p][q] = B[q][p] ← 0
B[r][p] ← c · B[r][p] − s · B[r][q]    (r ≠ p, q)
B[r][q] ← s · B[r][p] + c · B[r][q]
```

Eigenvector accumulation `V ← V · J`:

```
V[r][p] ←  c · V[r][p] − s · V[r][q]
V[r][q] ←  s · V[r][p] + c · V[r][q]
```

Convergence: `max|B[p][q]| < 1e-12`.

**Step 3** — Extract singular values and right singular vectors:

```
σᵢ = √|B[i][i]|,    V[:,i] = V_accumulated[:,i]
```

**Step 4** — Left singular vectors via column normalisation:

```
col_j = M · v_j             (matrix-vector product)
u_j   = col_j / ‖col_j‖    (‖col_j‖ = σ_j for unit v_j)
```

### Polar Factor and det-Correction

```
R = U · Vᵀ
```

If `det(R) = −1` (reflection), flip the U column for the smallest `σ`:

```
U[:, argmin σ] ← −U[:, argmin σ]
R ← U · Vᵀ                              → det(R) = +1
```

This is the **Kabsch correction**, giving the nearest proper rotation in SO(3).

Implementation: `svd3x3()` / `det3()` / `polar3()` → `mSPMModel.cpp:293–415`

---

## 9. Math Utilities

### Quadratic Solver (`mMath.h`)

Solves `a x² + b x + c = 0` for real roots.

```
discriminant = b² − 4ac

disc < 0  →  no real roots  (x1 = x2 = NaN)
disc = 0  →  one root:   x = −b / (2a)
disc > 0  →  two roots:  x = (−b ± √disc) / (2a)
```

Template: `QuadraticResult<T> solveQuadratic(T a, T b, T c)`

### Trigonometric Equation Solver (`mMath.h`)

Solves `A cos θ + B sin θ = C` for θ ∈ (−π, π].

**Substitution:** `t = tan(θ/2)` transforms the equation to a quadratic:

```
cos θ = (1 − t²)/(1 + t²),   sin θ = 2t/(1 + t²)

(A + C) t²  −  2B t  +  (C − A)  =  0
```

**Special case:** `A + C ≈ 0` (leading coefficient zero) → linear equation:

```
t = (C − A) / (2B)
```

Solutions exist only when `A² + B² ≥ C²`. Each `t` is converted back via
`θ = atan2(sin θ, cos θ)` with a numerical stability check on `sin²θ + cos²θ ≈ 1`.
Duplicate solutions are deduplicated.

Returns 0, 1, or 2 solutions in `TrigResult<T>`.

Template: `TrigResult<T> solveTrigEquation(T A, T B, T C)`

---

## 10. Default Architecture Values

Coaxial Input SPM (3-fold symmetry, legs at 120° apart):

| Parameter | Leg 0 | Leg 1 | Leg 2 | Value |
|-----------|-------|-------|-------|-------|
| η_{i1} (base azimuth) | 0 | 2π/3 | 4π/3 | — |
| η_{i2} (platform azimuth) | 0 | 2π/3 | 4π/3 | — |
| γ_{i1} (base tilt) | 0 | 0 | 0 | 0 rad |
| γ_{i2} (platform tilt) | π/2 − 5° | π/2 − 5° | π/2 − 5° | ≈ 1.484 rad |
| α_{i1} (proximal half-angle) | π/3 | π/3 | π/3 | 60° |
| α_{i2} (distal half-angle) | π/4 | π/4 | π/4 | 45° |

The 5° offset in γ_{i2} from π/2 avoids the gimbal singularity at the neutral
pose (R = I).

---

## Data Flow Summary

```
Architecture (SPMArch)
        │
        ▼
   Constructor
   ├── precompute u_i, w_i0           (joint axis vectors)
   ├── precompute a1..c3 coefficients  (proximal link)
   └── computeIK(I, update=true)      (neutral pose thetas)

         ┌──────────────┐
         │   IK Path    │
         └──────────────┘
R  ──►  computeIK(R)
        │  w_i = R · w_i0
        │  A·cosθ + B·sinθ = C   (per leg)
        │  Weierstrass → quadratic → solveQuadratic()
        └──►  θ_i[0..1],  IKResult

θ_i  ──►  computeVelJacobian()
          │  A_mat[i,:] = w_i × v_i
          │  B_mat[i,i] = u_i · (w_i × v_i)
          └──►  J = A⁻¹ B,  VelJacobian

         ┌──────────────┐
         │   FK Path    │
         └──────────────┘
θ_i  ──►  computeFK(θ)
          │  build FKProblem (v_i, c2, c3)
          │  fk_solve() — Gauss-Newton dogleg
          │    ├── fk_residual_and_jac()  → F[9], J_F[9×9]
          │    ├── fk_normal_eqs()        → g, B_GN
          │    ├── chol9() + fwd9/bwd9   → Newton step
          │    └── dogleg_step()          → trust-region step p
          │  polar3(M)
          │    ├── svd3x3()   → U, σ, V
          │    └── R = U Vᵀ  (with det-correction)
          └──►  w_i[3],  R,  FKResult
```
