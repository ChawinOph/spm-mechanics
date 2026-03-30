/**
 * @file mSPMModel.cpp
 * @brief 3-RRR SPM kinematics – implementation
 */

#include "mSPMModel.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

// ---------------------------------------------------------------------------
// Internal FK dogleg solver
// All symbols are file-local (anonymous namespace).
// ---------------------------------------------------------------------------
namespace {

// ── small fixed-size vector / matrix helpers ─────────────────────────────────

static inline void v9_copy(float d[9], const float s[9]) {
    for (int i = 0; i < 9; i++) d[i] = s[i];
}
static inline void v9_zero(float x[9]) {
    for (int i = 0; i < 9; i++) x[i] = 0.f;
}
static inline void v9_add(float o[9], const float a[9], const float b[9]) {
    for (int i = 0; i < 9; i++) o[i] = a[i] + b[i];
}
static inline void v9_sub(float o[9], const float a[9], const float b[9]) {
    for (int i = 0; i < 9; i++) o[i] = a[i] - b[i];
}
static inline void v9_scale(float o[9], const float a[9], float s) {
    for (int i = 0; i < 9; i++) o[i] = s * a[i];
}

static inline float v9_dot(const float a[9], const float b[9]) {
    float s = 0.f;
    for (int i = 0; i < 9; i++) s += a[i] * b[i];
    return s;
}
static inline float v9_normsq(const float a[9]) { return v9_dot(a, a); }
static inline float v9_norm(const float a[9]) { return sqrtf(v9_normsq(a)); }

static inline void m99_zero(float A[9][9]) {
    for (int r = 0; r < 9; r++)
        for (int c = 0; c < 9; c++) A[r][c] = 0.f;
}
static inline void m99_mul_v9(float y[9], const float A[9][9], const float x[9]) {
    for (int r = 0; r < 9; r++) {
        float s = 0.f;
        for (int c = 0; c < 9; c++) s += A[r][c] * x[c];
        y[r] = s;
    }
}

// ── residual + Jacobian ───────────────────────────────────────────────────────
//
// x = [w0x w0y w0z | w1x w1y w1z | w2x w2y w2z]
//
// F0 = v0·w0 - c2[0]     (alignment leg 0)
// F1 = v1·w1 - c2[1]     (alignment leg 1)
// F2 = v2·w2 - c2[2]     (alignment leg 2)
// F3 = w1·w2 - c3[0]     (platform angle 1-2)
// F4 = w2·w0 - c3[1]     (platform angle 2-0)
// F5 = w0·w1 - c3[2]     (platform angle 0-1)
// F6 = w0·w0 - 1         (unit norm)
// F7 = w1·w1 - 1
// F8 = w2·w2 - 1
//
// Jacobian rows follow the same order (see mSPMModel.h derivation).

struct FKProblem {
    float v[3][3];  ///< proximal link tip vectors v_i[x,y,z]
    float c2[3];    ///< cos(alpha_i2) per leg
    float c3[3];    ///< mutual dot targets: [w1·w2, w2·w0, w0·w1]
};

static void fk_residual_and_jac(const FKProblem& P, const float x[9], float F[9], float J[9][9]) {
    // extract w vectors
    const float *w0 = x + 0, *w1 = x + 3, *w2 = x + 6;

    // dot helpers
    auto dot3 = [](const float* a, const float* b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };

    // residuals
    F[0] = dot3(P.v[0], w0) - P.c2[0];
    F[1] = dot3(P.v[1], w1) - P.c2[1];
    F[2] = dot3(P.v[2], w2) - P.c2[2];
    F[3] = dot3(w1, w2) - P.c3[0];
    F[4] = dot3(w2, w0) - P.c3[1];
    F[5] = dot3(w0, w1) - P.c3[2];
    F[6] = dot3(w0, w0) - 1.f;
    F[7] = dot3(w1, w1) - 1.f;
    F[8] = dot3(w2, w2) - 1.f;

    m99_zero(J);

    // rows 0-2: ∂(v_i·w_i)/∂w_i = v_i  (linear, only diagonal block nonzero)
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++) J[i][3 * i + k] = P.v[i][k];

    // row 3: F3 = w1·w2  →  ∂/∂w1 = w2,  ∂/∂w2 = w1
    for (int k = 0; k < 3; k++) {
        J[3][3 + k] = w2[k];
        J[3][6 + k] = w1[k];
    }

    // row 4: F4 = w2·w0  →  ∂/∂w2 = w0,  ∂/∂w0 = w2
    for (int k = 0; k < 3; k++) {
        J[4][6 + k] = w0[k];
        J[4][0 + k] = w2[k];
    }

    // row 5: F5 = w0·w1  →  ∂/∂w0 = w1,  ∂/∂w1 = w0
    for (int k = 0; k < 3; k++) {
        J[5][0 + k] = w1[k];
        J[5][3 + k] = w0[k];
    }

    // rows 6-8: ∂(w_i·w_i)/∂w_i = 2·w_i  (power rule)
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++) J[6 + i][3 * i + k] = 2.f * x[3 * i + k];
}

// ── normal equations: g = J^T F,  B = J^T J ──────────────────────────────────

static void fk_normal_eqs(const float F[9], const float J[9][9], float g[9], float B[9][9]) {
    v9_zero(g);
    m99_zero(B);
    for (int r = 0; r < 9; r++)
        for (int c = 0; c < 9; c++) g[c] += J[r][c] * F[r];  // g = J^T F

    for (int i = 0; i < 9; i++)
        for (int j = i; j < 9; j++) {
            float s = 0.f;
            for (int r = 0; r < 9; r++) s += J[r][i] * J[r][j];
            B[i][j] = B[j][i] = s;  // B = J^T J  (symmetric)
        }
}

// ── Cholesky LL^T solve for 9×9 SPD system ───────────────────────────────────

static bool chol9(const float A[9][9], float L[9][9]) {
    m99_zero(L);
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j <= i; j++) {
            float s = A[i][j];
            for (int k = 0; k < j; k++) s -= L[i][k] * L[j][k];
            if (i == j) {
                if (s <= 1e-12f) return false;
                L[i][j] = sqrtf(s);
            } else
                L[i][j] = s / L[j][j];
        }
    }
    return true;
}

static void fwd9(const float L[9][9], const float b[9], float y[9]) {
    for (int i = 0; i < 9; i++) {
        float s = b[i];
        for (int j = 0; j < i; j++) s -= L[i][j] * y[j];
        y[i] = s / L[i][i];
    }
}

static void bwd9(const float L[9][9], const float y[9], float x[9]) {
    for (int i = 8; i >= 0; i--) {
        float s = y[i];
        for (int j = i + 1; j < 9; j++) s -= L[j][i] * x[j];
        x[i] = s / L[i][i];
    }
}

static bool solve_spd9(const float A[9][9], const float b[9], float x[9]) {
    float L[9][9], y[9];
    if (!chol9(A, L)) return false;
    fwd9(L, b, y);
    bwd9(L, y, x);
    return true;
}

// ── dogleg step ───────────────────────────────────────────────────────────────

static float pred_reduction(const float g[9], const float B[9][9], const float p[9]) {
    float Bp[9];
    m99_mul_v9(Bp, B, p);
    return -(v9_dot(g, p) + 0.5f * v9_dot(p, Bp));
}

static bool dogleg_step(const float g[9], const float B[9][9], float delta, float p[9]) {
    // Newton step pB: B*pB = -g
    float pB[9], rhs[9];
    for (int i = 0; i < 9; i++) rhs[i] = -g[i];
    if (!solve_spd9(B, rhs, pB)) return false;

    if (v9_norm(pB) <= delta) {
        v9_copy(p, pB);
        return true;
    }  // inside trust region

    // Cauchy step pU: optimal step along -g
    float Bg[9];
    m99_mul_v9(Bg, B, g);
    const float gg = v9_dot(g, g), gBg = v9_dot(g, Bg);
    float pU[9];
    if (gBg <= 1e-20f) {
        v9_scale(pU, g, -1.f);
    } else {
        v9_scale(pU, g, -gg / gBg);
    }

    const float nU = v9_norm(pU);
    if (nU >= delta) {  // Cauchy step outside
        v9_scale(p, pU, delta / nU);
        return true;
    }

    // Interpolate pU → pB to hit trust-region boundary
    float d[9];
    v9_sub(d, pB, pU);
    const float a = v9_normsq(d), b = 2.f * v9_dot(pU, d), c = v9_normsq(pU) - delta * delta;
    const float disc = b * b - 4.f * a * c;
    if (disc < 0.f || a < 1e-20f) return false;
    const float tau = ((-b + sqrtf(disc)) / (2.f * a));
    for (int i = 0; i < 9; i++) p[i] = pU[i] + tau * d[i];
    return true;
}

// ── main FK solver (trust-region dogleg) ─────────────────────────────────────

struct FKSolverOut {
    float x[9];
    float cost;
    int iters;
    bool success;
};

static FKSolverOut fk_solve(const FKProblem& P, const float x0[9]) {
    FKSolverOut out{};
    v9_copy(out.x, x0);
    out.success = false;

    const float gtol = 1e-6f, ftol = 1e-10f, xtol = 1e-7f;
    const float eta = 0.1f, delta_max = 2.f;
    float delta = 0.25f;

    for (int iter = 0; iter < 100; ++iter) {
        float F[9], J[9][9];
        fk_residual_and_jac(P, out.x, F, J);
        out.cost = 0.5f * v9_dot(F, F);
        out.iters = iter;

        float g[9], B[9][9];
        fk_normal_eqs(F, J, g, B);

        if (v9_norm(g) < gtol) {
            out.success = true;
            break;
        }

        float p[9];
        if (!dogleg_step(g, B, delta, p)) break;

        const float pn = v9_norm(p);
        if (pn < xtol) {
            out.success = true;
            break;
        }

        const float pred = pred_reduction(g, B, p);
        if (pred <= 1e-20f) break;

        float xt[9];
        v9_add(xt, out.x, p);
        float Ft[9], Jt[9][9];
        fk_residual_and_jac(P, xt, Ft, Jt);
        const float cost_t = 0.5f * v9_dot(Ft, Ft);
        const float rho = (out.cost - cost_t) / pred;

        // trust-region radius update
        if (rho < 0.25f)
            delta *= 0.25f;
        else if (rho > 0.75f && fabsf(pn - delta) < 1e-5f)
            delta = fminf(2.f * delta, delta_max);

        if (rho > eta) {
            v9_copy(out.x, xt);
            if (fabsf(out.cost - cost_t) < ftol) {
                out.success = true;
                out.cost = cost_t;
                break;
            }
        }
        if (delta < 1e-8f) break;
    }
    return out;
}

// ---------------------------------------------------------------------------
// SVD-based polar decomposition  (replaces Newton iteration)
// ---------------------------------------------------------------------------
//
// Given M (3×3), compute the nearest rotation R ∈ SO(3) via:
//   M = U · Σ · V^T   →   R = U · V^T   (with det-correction)
//
// Algorithm
// ─────────
// 1. Form B = M^T M  (symmetric PSD, 3×3).
// 2. Jacobi-diagonalise B: accumulate V so that V^T B V = diag(σ²).
//    Each sweep finds the largest off-diagonal entry |B[p][q]| and
//    zeroes it with a Givens rotation.  Convergence is quadratic and
//    typically needs ≤ 10 sweeps for 3×3.
// 3. σ_i = sqrt(B[i][i]) after diagonalisation.
// 4. U columns: u_j = M v_j / σ_j  (normalised via the 2-norm of M v_j,
//    which equals σ_j when v_j is a unit eigenvector of B).
//    Near-zero σ_j are treated as zero (zero column in U).
// 5. R = U V^T.  If det(R) = −1 (a reflection), flip the column of U
//    for the smallest σ and recompute — this gives the nearest SO(3) matrix.
//
// Why SVD is better than Newton here
// ────────────────────────────────────
// The default SPM geometry has w_i0 nearly coplanar (z ≈ cos 85° ≈ 0.087),
// making σ_min(W_i0) ≈ 0.023.  The Newton polar iteration maps each
// singular value σ → (σ + 1/σ)/2 and needs ~8 steps to pull σ = 0.023
// up to 1; with only 5 steps the result was σ ≈ 1.6 (visible as m33 > 1).
// SVD has no such dependence on the condition number.

// ── Jacobi SVD ───────────────────────────────────────────────────────────────

// Compute SVD of a row-major 3×3 matrix M:
//   M = U · diag(s) · V^T
// U and V are orthogonal (columns are eigenvectors / left/right singular
// vectors respectively).  s[i] ≥ 0.  Neither U nor V is guaranteed to
// have determinant +1 — the caller handles the SO(3) correction.
static void svd3x3(const float M[3][3], float U[3][3], float s[3], float V[3][3]) {
    // ── Step 1: B = M^T M  (symmetric PSD) ──────────────────────────────────
    float B[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float acc = 0.f;
            for (int k = 0; k < 3; k++) acc += M[k][r] * M[k][c];
            B[r][c] = acc;
        }

    // ── Step 2: Jacobi diagonalisation of B, V accumulates eigenvectors ─────
    // Initialise V = I₃
    float Vm[3][3] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};

    for (int sweep = 0; sweep < 30; ++sweep) {
        // Find index (p,q) of the largest off-diagonal |B[p][q]|
        int p = 0, q = 1;
        float maxOff = fabsf(B[0][1]);
        if (fabsf(B[0][2]) > maxOff) {
            maxOff = fabsf(B[0][2]);
            p = 0;
            q = 2;
        }
        if (fabsf(B[1][2]) > maxOff) {
            maxOff = fabsf(B[1][2]);
            p = 1;
            q = 2;
        }
        if (maxOff < 1e-12f) break;  // converged

        // Jacobi angle: choose t = tan θ to zero B[p][q]
        //   τ = (B[q][q] - B[p][p]) / (2 B[p][q])
        //   t = sign(τ) / (|τ| + sqrt(1 + τ²))
        float tau = (B[q][q] - B[p][p]) / (2.f * B[p][q]);
        float t = (tau >= 0.f) ? 1.f / (tau + sqrtf(1.f + tau * tau))
                               : 1.f / (tau - sqrtf(1.f + tau * tau));
        float c = 1.f / sqrtf(1.f + t * t);  // cos θ
        float sv = t * c;                    // sin θ  ('sv' avoids clash with s[])

        // Symmetric update: B ← Jᵀ B J  (only 5 entries change)
        float Bpp = B[p][p], Bqq = B[q][q], Bpq = B[p][q];
        B[p][p] = Bpp - t * Bpq;  // diagonal pp
        B[q][q] = Bqq + t * Bpq;  // diagonal qq
        B[p][q] = B[q][p] = 0.f;  // off-diagonal zeroed
        for (int r = 0; r < 3; r++) {
            if (r == p || r == q) continue;
            float Brp = B[r][p], Brq = B[r][q];
            B[r][p] = B[p][r] = c * Brp - sv * Brq;
            B[r][q] = B[q][r] = sv * Brp + c * Brq;
        }

        // Accumulate V ← V · J
        for (int r = 0; r < 3; r++) {
            float Vrp = Vm[r][p], Vrq = Vm[r][q];
            Vm[r][p] = c * Vrp - sv * Vrq;
            Vm[r][q] = sv * Vrp + c * Vrq;
        }
    }

    // ── Step 3: singular values and V ────────────────────────────────────────
    for (int i = 0; i < 3; i++) {
        s[i] = sqrtf(fabsf(B[i][i]));
        for (int r = 0; r < 3; r++) V[r][i] = Vm[r][i];
    }

    // ── Step 4: U columns  u_j = M v_j / σ_j  ───────────────────────────────
    // ||M v_j|| = σ_j when v_j is a unit right singular vector, so normalising
    // by the Euclidean norm of M v_j is equivalent and avoids division by σ_j.
    for (int j = 0; j < 3; j++) {
        // col = M * v_j
        float col[3] = {0.f, 0.f, 0.f};
        for (int r = 0; r < 3; r++)
            for (int k = 0; k < 3; k++) col[r] += M[r][k] * Vm[k][j];

        float norm = sqrtf(col[0] * col[0] + col[1] * col[1] + col[2] * col[2]);
        float inv = (norm > 1e-10f) ? 1.f / norm : 0.f;
        U[0][j] = col[0] * inv;
        U[1][j] = col[1] * inv;
        U[2][j] = col[2] * inv;
    }
}

// ── 3×3 determinant (row-major) ──────────────────────────────────────────────
static float det3(const float A[3][3]) {
    return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
           A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
           A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
}

// ── Polar factor: nearest SO(3) matrix to M ──────────────────────────────────
//
//   R = U V^T
//
// If det(U V^T) = −1 (a reflection rather than a rotation), flip the column
// of U with the smallest singular value — this minimises the change to U while
// making det = +1.
static void polar3(const float M[3][3], Matrix33f& R) {
    float U[3][3], s[3], V[3][3];
    svd3x3(M, U, s, V);

    // R = U · V^T
    float Rt[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float acc = 0.f;
            for (int k = 0; k < 3; k++) acc += U[r][k] * V[c][k];  // V^T[k][c] = V[c][k]
            Rt[r][c] = acc;
        }

    // det-correction: if det = −1, flip the U column for the smallest σ
    if (det3(Rt) < 0.f) {
        int minIdx = (s[0] < s[1]) ? 0 : 1;
        if (s[2] < s[minIdx]) minIdx = 2;
        for (int r = 0; r < 3; r++) U[r][minIdx] = -U[r][minIdx];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                float acc = 0.f;
                for (int k = 0; k < 3; k++) acc += U[r][k] * V[c][k];
                Rt[r][c] = acc;
            }
    }

    R.m11 = Rt[0][0];
    R.m12 = Rt[0][1];
    R.m13 = Rt[0][2];
    R.m21 = Rt[1][0];
    R.m22 = Rt[1][1];
    R.m23 = Rt[1][2];
    R.m31 = Rt[2][0];
    R.m32 = Rt[2][1];
    R.m33 = Rt[2][2];
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// SPMArch – default Coaxial Input SPM parameters
// ---------------------------------------------------------------------------

SPMArch::SPMArch() {
    const float two_pi_3 = 2.0f * (float)M_PI / 3.0f;
    const float four_pi_3 = 4.0f * (float)M_PI / 3.0f;

    eta_i1[0] = 0.0f;
    eta_i1[1] = two_pi_3;
    eta_i1[2] = four_pi_3;
    eta_i2[0] = 0.0f;
    eta_i2[1] = two_pi_3;
    eta_i2[2] = four_pi_3;

    gamma_i1[0] = 0.0f;
    gamma_i1[1] = 0.0f;
    gamma_i1[2] = 0.0f;

    // pi/2 - 5 deg small tilt to avoid singularity at neutral pose
    const float mp_tilt = (float)M_PI / 2.0f - 0.08726646f;
    gamma_i2[0] = mp_tilt;
    gamma_i2[1] = mp_tilt;
    gamma_i2[2] = mp_tilt;

    const float pi_3 = (float)M_PI / 3.0f;
    alpha_i1[0] = pi_3;
    alpha_i1[1] = pi_3;
    alpha_i1[2] = pi_3;

    const float pi_4 = (float)M_PI / 4.0f;
    alpha_i2[0] = pi_4;
    alpha_i2[1] = pi_4;
    alpha_i2[2] = pi_4;
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

mVector3f SPMModel::computeJointVector(float eta, float gamma) {
    // Rz(eta) * Rx(gamma) * [0,0,1] = [sin(eta)*sin(gamma), -cos(eta)*sin(gamma), cos(gamma)]
    float ce = cosf(eta), se = sinf(eta);
    float cg = cosf(gamma), sg = sinf(gamma);
    return mVector3f(se * sg, -ce * sg, cg);
}

mVector3f SPMModel::computeVVector(float theta, float a1, float b1, float c1, float a2, float b2,
                                   float c2, float a3, float c3) {
    // v_i(theta) = [a1*cos+b1*sin+c1, a2*cos+b2*sin+c2, a3*cos+c3]
    float ct = cosf(theta), st = sinf(theta);
    return mVector3f(a1 * ct + b1 * st + c1, a2 * ct + b2 * st + c2, a3 * ct + c3);
}

mVector3f SPMModel::mat3MulVec(const Matrix33f& M, const mVector3f& v) {
    return mVector3f(M.m11 * v.x + M.m12 * v.y + M.m13 * v.z,
                     M.m21 * v.x + M.m22 * v.y + M.m23 * v.z,
                     M.m31 * v.x + M.m32 * v.y + M.m33 * v.z);
}

bool SPMModel::verifyKinematicConstraints() {
    bool all_valid = true;
    for (int i = 0; i < SPM_LEGS; i++) {
        if ((v_i[i] ^ w_i[i]) - cosf(arch_.alpha_i2[i]) > 1e-6f) {
            all_valid = false;
            break;
        }

        if ((u_i[i] ^ v_i[i]) - cosf(arch_.alpha_i1[i]) > 1e-6f) {
            all_valid = false;
            break;
        }
    }
    return all_valid;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

SPMModel::SPMModel(const SPMArch& arch) : is_initialized(false), arch_(arch) {
    theta_sol_indices[0] = 0;
    theta_sol_indices[1] = 0;
    theta_sol_indices[2] = 0;

    // Precompute fixed joint axes and proximal-link coefficients
    for (int i = 0; i < SPM_LEGS; i++) {
        float ce1 = cosf(arch_.eta_i1[i]), se1 = sinf(arch_.eta_i1[i]);
        float cg1 = cosf(arch_.gamma_i1[i]), sg1 = sinf(arch_.gamma_i1[i]);
        float ca1 = cosf(arch_.alpha_i1[i]), sa1 = sinf(arch_.alpha_i1[i]);

        u_i[i] = computeJointVector(arch_.eta_i1[i], arch_.gamma_i1[i]);
        w_i0[i] = computeJointVector(arch_.eta_i2[i], arch_.gamma_i2[i]);

        // Proximal link parameterisation coefficients from symbolic derivation:
        //   v_i(theta) = [a1*cos+b1*sin+c1, a2*cos+b2*sin+c2, a3*cos+c3]
        a1_i[i] = se1 * cg1 * sa1;
        b1_i[i] = ce1 * sa1;
        c1_i[i] = se1 * sg1 * ca1;

        a2_i[i] = -ce1 * cg1 * sa1;
        b2_i[i] = se1 * sa1;
        c2_i[i] = -ce1 * sg1 * ca1;

        a3_i[i] = -sg1 * sa1;
        c3_i[i] = cg1 * ca1;
    }

    // Solve IK at neutral pose (R = Identity)
    Matrix33f R_identity;
    Matrix33f::Identity(&R_identity);

    IKResult neutral = computeIK(R_identity, /*update_config=*/true);

    bool all_valid = true;
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!neutral.has_solution[i]) {
            all_valid = false;
            break;
        }
    }

    if (all_valid) {
        for (int i = 0; i < SPM_LEGS; i++) {
            thetas_neutral[i] = neutral.theta[i][theta_sol_indices[i]];
            theta_i[i] = thetas_neutral[i];
        }
        J_neutral = computeVelJacobian();
        is_initialized = true;
    } else {
        for (int i = 0; i < SPM_LEGS; i++) {
            thetas_neutral[i] = 0.0f;
            theta_i[i] = 0.0f;
        }
        J_neutral.is_valid = false;
    }
}

// ---------------------------------------------------------------------------
// computeIK
// ---------------------------------------------------------------------------

IKResult SPMModel::computeIK(const Matrix33f& R, bool update_config) {
    IKResult result;

    // w_i = R * w_i0 for each leg
    mVector3f w[SPM_LEGS];
    for (int i = 0; i < SPM_LEGS; i++) {
        w[i] = mat3MulVec(R, w_i0[i]);
    }

    // Per-leg: A_i*cos(th) + B_i*sin(th) = C_i  from v_i . w_i = cos(alpha_i2)
    for (int i = 0; i < SPM_LEGS; i++) {
        float wx = w[i].x, wy = w[i].y, wz = w[i].z;

        float A = wx * a1_i[i] + wy * a2_i[i] + wz * a3_i[i];
        float B = wx * b1_i[i] + wy * b2_i[i];  // b3 = 0
        float C = cosf(arch_.alpha_i2[i]) - (wx * c1_i[i] + wy * c2_i[i] + wz * c3_i[i]);

        TrigResultf sol = solveTrigEquation(A, B, C);

        result.has_solution[i] = (sol.count > 0);
        result.num_solutions[i] = (uint8_t)sol.count;
        result.theta[i][0] = sol.theta[0];
        result.theta[i][1] = sol.theta[1];
    }

    if (update_config) {
        bool all_valid = true;
        for (int i = 0; i < SPM_LEGS; i++) {
            if (!result.has_solution[i]) {
                all_valid = false;
                break;
            }
        }

        if (all_valid) {
            for (int i = 0; i < SPM_LEGS; i++) {
                int idx = theta_sol_indices[i];
                float th = result.theta[i][idx];
                // wrap to [0, 2π) since atan2f returns (-π, π]
                if (th < 0.f) th += 2.f * (float)M_PI;
                theta_i[i] = th;
                w_i[i] = w[i];
                v_i[i] = computeVVector(th, a1_i[i], b1_i[i], c1_i[i], a2_i[i], b2_i[i], c2_i[i],
                                        a3_i[i], c3_i[i]);
            }
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// computeVelJacobian
// ---------------------------------------------------------------------------

VelJacobian SPMModel::computeVelJacobian() const {
    VelJacobian jac;
    jac.is_valid = false;

    // Zero B_mat; only diagonal entries are set
    jac.B_mat = Matrix33f(0, 0, 0, 0, 0, 0, 0, 0, 0);

    // Row i of A_mat = w_i x v_i;  B_mat[i,i] = u_i . (w_i x v_i)
    for (int i = 0; i < SPM_LEGS; i++) {
        mVector3f wxv = w_i[i] * v_i[i];  // cross product operator* in mVector3

        if (i == 0) {
            jac.A_mat.m11 = wxv.x;
            jac.A_mat.m12 = wxv.y;
            jac.A_mat.m13 = wxv.z;
            jac.B_mat.m11 = u_i[i] ^ wxv;  // dot product operator^
        } else if (i == 1) {
            jac.A_mat.m21 = wxv.x;
            jac.A_mat.m22 = wxv.y;
            jac.A_mat.m23 = wxv.z;
            jac.B_mat.m22 = u_i[i] ^ wxv;
        } else {
            jac.A_mat.m31 = wxv.x;
            jac.A_mat.m32 = wxv.y;
            jac.A_mat.m33 = wxv.z;
            jac.B_mat.m33 = u_i[i] ^ wxv;
        }
    }

    float det = Matrix33f::Determinant(&jac.A_mat);
    if (fabsf(det) < 1e-6f) {
        // #ifdef ARDUINO
        //         Serial.println(F("[SPMModel] Warning: Jacobian A_mat is singular."));
        // #endif
        return jac;
    }

    Matrix33f A_inv;
    Matrix33f::Invert(&A_inv, &jac.A_mat);
    jac.J = A_inv * jac.B_mat;
    jac.is_valid = true;

    return jac;
}

// ---------------------------------------------------------------------------
// computeFK
// ---------------------------------------------------------------------------

FKResult SPMModel::computeFK(const float theta[SPM_LEGS], bool update_config, bool init_prev_sol) {
    FKResult result{};
    result.success = false;

    // ── build FK problem ─────────────────────────────────────────────────────

    FKProblem P;

    // v_i: proximal link tip vectors computed from the given joint angles
    for (int i = 0; i < SPM_LEGS; i++) {
        mVector3f vi = computeVVector(theta[i], a1_i[i], b1_i[i], c1_i[i], a2_i[i], b2_i[i],
                                      c2_i[i], a3_i[i], c3_i[i]);
        P.v[i][0] = vi.x;
        P.v[i][1] = vi.y;
        P.v[i][2] = vi.z;
    }

    // c2[i] = cos(alpha_i2): target for v_i · w_i
    for (int i = 0; i < SPM_LEGS; i++) P.c2[i] = cosf(arch_.alpha_i2[i]);

    // c3: mutual dot products between body-frame attachment vectors w_i0
    // Since w_i · w_j = (R w_i0) · (R w_j0) = w_i0 · w_j0, these are constant.
    // Ordering matches residual layout: [w1·w2, w2·w0, w0·w1]
    P.c3[0] = w_i0[1] ^ w_i0[2];
    P.c3[1] = w_i0[2] ^ w_i0[0];
    P.c3[2] = w_i0[0] ^ w_i0[1];

    // ── initial guess ────────────────────────────────────────────────────────

    float x0[9];
    if (init_prev_sol) {
        // warm start: use w_i from the last accepted configuration
        for (int i = 0; i < SPM_LEGS; i++) {
            x0[3 * i + 0] = w_i[i].x;
            x0[3 * i + 1] = w_i[i].y;
            x0[3 * i + 2] = w_i[i].z;
        }
    } else {
        // cold start: body-frame vectors are a reasonable neutral guess
        for (int i = 0; i < SPM_LEGS; i++) {
            x0[3 * i + 0] = w_i0[i].x;
            x0[3 * i + 1] = w_i0[i].y;
            x0[3 * i + 2] = w_i0[i].z;
        }
    }

    // ── solve ────────────────────────────────────────────────────────────────

    FKSolverOut sol = fk_solve(P, x0);

    result.iterations = sol.iters;
    result.cost = sol.cost;
    result.success = sol.success && (sol.cost < 1e-6f);

    for (int i = 0; i < SPM_LEGS; i++) {
        result.w[i][0] = sol.x[3 * i + 0];
        result.w[i][1] = sol.x[3 * i + 1];
        result.w[i][2] = sol.x[3 * i + 2];
    }

    // ── recover R via polar decomposition of M = W * B^T ────────────────────
    //
    // M = sum_i outer(w_i, w_i0[i])   (= W * B^T using column convention)
    // Polar iteration: R <- (R + R^{-T}) / 2  converges to nearest SO(3) matrix

    Matrix33f M(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (int i = 0; i < SPM_LEGS; i++) {
        // outer product:  M += w_sol[i] * w_i0[i]^T
        M.m11 += sol.x[3 * i + 0] * w_i0[i].x;
        M.m12 += sol.x[3 * i + 0] * w_i0[i].y;
        M.m13 += sol.x[3 * i + 0] * w_i0[i].z;

        M.m21 += sol.x[3 * i + 1] * w_i0[i].x;
        M.m22 += sol.x[3 * i + 1] * w_i0[i].y;
        M.m23 += sol.x[3 * i + 1] * w_i0[i].z;

        M.m31 += sol.x[3 * i + 2] * w_i0[i].x;
        M.m32 += sol.x[3 * i + 2] * w_i0[i].y;
        M.m33 += sol.x[3 * i + 2] * w_i0[i].z;
    }

    // Recover R ∈ SO(3) via SVD-based polar decomposition: R = U V^T
    float Marr[3][3] = {{M.m11, M.m12, M.m13}, {M.m21, M.m22, M.m23}, {M.m31, M.m32, M.m33}};
    polar3(Marr, result.R);

    // ── optional state update ────────────────────────────────────────────────

    if (result.success && update_config) {
        for (int i = 0; i < SPM_LEGS; i++) {
            w_i[i] = mVector3f(result.w[i][0], result.w[i][1], result.w[i][2]);
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// Link surpass detection
// ---------------------------------------------------------------------------

/**
 * @brief CCW arc from t1 to t2 on [0, 2π).
 *
 * Uses the floor-based modulo definition:
 *   a mod m = a - m * floor(a / m)
 * which always returns a value in [0, m) for positive m,
 * correctly handling both negative inputs and inputs beyond 2π.
 */
static inline float ccwArc(float t1, float t2) {
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    float diff = t2 - t1;
    // floor-based modulo: diff - two_pi * floor(diff / two_pi)
    return diff - two_pi * floorf(diff / two_pi);
}

/** @brief Normalize angle to [0, 2π) using floor-based modulo. */
static inline float normalizeAngle(float theta) {
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    return theta - two_pi * floorf(theta / two_pi);
}

bool SPMModel::linkSurpassOccur(const float test_thetas[SPM_LEGS], float margin) {
    // only valid for coaxial case
    for (int i = 0; i < SPM_LEGS; i++) {
        if (arch_.gamma_i1[i] != 0.0f) return false;
    }

    // adjusted angles: wrap(theta[i] + eta_i1[i]) → [0, 2π)
    float a[SPM_LEGS];
    for (int i = 0; i < SPM_LEGS; i++) {
        a[i] = normalizeAngle(test_thetas[i] + arch_.eta_i1[i]);
    }

    const float two_pi = 2.0f * static_cast<float>(M_PI);
    float ccw12 = ccwArc(a[0], a[1]);
    float ccw23 = ccwArc(a[1], a[2]);
    float ccw31 = ccwArc(a[2], a[0]);

    // surpass: links not in CCW cyclic order 1→2→3
    float sum = ccw12 + ccw23 + ccw31;
    if (fabsf(sum - two_pi) > 1e-5f) return true;

    // margin: shortest arc between each pair must be >= margin
    float arcs[3] = {ccw12, ccw23, ccw31};
    float abs_margin = fabsf(margin);
    for (int i = 0; i < 3; i++) {
        float shortest = (arcs[i] < two_pi - arcs[i]) ? arcs[i] : two_pi - arcs[i];
        if (shortest < abs_margin - 1e-6f) return true;
    }

    return false;
}
