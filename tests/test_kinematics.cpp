/**
 * @file test_kinematics.cpp
 * @brief Unit tests for SPMModel: construction, IK, FK, Jacobian, math utilities
 */

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "mMath.h"
#include "mSPMModel.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static const float TOL = 1e-6f;     // general tolerance
static const float TOL_FK = 1e-6f;  // FK round-trip tolerance (iterative solver)

static bool near(float a, float b, float tol = TOL) { return fabsf(a - b) < tol; }

static bool matricesNear(const Matrix33f& A, const Matrix33f& B, float tol = TOL) {
    return near(A.m11, B.m11, tol) && near(A.m12, B.m12, tol) && near(A.m13, B.m13, tol) &&
           near(A.m21, B.m21, tol) && near(A.m22, B.m22, tol) && near(A.m23, B.m23, tol) &&
           near(A.m31, B.m31, tol) && near(A.m32, B.m32, tol) && near(A.m33, B.m33, tol);
}

/// Rotation about Z-axis
static Matrix33f Rz(float a) {
    float c = cosf(a), s = sinf(a);
    return Matrix33f(c, -s, 0.f, s, c, 0.f, 0.f, 0.f, 1.f);
}

/// Rotation about X-axis
static Matrix33f Rx(float a) {
    float c = cosf(a), s = sinf(a);
    return Matrix33f(1.f, 0.f, 0.f, 0.f, c, -s, 0.f, s, c);
}

/// Rotation about Y-axis
static Matrix33f Ry(float a) {
    float c = cosf(a), s = sinf(a);
    return Matrix33f(c, 0.f, s, 0.f, 1.f, 0.f, -s, 0.f, c);
}

// ---------------------------------------------------------------------------
// Test runner
// ---------------------------------------------------------------------------

static int g_passed = 0;
static int g_total = 0;

// RUN(fn) — execute one test function, print result + wall-clock time.
//
//  g_total++
//      Count every test attempted (pass or fail).
//
//  std::printf("  %-42s", #fn " ...")
//      Print the function name left-aligned in a 42-char field so the
//      PASS/FAIL column lines up regardless of name length.
//      #fn is the preprocessor stringification of the function name.
//
//  high_resolution_clock::now()  (called before and after fn())
//      Capture wall-clock timestamps with the highest available resolution.
//
//  bool ok = fn()
//      Call the test; true = passed, false = failed.
//
//  duration<double, std::micro>(_t1 - _t0).count()
//      Convert the tick difference to microseconds as a double.
//
//  std::printf(..., ok ? "PASS" : "FAIL", _us)
//      Print result and elapsed time; column width keeps output aligned.
//
//  if (ok) g_passed++
//      Only increment the pass counter on success.
#define RUN(fn)                                                                    \
    do {                                                                           \
        g_total++;                                                                 \
        std::printf("  %-42s", #fn " ...");                                        \
        auto _t0 = std::chrono::high_resolution_clock::now();                      \
        bool ok = fn();                                                            \
        auto _t1 = std::chrono::high_resolution_clock::now();                      \
        double _us = std::chrono::duration<double, std::micro>(_t1 - _t0).count(); \
        std::printf("%s  %8.2f us\n", ok ? "PASS" : "FAIL", _us);                  \
        if (ok) g_passed++;                                                        \
    } while (0)

// ---------------------------------------------------------------------------
// 1. Math utilities
// ---------------------------------------------------------------------------

static bool test_solveQuadratic_two_roots() {
    // x² - 3x + 2 = 0  →  x = 2, 1
    QuadraticResultf r = solveQuadratic(1.f, -3.f, 2.f);
    if (!r.hasRealRoots) return false;
    bool ok = (near(r.x1, 2.f) && near(r.x2, 1.f)) || (near(r.x1, 1.f) && near(r.x2, 2.f));
    return ok;
}

static bool test_solveQuadratic_no_roots() {
    // x² + 1 = 0  →  no real roots
    QuadraticResultf r = solveQuadratic(1.f, 0.f, 1.f);
    return !r.hasRealRoots;
}

static bool test_solveQuadratic_repeated_root() {
    // x² - 2x + 1 = 0  →  x = 1 (double)
    QuadraticResultf r = solveQuadratic(1.f, -2.f, 1.f);
    return r.hasRealRoots && near(r.x1, 1.f) && near(r.x2, 1.f);
}

static bool test_solveTrigEquation_basic() {
    // cos(θ) = 0.5  →  θ = ±π/3
    TrigResultf r = solveTrigEquation(1.f, 0.f, 0.5f);
    if (r.count != 2) return false;
    float expected = acosf(0.5f);  // π/3
    return (near(fabsf(r.theta[0]), expected) && near(fabsf(r.theta[1]), expected));
}

static bool test_solveTrigEquation_no_solution() {
    // 0·cos(θ) + 0·sin(θ) = 2  →  no solution
    TrigResultf r = solveTrigEquation(0.f, 0.f, 2.f);
    return (r.count == 0);
}

static bool test_solveTrigEquation_verify_residual() {
    // 3·cos(θ) + 4·sin(θ) = 2
    float A = 3.f, B = 4.f, C = 2.f;
    TrigResultf r = solveTrigEquation(A, B, C);
    if (r.count == 0) return false;
    for (int i = 0; i < r.count; i++) {
        float residual = A * cosf(r.theta[i]) + B * sinf(r.theta[i]) - C;
        if (!near(residual, 0.f, 1e-5f)) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// 2. SPMModel construction
// ---------------------------------------------------------------------------

static bool test_construction_default() {
    SPMModel spm;
    return spm.is_initialized;
}

static bool test_neutral_thetas_finite() {
    SPMModel spm;
    if (!spm.is_initialized) return false;
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!std::isfinite(spm.thetas_neutral[i])) return false;
    }
    return true;
}

static bool test_jacobian_valid_at_neutral() {
    SPMModel spm;
    return spm.is_initialized && spm.J_neutral.is_valid;
}

// ---------------------------------------------------------------------------
// 3. Inverse Kinematics
// ---------------------------------------------------------------------------

static bool test_ik_identity() {
    SPMModel spm;
    Matrix33f R;
    Matrix33f::Identity(&R);
    IKResult ik = spm.computeIK(R, false);
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!ik.has_solution[i]) return false;
    }
    return true;
}

static bool test_ik_neutral_matches_stored() {
    // IK at identity should recover the stored neutral thetas
    SPMModel spm;
    if (!spm.is_initialized) return false;
    Matrix33f R;
    Matrix33f::Identity(&R);
    IKResult ik = spm.computeIK(R, false);
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!ik.has_solution[i]) return false;
        bool branch0_matches = near(ik.theta[i][0], spm.thetas_neutral[i], 1e-3f);
        bool branch1_matches =
            (ik.num_solutions[i] > 1) && near(ik.theta[i][1], spm.thetas_neutral[i], 1e-3f);
        if (!branch0_matches && !branch1_matches) return false;
    }
    return true;
}

static bool test_ik_small_rz() {
    SPMModel spm;
    IKResult ik = spm.computeIK(Rz(0.1f), false);
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!ik.has_solution[i]) return false;
    }
    return true;
}

static bool test_ik_small_rx() {
    SPMModel spm;
    IKResult ik = spm.computeIK(Rx(0.15f), false);
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!ik.has_solution[i]) return false;
    }
    return true;
}

static bool test_ik_combined_rotation() {
    SPMModel spm;
    // Compose Rz * Rx — a typical ankle-like pose
    Matrix33f R = Rz(0.1f) * Rx(0.1f);
    IKResult ik = spm.computeIK(R, false);
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!ik.has_solution[i]) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// 4. Forward Kinematics (IK → FK round-trip)
// ---------------------------------------------------------------------------

static bool ik_fk_roundtrip(const Matrix33f& R_target, const char* label = "") {
    SPMModel spm;

    IKResult ik = spm.computeIK(R_target, /*update_config=*/true);
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!ik.has_solution[i]) {
            std::printf("    [%s] IK: no solution for leg %d\n", label, i);
            return false;
        }
    }

    float theta[SPM_LEGS];
    for (int i = 0; i < SPM_LEGS; i++) theta[i] = ik.theta[i][spm.theta_sol_indices[i]];

    // warm start: w_i were just set by computeIK above
    FKResult fk = spm.computeFK(theta, /*update_config=*/false, /*init_prev_sol=*/true);

    if (!fk.success) {
        std::printf("    [%s] FK failed: success=%d cost=%.3e iters=%d\n", label, fk.success,
                    (double)fk.cost, fk.iterations);
        return false;
    }

    bool ok = matricesNear(fk.R, R_target, TOL_FK);
    if (!ok) {
        std::printf("    [%s] R mismatch (cost=%.3e):\n", label, (double)fk.cost);
        std::printf("      target  [%.4f %.4f %.4f | %.4f %.4f %.4f | %.4f %.4f %.4f]\n",
                    (double)R_target.m11, (double)R_target.m12, (double)R_target.m13,
                    (double)R_target.m21, (double)R_target.m22, (double)R_target.m23,
                    (double)R_target.m31, (double)R_target.m32, (double)R_target.m33);
        std::printf("      fk.R    [%.4f %.4f %.4f | %.4f %.4f %.4f | %.4f %.4f %.4f]\n",
                    (double)fk.R.m11, (double)fk.R.m12, (double)fk.R.m13, (double)fk.R.m21,
                    (double)fk.R.m22, (double)fk.R.m23, (double)fk.R.m31, (double)fk.R.m32,
                    (double)fk.R.m33);
    }
    return ok;
}

static bool test_fk_at_identity() {
    Matrix33f R;
    Matrix33f::Identity(&R);
    return ik_fk_roundtrip(R, "identity");
}

static bool test_fk_rz_10deg() { return ik_fk_roundtrip(Rz(10.f * (float)M_PI / 180.f), "Rz10"); }

static bool test_fk_rx_10deg() { return ik_fk_roundtrip(Rx(10.f * (float)M_PI / 180.f), "Rx10"); }

static bool test_fk_ry_10deg() { return ik_fk_roundtrip(Ry(10.f * (float)M_PI / 180.f), "Ry10"); }

static bool test_fk_combined() {
    Matrix33f R = Rz(0.1f) * Rx(0.1f);
    return ik_fk_roundtrip(R, "Rz*Rx");
}

static bool test_fk_warm_start() {
    // Second call uses warm-start from a previous FK update
    SPMModel spm;
    Matrix33f R = Rx(0.12f);

    IKResult ik = spm.computeIK(R, /*update_config=*/true);
    for (int i = 0; i < SPM_LEGS; i++) {
        if (!ik.has_solution[i]) return false;
    }

    float theta[SPM_LEGS];
    for (int i = 0; i < SPM_LEGS; i++) theta[i] = ik.theta[i][spm.theta_sol_indices[i]];

    // Cold start first to populate fk result state
    FKResult fk1 = spm.computeFK(theta, /*update_config=*/true, /*init_prev_sol=*/false);
    if (!fk1.success) {
        std::printf("    [warm_start] cold-start FK failed: cost=%.3e\n", (double)fk1.cost);
        return false;
    }
    // Warm start should converge equally or faster
    FKResult fk2 = spm.computeFK(theta, /*update_config=*/false, /*init_prev_sol=*/true);
    if (!fk2.success) {
        std::printf("    [warm_start] warm-start FK failed: cost=%.3e\n", (double)fk2.cost);
        return false;
    }
    return matricesNear(fk2.R, R, TOL_FK);
}

// ---------------------------------------------------------------------------
// 5. Velocity Jacobian
// ---------------------------------------------------------------------------

static bool test_jacobian_valid_after_ik() {
    SPMModel spm;
    Matrix33f R = Rz(0.1f);
    spm.computeIK(R, /*update_config=*/true);
    VelJacobian jac = spm.computeVelJacobian();
    return jac.is_valid;
}

static bool test_jacobian_b_diagonal() {
    // B_mat must be diagonal (off-diagonal entries zero)
    SPMModel spm;
    spm.computeIK(Rx(0.1f), /*update_config=*/true);
    VelJacobian jac = spm.computeVelJacobian();
    if (!jac.is_valid) return false;
    return near(jac.B_mat.m12, 0.f) && near(jac.B_mat.m13, 0.f) && near(jac.B_mat.m21, 0.f) &&
           near(jac.B_mat.m23, 0.f) && near(jac.B_mat.m31, 0.f) && near(jac.B_mat.m32, 0.f);
}

// ---------------------------------------------------------------------------
// 6. Kinematic constraints
// ---------------------------------------------------------------------------

static bool test_kinematic_constraints_after_ik() {
    SPMModel spm;
    Matrix33f R = Rz(0.1f);
    spm.computeIK(R, /*update_config=*/true);
    return spm.verifyKinematicConstraints();
}

// ---------------------------------------------------------------------------
// 7. Link Surpass Detection
// ---------------------------------------------------------------------------

static const float kDeg = (float)M_PI / 180.f;
static const float kMargin10 = 10.f * kDeg;  // 10° safety margin used throughout

static bool test_surpass_neutral_no_surpass() {
    // Neutral configuration — guaranteed valid, no surpass expected
    SPMModel spm;
    return !spm.linkSurpassOccur(spm.thetas_neutral, kMargin10);
}

static bool test_surpass_leg1_passes_leg2() {
    // (45°,-90°,0°) → adjusted (45°,30°,240°) — leg 1 overtakes leg 2
    SPMModel spm;
    float t[SPM_LEGS] = {45.f * kDeg, -90.f * kDeg, 0.f};
    return spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_order_preserved() {
    // (0°,-105°,-60°) → adjusted (0°,15°,180°) — CCW order intact, min gap 15° > 10° margin
    SPMModel spm;
    float t[SPM_LEGS] = {0.f, -105.f * kDeg, -60.f * kDeg};
    return !spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_margin_collision() {
    // (45°,-90°,-205°) → adjusted (45°,30°,35°) — no surpass but 5° gap < 10° margin
    SPMModel spm;
    float t[SPM_LEGS] = {45.f * kDeg, -90.f * kDeg, -205.f * kDeg};
    return spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_uniform_displacement() {
    // (1°,1°,1°) → adjusted (1°,121°,241°) — evenly spaced, no surpass
    SPMModel spm;
    float t[SPM_LEGS] = {1.f * kDeg, 1.f * kDeg, 1.f * kDeg};
    return !spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_leg2_passes_leg3() {
    // (0°,15°,-130°) → adjusted (0°,135°,110°) — leg 2 overtakes leg 3
    SPMModel spm;
    float t[SPM_LEGS] = {0.f, 15.f * kDeg, -130.f * kDeg};
    return spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_degenerate_coincident() {
    // (0°,-120°,-60°) → adjusted (0°,0°,180°) — legs 1&2 coincident, shortest arc=0 < margin
    SPMModel spm;
    float t[SPM_LEGS] = {0.f, -120.f * kDeg, -60.f * kDeg};
    return spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_margin_5deg() {
    // (0°,-115°,-60°) → adjusted (0°,5°,180°) — 5° gap < 10° margin
    SPMModel spm;
    float t[SPM_LEGS] = {0.f, -115.f * kDeg, -60.f * kDeg};
    return spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_margin_boundary_10deg() {
    // (0°,-110°,-60°) → adjusted (0°,10°,180°) — exactly 10°, not strictly less → no flag
    SPMModel spm;
    float t[SPM_LEGS] = {0.f, -110.f * kDeg, -60.f * kDeg};
    return !spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_wrap_large_positive() {
    // (720°,720°,720°) → adjusted (0°,120°,240°) after wrap — evenly spaced, no surpass
    SPMModel spm;
    float t[SPM_LEGS] = {720.f * kDeg, 720.f * kDeg, 720.f * kDeg};
    return !spm.linkSurpassOccur(t, kMargin10);
}

static bool test_surpass_wrap_large_negative() {
    // (-360°,-360°,-360°) → adjusted (0°,120°,240°) after wrap — no surpass
    SPMModel spm;
    float t[SPM_LEGS] = {-360.f * kDeg, -360.f * kDeg, -360.f * kDeg};
    return !spm.linkSurpassOccur(t, kMargin10);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::printf("\n================================================\n");
    std::printf("  SPM Kinematics Tests\n");
    std::printf("================================================\n\n");

    std::printf("[ Math utilities ]\n");
    RUN(test_solveQuadratic_two_roots);
    RUN(test_solveQuadratic_no_roots);
    RUN(test_solveQuadratic_repeated_root);
    RUN(test_solveTrigEquation_basic);
    RUN(test_solveTrigEquation_no_solution);
    RUN(test_solveTrigEquation_verify_residual);

    std::printf("\n[ SPMModel construction ]\n");
    RUN(test_construction_default);
    RUN(test_neutral_thetas_finite);
    RUN(test_jacobian_valid_at_neutral);

    std::printf("\n[ Inverse Kinematics ]\n");
    RUN(test_ik_identity);
    RUN(test_ik_neutral_matches_stored);
    RUN(test_ik_small_rz);
    RUN(test_ik_small_rx);
    RUN(test_ik_combined_rotation);

    std::printf("\n[ Forward Kinematics (IK-FK round-trip) ]\n");
    RUN(test_fk_at_identity);
    RUN(test_fk_rz_10deg);
    RUN(test_fk_rx_10deg);
    RUN(test_fk_ry_10deg);
    RUN(test_fk_combined);
    RUN(test_fk_warm_start);

    std::printf("\n[ Velocity Jacobian ]\n");
    RUN(test_jacobian_valid_after_ik);
    RUN(test_jacobian_b_diagonal);

    std::printf("\n[ Kinematic constraints ]\n");
    RUN(test_kinematic_constraints_after_ik);

    std::printf("\n[ Link Surpass Detection ]\n");
    RUN(test_surpass_neutral_no_surpass);
    RUN(test_surpass_leg1_passes_leg2);
    RUN(test_surpass_order_preserved);
    RUN(test_surpass_margin_collision);
    RUN(test_surpass_uniform_displacement);
    RUN(test_surpass_leg2_passes_leg3);
    RUN(test_surpass_degenerate_coincident);
    RUN(test_surpass_margin_5deg);
    RUN(test_surpass_margin_boundary_10deg);
    RUN(test_surpass_wrap_large_positive);
    RUN(test_surpass_wrap_large_negative);

    std::printf("\n================================================\n");
    std::printf("  Results: %d/%d passed\n", g_passed, g_total);
    std::printf("================================================\n\n");

    return (g_passed == g_total) ? 0 : 1;
}
