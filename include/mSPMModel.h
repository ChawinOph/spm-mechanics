/**
 * @file mSPMModel.h
 * @brief 3-RRR Spherical Parallel Mechanism (SPM) kinematics model
 *
 * @details
 * Models the Agile Eye-type 3-RRR SPM. Each leg i has three revolute joints
 * with unit axes on the unit sphere:
 *   u_i : first  joint axis (base platform), fixed in world frame
 *   v_i : second joint axis (proximal link tip), config-dependent
 *   w_i : third  joint axis (moving platform side) = R * w_i0
 *
 * Joint axes defined by  Rz(eta) * Rx(gamma) * [0,0,1].
 *
 * The proximal link tip is parameterised as:
 *   v_i(theta) = [ a1*cos(th) + b1*sin(th) + c1,
 *                  a2*cos(th) + b2*sin(th) + c2,
 *                  a3*cos(th)              + c3 ]
 * where (a1..c3) are precomputed from eta_i1, gamma_i1, alpha_i1.
 *
 * IK per leg – from v_i · w_i = cos(alpha_i2):
 *   A_i*cos(th_i) + B_i*sin(th_i) = C_i
 *   A_i = wx*a1 + wy*a2 + wz*a3
 *   B_i = wx*b1 + wy*b2
 *   C_i = cos(alpha_i2) - (wx*c1 + wy*c2 + wz*c3)
 *
 * Velocity Jacobian – from d/dt(v_i · w_i) = 0:
 *   A_mat * omega = B_mat * theta_dot   =>   omega = J * theta_dot
 *   A_mat[i,:]   = w_i x v_i
 *   B_mat[i,i]   = u_i . (w_i x v_i)
 *   J            = A_mat^{-1} * B_mat
 */

#ifndef SPM_MODEL_H
#define SPM_MODEL_H

#include <math.h>
#include <stdint.h>

#include "mMath.h"
#include "mMatrix.h"

#define SPM_LEGS 3

// ---------------------------------------------------------------------------
// Architecture parameters
// ---------------------------------------------------------------------------

/**
 * @brief Geometric parameters of the 3-RRR SPM.
 *
 * Default values produce the standard Coaxial Input SPM configuration.
 */
struct SPMArch {
    float eta_i1[SPM_LEGS];    ///< Base platform polar angles          (rad)
    float eta_i2[SPM_LEGS];    ///< Moving platform polar angles        (rad)
    float gamma_i1[SPM_LEGS];  ///< Base platform tilt angles           (rad)
    float gamma_i2[SPM_LEGS];  ///< Moving platform tilt angles         (rad)
    float alpha_i1[SPM_LEGS];  ///< Proximal curved-link angles          (rad)
    float alpha_i2[SPM_LEGS];  ///< Distal curved-link angles            (rad)

    SPMArch();  ///< Default Coaxial Input SPM parameters
};

// ---------------------------------------------------------------------------
// Result structures
// ---------------------------------------------------------------------------

/**
 * @brief IK result for all three legs.
 *
 * Each leg may have 0, 1, or 2 solutions in theta[i][0..1].
 */
struct IKResult {
    float theta[SPM_LEGS][2];         ///< Solutions per leg (rad) in [0, 2π)
    bool has_solution[SPM_LEGS];      ///< true if at least one real solution exists
    uint8_t num_solutions[SPM_LEGS];  ///< 0, 1, or 2
};

/**
 * @brief Velocity Jacobian at current configuration.
 *
 *   A_mat * omega = B_mat * theta_dot   =>   omega = J * theta_dot
 *   A_mat[i,:]   = w_i x v_i
 *   B_mat[i,i]   = u_i . (w_i x v_i)   (diagonal)
 */
struct VelJacobian {
    Matrix33f J;      ///< Forward Jacobian: omega = J * theta_dot
    Matrix33f A_mat;  ///< Row i = (w_i x v_i)
    Matrix33f B_mat;  ///< Diagonal: B[i,i] = u_i . (w_i x v_i)
    bool is_valid;    ///< false when A_mat is singular
};

/**
 * @brief FK result: solved w_i vectors and recovered rotation matrix.
 *
 * The solver minimises f(x) = ½‖F(x)‖² where F encodes the 9 constraint
 * equations.  R is recovered from the solved w_i via SVD-based polar
 * decomposition (R = U Vᵀ) to guarantee R ∈ SO(3).
 */
struct FKResult {
    float w[SPM_LEGS][3];  ///< Solved w_i vectors in world frame [leg][x,y,z]
    Matrix33f R;           ///< Recovered rotation matrix R ∈ SO(3)
    bool success;          ///< true if solver converged with cost ≈ 0
    int iterations;        ///< Solver iterations used
    float cost;            ///< Final cost ½‖F(x*)‖²
};

// ---------------------------------------------------------------------------
// SPMModel
// ---------------------------------------------------------------------------

class SPMModel {
   public:
    /**
     * @brief Construct and initialise at neutral pose (R = Identity).
     * @param arch  Architecture parameters (default: Coaxial Input SPM)
     */
    explicit SPMModel(const SPMArch& arch = SPMArch());

    /**
     * @brief Inverse kinematics: compute joint angles for a given rotation.
     *
     * @param R             3x3 rotation matrix (Matrix33f)
     * @param update_config If true and all legs have solutions, updates
     *                      v_i[], w_i[], and theta_i[] using theta_sol_indices[].
     * @return IKResult     All solutions and validity flags per leg.
     */
    IKResult computeIK(const Matrix33f& R, bool update_config = false);

    /**
     * @brief Velocity Jacobian at current configuration.
     *
     * Requires computeIK(update_config=true) to be called first.
     */
    VelJacobian computeVelJacobian() const;

    /**
     * @brief Forward kinematics: given joint angles, recover w_i and R.
     *
     * Solves the 9-equation constraint system using a Gauss-Newton dogleg
     * trust-region method, then recovers R via polar decomposition.
     *
     * Constraint equations (9 total):
     *   v_i · w_i  = cos(alpha_i2)        (alignment,   i = 0,1,2)
     *   w_i · w_j  = w_i0[i] · w_i0[j]   (platform angle, pairs)
     *   ‖w_i‖      = 1                    (unit norm,   i = 0,1,2)
     *
     * @param theta         Joint angles (rad) for each leg
     * @param update_config If true and successful, updates w_i[] in state
     * @param init_prev_sol If true, use current w_i[] as initial guess;
     *                      else use w_i0[] (body-frame vectors) as guess
     * @return FKResult
     */
    FKResult computeFK(const float theta[SPM_LEGS], bool update_config = false,
                       bool init_prev_sol = false);

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------

    float theta_i[SPM_LEGS];          ///< Current joint angles (rad)
    float thetas_neutral[SPM_LEGS];   ///< Neutral pose joint angles (rad)
    int theta_sol_indices[SPM_LEGS];  ///< Active solution branch per leg (0 or 1)
    VelJacobian J_neutral;            ///< Jacobian at neutral pose
    bool is_initialized;              ///< true after successful construction

    /** Test kinematic constraints
     * @brief Verify kinematic constraints
     *
     * Requires computeIK(update_config=true) to be called first.
     */
    bool verifyKinematicConstraints();

    /**
     * @brief Check whether any coaxial input link has surpassed another,
     *        and optionally enforce a minimum angular separation (margin).
     *
     * The three adjusted angles a[i] = wrap(theta[i] + eta_i1[i]) are placed
     * on the circle [0, 2π).  The links are in valid CCW cyclic order 1→2→3
     * iff the three CCW arcs sum to exactly 2π.  Any other sum means at least
     * one link has overtaken another (surpass).  When no surpass is detected,
     * the shortest arc between each adjacent pair is compared against margin.
     *
     * @note Only valid for the coaxial case (all gamma_i1 == 0).
     *       Double-surpass (two simultaneous overtakes) is NOT detected.
     *
     * @param test_thetas  Raw joint angles (rad) for each leg [SPM_LEGS]
     * @param margin       Minimum angular separation to enforce (rad, default 0)
     * @return true  if surpass occurs, or any shortest arc < margin
     * @return false otherwise (or if gamma_i1 != 0 for any leg)
     */
    bool linkSurpassOccur(const float test_thetas[SPM_LEGS], float margin = 0.0f);

   private:
    SPMArch arch_;

    // Precomputed per-leg joint-axis vectors
    mVector3f u_i[SPM_LEGS];   ///< First joint axes (base), fixed in world frame
    mVector3f w_i0[SPM_LEGS];  ///< Third joint axes in moving platform body frame

    // Precomputed proximal-link parameterisation coefficients
    // v_i(theta) = [a1*cos+b1*sin+c1, a2*cos+b2*sin+c2, a3*cos+c3]
    float a1_i[SPM_LEGS], b1_i[SPM_LEGS], c1_i[SPM_LEGS];
    float a2_i[SPM_LEGS], b2_i[SPM_LEGS], c2_i[SPM_LEGS];
    float a3_i[SPM_LEGS], c3_i[SPM_LEGS];

    // Updated by computeIK(update_config=true)
    mVector3f v_i[SPM_LEGS];  ///< Second joint axes (proximal link tip)
    mVector3f w_i[SPM_LEGS];  ///< Third joint axes in world frame = R * w_i0

    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    /** Compute joint axis: Rz(eta)*Rx(gamma)*[0,0,1] */
    static mVector3f computeJointVector(float eta, float gamma);

    /** Compute proximal link tip from joint angle and leg parameters */
    static mVector3f computeVVector(float theta, float a1, float b1, float c1, float a2, float b2,
                                    float c2, float a3, float c3);

    /** Multiply 3x3 matrix by column vector */
    static mVector3f mat3MulVec(const Matrix33f& M, const mVector3f& v);
};

#endif  // SPM_MODEL_H
