#include <cmath>
#include <stdexcept>

#include "kinematics.h"

namespace kinematics {

RobotArchitecture::RobotArchitecture() {
    // Default Agile Eye parameters (all 90 degrees)
    alpha_i1 = {M_PI / 2, M_PI / 2, M_PI / 2};
    alpha_i2 = {M_PI / 2, M_PI / 2, M_PI / 2};
    beta_i1 = {M_PI / 2, M_PI / 2, M_PI / 2};
    beta_i2 = {M_PI / 2, M_PI / 2, M_PI / 2};
}

Class1Kinematics::Class1Kinematics(const RobotArchitecture& arch) : arch_(arch) {
    // Verify Class I constraints: orthogonal MP and 90° distal links
    for (int i = 0; i < 3; i++) {
        if (!utils::areEqual(arch.alpha_i2[i], M_PI / 2, 1e-6)) {
            throw std::invalid_argument("Class I requires 90° distal links");
        }
        if (!utils::areEqual(arch.beta_i2[i], M_PI / 2, 1e-6)) {
            throw std::invalid_argument("Class I requires orthogonal MP");
        }
    }
}

std::vector<FKSolution> Class1Kinematics::solveFK(const JointAngles& theta) {
    std::vector<FKSolution> solutions;

    // Compute v vectors (intermediate joint axis directions)
    std::array<Vector3, 3> v_vectors;

    // Assuming base frame with z-axis pointing up
    // and y-axis in the plane of z and first input axis
    for (int i = 0; i < 3; i++) {
        double angle = theta[i];
        // TODO: BUG - Beta angle indexing is incorrect. Only uses beta_i1[0] and beta_i1[1],
        // never uses beta_i1[2]. Should properly distribute all three base joint orientations.
        // Suggested fix: Use cumulative angles or proper indexing scheme.
        double beta = (i == 0) ? 0 : arch_.beta_i1[i - 1];

        // Compute v_i direction based on input angle and proximal link
        double cos_alpha = std::cos(arch_.alpha_i1[i]);
        double sin_alpha = std::sin(arch_.alpha_i1[i]);
        double cos_theta = std::cos(angle);
        double sin_theta = std::sin(angle);
        double cos_beta = std::cos(beta);
        double sin_beta = std::sin(beta);

        // TODO: CRITICAL BUG - This formula produces zero vectors when theta={0,0,0}!
        // When theta[i]=0 and alpha_i1[i]=π/2: sin_theta=0, cos_alpha=0, sin_alpha=1
        // Result: v_vectors[i] = {0, 0, 0} which is invalid and causes FK to fail.
        // The formula doesn't correctly implement spherical rotation kinematics.
        // Should use proper rotation matrices or Rodrigues' formula to transform
        // the default intermediate joint axis by rotation theta[i] around input axis.
        v_vectors[i] = {sin_alpha * sin_theta * cos_beta, sin_alpha * sin_theta * sin_beta,
                        cos_alpha};
    }

    // Method 1: Quadratic formulation (Equation 10 from paper)
    try {
        std::vector<double> epsilon1_solutions = solveQuadraticFKMethod(v_vectors);

        for (double epsilon1 : epsilon1_solutions) {
            // Compute w vectors for each epsilon1
            auto w_vectors = computeWVectors(epsilon1, v_vectors);

            // Create rotation matrix
            Matrix3x3 R = utils::rotationMatrix(w_vectors[0], w_vectors[1], w_vectors[2]);

            FKSolution sol;
            sol.rotation_matrix = R;
            sol.is_valid = true;
            sol.solution_index = solutions.size();
            solutions.push_back(sol);
        }
    } catch (const std::exception& e) {
        // TODO: NOTE - Exception is silently caught here. Due to v_vector bugs above,
        // this catches "Singular configuration: v1 parallel to v2" error and returns
        // empty solutions. Fix v_vector computation to resolve this issue.
        // If quadratic method fails, solutions remain empty
    }

    return solutions;
}

std::vector<double> Class1Kinematics::solveQuadraticFKMethod(
    const std::array<Vector3, 3>& v_vectors) {
    // Choose a1 parallel to v1 × v2 (Equation 6)
    Vector3 v1_cross_v2 = utils::cross(v_vectors[0], v_vectors[1]);
    double norm_cross = utils::norm(v1_cross_v2);

    if (utils::isNearZero(norm_cross)) {
        throw std::runtime_error("Singular configuration: v1 parallel to v2");
    }

    Vector3 a1 = utils::normalize(v1_cross_v2);
    Vector3 b1 = utils::cross(v_vectors[0], a1);

    // Formulate quadratic equation (Equation 10)
    // (v1^T v2)(v1^T v3) tan²ε1 + (v1×v2)^T v3 tan ε1 + v2^T v3 = 0

    double v1_dot_v2 = utils::dot(v_vectors[0], v_vectors[1]);
    double v1_dot_v3 = utils::dot(v_vectors[0], v_vectors[2]);
    double v2_dot_v3 = utils::dot(v_vectors[1], v_vectors[2]);
    Vector3 v1_cross_v2_vec = utils::cross(v_vectors[0], v_vectors[1]);
    double cross_dot_v3 = utils::dot(v1_cross_v2_vec, v_vectors[2]);

    double coeff_a = v1_dot_v2 * v1_dot_v3;
    double coeff_b = cross_dot_v3;
    double coeff_c = v2_dot_v3;

    // Solve quadratic in tan(epsilon1)
    auto tan_solutions = utils::solveQuadratic(coeff_a, coeff_b, coeff_c);

    // Convert tan(epsilon1) to epsilon1
    std::vector<double> epsilon1_solutions;
    for (double tan_eps1 : tan_solutions) {
        double eps1 = std::atan(tan_eps1);
        epsilon1_solutions.push_back(eps1);
        epsilon1_solutions.push_back(eps1 + M_PI);  // Add periodic solution
    }

    return epsilon1_solutions;
}

double Class1Kinematics::solveLinearFK(const std::array<Vector3, 3>& v_vectors) {
    // Linear formulation (Equation 17)
    // Choose a13 = (a11 + a12) / ||a11 + a12||

    Vector3 a11 = utils::normalize(utils::cross(v_vectors[0], v_vectors[1]));
    Vector3 a12 = utils::normalize(utils::cross(v_vectors[0], v_vectors[2]));

    Vector3 a13_unnorm = {a11[0] + a12[0], a11[1] + a12[1], a11[2] + a12[2]};
    Vector3 a13 = utils::normalize(a13_unnorm);
    Vector3 b13 = utils::cross(v_vectors[0], a13);

    // Compute cos(2ε1'') according to Equation 17
    double v2_dot_v3 = utils::dot(v_vectors[1], v_vectors[2]);
    double v1_dot_v2 = utils::dot(v_vectors[0], v_vectors[1]);
    double v1_dot_v3 = utils::dot(v_vectors[0], v_vectors[2]);

    Vector3 v1_cross_v2 = utils::cross(v_vectors[0], v_vectors[1]);
    Vector3 v1_cross_v3 = utils::cross(v_vectors[0], v_vectors[2]);
    double norm_v1_cross_v2 = utils::norm(v1_cross_v2);
    double norm_v1_cross_v3 = utils::norm(v1_cross_v3);

    double numerator = v2_dot_v3 + v1_dot_v2 * v1_dot_v3;
    double denominator = -norm_v1_cross_v2 * norm_v1_cross_v3;

    double cos_2eps = numerator / denominator;

    // Solve for epsilon1''
    double phi1 = std::acos(cos_2eps);  // phi1 ∈ [0, π]

    // Four solutions: ±φ1/2 + kπ, k=0,1
    return phi1 / 2.0;  // Return one solution for simplicity
}

std::array<Vector3, 3> Class1Kinematics::computeWVectors(double epsilon1,
                                                         const std::array<Vector3, 3>& v_vectors) {
    std::array<Vector3, 3> w_vectors;

    // Compute a1 and b1
    Vector3 v1_cross_v2 = utils::cross(v_vectors[0], v_vectors[1]);
    Vector3 a1 = utils::normalize(v1_cross_v2);
    Vector3 b1 = utils::cross(v_vectors[0], a1);

    // Compute w1 from Equation 4
    double cos_eps1 = std::cos(epsilon1);
    double sin_eps1 = std::sin(epsilon1);

    w_vectors[0] = {cos_eps1 * a1[0] + sin_eps1 * b1[0], cos_eps1 * a1[1] + sin_eps1 * b1[1],
                    cos_eps1 * a1[2] + sin_eps1 * b1[2]};

    // Compute w2 from Equation 31 (two possibilities)
    Vector3 v2_cross_w1 = utils::cross(v_vectors[1], w_vectors[0]);
    double norm_v2_cross_w1 = utils::norm(v2_cross_w1);

    if (!utils::isNearZero(norm_v2_cross_w1)) {
        w_vectors[1] = utils::normalize(v2_cross_w1);
    } else {
        throw std::runtime_error("Singular configuration in w2 computation");
    }

    // Compute w3 = w1 × w2
    w_vectors[2] = utils::cross(w_vectors[0], w_vectors[1]);

    return w_vectors;
}

std::vector<JointAngles> Class1Kinematics::solveIK(const Matrix3x3& rotation) {
    std::vector<JointAngles> solutions;

    // Extract w vectors from rotation matrix (columns)
    std::array<Vector3, 3> w_vectors = {{{rotation[0][0], rotation[1][0], rotation[2][0]},
                                         {rotation[0][1], rotation[1][1], rotation[2][1]},
                                         {rotation[0][2], rotation[1][2], rotation[2][2]}}};

    // For each limb, solve for theta_i
    // This requires iterating through possible configurations
    // Implementation depends on specific inverse kinematics approach

    // Placeholder: implement full IK solver
    // The IK is generally simpler than FK for parallel mechanisms

    return solutions;
}

}  // namespace kinematics
