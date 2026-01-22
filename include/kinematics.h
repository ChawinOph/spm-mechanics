#ifndef SPHERICAL_KINEMATICS_H
#define SPHERICAL_KINEMATICS_H

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace kinematics {

// Type definitions
using Vector3 = std::array<double, 3>;
using Matrix3x3 = std::array<std::array<double, 3>, 3>;
using JointAngles = std::array<double, 3>;

// Architecture parameters for 3-RRR spherical parallel mechanism
struct RobotArchitecture {
    std::array<double, 3> alpha_i1;  // Proximal link dimensions
    std::array<double, 3> alpha_i2;  // Distal link dimensions
    std::array<double, 3> beta_i1;   // Angles between base joint axes
    std::array<double, 3> beta_i2;   // Angles between platform joint axes

    // Constructor with default Agile Eye parameters
    RobotArchitecture();
};

// Solution structure for FK
struct FKSolution {
    Matrix3x3 rotation_matrix;
    bool is_valid;
    int solution_index;
};

// Class I: Orthogonal MP with 90° distal links
class Class1Kinematics {
   public:
    Class1Kinematics(const RobotArchitecture& arch);

    // Forward kinematics - returns up to 8 solutions
    std::vector<FKSolution> solveFK(const JointAngles& theta);

    // Inverse kinematics
    std::vector<JointAngles> solveIK(const Matrix3x3& rotation);

   private:
    RobotArchitecture arch_;

    // Helper methods for quadratic formulation
    std::vector<double> solveQuadraticFKMethod(const std::array<Vector3, 3>& v_vectors);

    // Helper methods for linear formulation
    double solveLinearFK(const std::array<Vector3, 3>& v_vectors);

    // Compute w vectors from epsilon
    std::array<Vector3, 3> computeWVectors(double epsilon1,
                                           const std::array<Vector3, 3>& v_vectors);
};

// Class II: Coplanar MP with 90° distal links
class Class2Kinematics {
   public:
    Class2Kinematics(const RobotArchitecture& arch);

    std::vector<FKSolution> solveFK(const JointAngles& theta);
    std::vector<JointAngles> solveIK(const Matrix3x3& rotation);

   private:
    RobotArchitecture arch_;
    std::array<double, 3> m_coefficients_;  // For coplanarity constraint

    // Solve quartic equation for epsilon1
    std::vector<double> solveQuarticFK(const std::array<Vector3, 3>& v_vectors);
};

// Class III: Coaxial input axes with 90° proximal and distal links
class Class3Kinematics {
   public:
    Class3Kinematics(const RobotArchitecture& arch);

    std::vector<FKSolution> solveFK(const JointAngles& theta);
    std::vector<JointAngles> solveIK(const Matrix3x3& rotation);

   private:
    RobotArchitecture arch_;
    std::array<double, 3> n_coefficients_;  // For w3 decomposition

    // Solve quartic equation in cos(epsilon1)
    std::vector<double> solveQuarticFK(const std::array<Vector3, 3>& v_vectors,
                                       const JointAngles& theta);
};

// Utility functions
namespace utils {
// Vector operations
Vector3 cross(const Vector3& a, const Vector3& b);
double dot(const Vector3& a, const Vector3& b);
double norm(const Vector3& v);
Vector3 normalize(const Vector3& v);

// Matrix operations
Matrix3x3 rotationMatrix(const Vector3& w1, const Vector3& w2, const Vector3& w3);
Vector3 matrixToEulerAngles(const Matrix3x3& R);
Matrix3x3 eulerAnglesToMatrix(const Vector3& euler);

// Polynomial solvers
std::vector<double> solveQuadratic(double a, double b, double c);
std::vector<double> solveQuartic(double a, double b, double c, double d, double e);

// Numerical comparisons
bool isNearZero(double value, double tolerance = 1e-10);
bool areEqual(double a, double b, double tolerance = 1e-10);
}  // namespace utils

}  // namespace kinematics

#endif  // SPHERICAL_KINEMATICS_H
