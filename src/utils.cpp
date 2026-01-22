#include "kinematics.h"
#include <cmath>
#include <algorithm>
#include <complex>

namespace kinematics {
namespace utils {

Vector3 cross(const Vector3& a, const Vector3& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

double dot(const Vector3& a, const Vector3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const Vector3& v) {
    return std::sqrt(dot(v, v));
}

Vector3 normalize(const Vector3& v) {
    double n = norm(v);
    if (isNearZero(n)) {
        return {0.0, 0.0, 0.0};
    }
    return {v[0] / n, v[1] / n, v[2] / n};
}

Matrix3x3 rotationMatrix(const Vector3& w1, const Vector3& w2, const Vector3& w3) {
    // Construct rotation matrix from three orthonormal vectors
    return {{{w1[0], w2[0], w3[0]},
             {w1[1], w2[1], w3[1]},
             {w1[2], w2[2], w3[2]}}};
}

Vector3 matrixToEulerAngles(const Matrix3x3& R) {
    // Convert rotation matrix to ZYZ Euler angles
    double theta = std::acos(R[2][2]);
    double psi, phi;
    
    if (std::abs(std::sin(theta)) > 1e-10) {
        psi = std::atan2(R[2][1], R[2][0]);
        phi = std::atan2(R[1][2], -R[0][2]);
    } else {
        psi = 0.0;
        phi = std::atan2(-R[0][1], R[0][0]);
    }
    
    return {psi, theta, phi};
}

Matrix3x3 eulerAnglesToMatrix(const Vector3& euler) {
    // Convert ZYZ Euler angles to rotation matrix
    double psi = euler[0], theta = euler[1], phi = euler[2];
    
    double c1 = std::cos(psi), s1 = std::sin(psi);
    double c2 = std::cos(theta), s2 = std::sin(theta);
    double c3 = std::cos(phi), s3 = std::sin(phi);
    
    return {{{c1*c2*c3 - s1*s3, -c1*c2*s3 - s1*c3, c1*s2},
             {s1*c2*c3 + c1*s3, -s1*c2*s3 + c1*c3, s1*s2},
             {-s2*c3,           s2*s3,             c2}}};
}

std::vector<double> solveQuadratic(double a, double b, double c) {
    std::vector<double> solutions;
    
    if (isNearZero(a)) {
        // Linear equation
        if (!isNearZero(b)) {
            solutions.push_back(-c / b);
        }
        return solutions;
    }
    
    double discriminant = b * b - 4 * a * c;
    
    if (discriminant < -1e-10) {
        return solutions;  // No real solutions
    }
    
    if (isNearZero(discriminant)) {
        solutions.push_back(-b / (2 * a));
    } else {
        double sqrt_disc = std::sqrt(discriminant);
        solutions.push_back((-b + sqrt_disc) / (2 * a));
        solutions.push_back((-b - sqrt_disc) / (2 * a));
    }
    
    return solutions;
}

std::vector<double> solveQuartic(double a, double b, double c, double d, double e) {
    // Ferrari's formula for quartic equation: ax^4 + bx^3 + cx^2 + dx + e = 0
    std::vector<double> solutions;
    
    if (isNearZero(a)) {
        // Reduce to cubic or lower
        return {};  // Implement cubic solver if needed
    }
    
    // Normalize coefficients
    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;
    
    // Solve resolvent cubic: y^3 + b2*y^2 + b1*y + b0 = 0
    double b2 = -a2;
    double b1 = a1 * a3 - 4 * a0;
    double b0 = 4 * a0 * a2 - a1 * a1 - a0 * a3 * a3;
    
    double p = b1 / 3.0 - b2 * b2 / 9.0;
    double q = b0 / 2.0 - b1 * b2 / 6.0 + b2 * b2 * b2 / 27.0;
    
    double delta = q * q + p * p * p;
    
    double y1;
    if (delta > 1e-10) {
        double sqrt_delta = std::sqrt(delta);
        y1 = std::cbrt(-q + sqrt_delta) + std::cbrt(-q - sqrt_delta) - b2 / 3.0;
    } else if (isNearZero(delta)) {
        y1 = -2.0 * std::cbrt(q) - b2 / 3.0;
    } else {
        double r = std::sqrt(-p * p * p);
        double alpha = std::acos(-q / r) / 3.0;
        y1 = 2.0 * std::cbrt(r) * std::cos(alpha) - b2 / 3.0;
    }
    
    // Compute R, D, E
    double R = std::sqrt(a3 * a3 / 4.0 - a2 + y1);
    
    double D, E;
    if (isNearZero(R)) {
        double temp = 3.0 * a3 * a3 / 4.0 - 2.0 * a2;
        double sqrt_term = std::sqrt(y1 * y1 - 4.0 * a0);
        D = std::sqrt(temp + 2.0 * sqrt_term);
        E = std::sqrt(temp - 2.0 * sqrt_term);
    } else {
        double temp = 3.0 * a3 * a3 / 4.0 - R * R - 2.0 * a2;
        double frac = (4.0 * a3 * a2 - 8.0 * a1 - a3 * a3 * a3) / (4.0 * R);
        D = std::sqrt(temp + frac);
        E = std::sqrt(temp - frac);
    }
    
    // Compute four solutions
    std::vector<std::complex<double>> complex_sols = {
        {-a3/4.0 + R/2.0 + D/2.0, 0.0},
        {-a3/4.0 + R/2.0 - D/2.0, 0.0},
        {-a3/4.0 - R/2.0 + E/2.0, 0.0},
        {-a3/4.0 - R/2.0 - E/2.0, 0.0}
    };
    
    // Extract real solutions
    for (const auto& sol : complex_sols) {
        if (std::abs(sol.imag()) < 1e-10) {
            solutions.push_back(sol.real());
        }
    }
    
    return solutions;
}

bool isNearZero(double value, double tolerance) {
    return std::abs(value) < tolerance;
}

bool areEqual(double a, double b, double tolerance) {
    return isNearZero(a - b, tolerance);
}

} // namespace utils
} // namespace kinematics
