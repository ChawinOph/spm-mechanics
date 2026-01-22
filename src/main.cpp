#include <chrono>
#include <iomanip>
#include <iostream>

#include "kinematics.h"

using namespace kinematics;

void printMatrix(const Matrix3x3& mat, const std::string& name) {
    std::cout << name << ":\n";
    for (int i = 0; i < 3; i++) {
        std::cout << "  [ ";
        for (int j = 0; j < 3; j++) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(6) << mat[i][j];
            if (j < 2) std::cout << ", ";
        }
        std::cout << " ]\n";
    }
}

void printVector(const Vector3& vec, const std::string& name) {
    std::cout << name << ": [ " << std::setw(10) << std::fixed << std::setprecision(6) << vec[0]
              << ", " << std::setw(10) << vec[1] << ", " << std::setw(10) << vec[2] << " ]\n";
}

void demonstrateClass1() {
    std::cout << "\n========================================\n";
    std::cout << "Class I: 3-RRR with Orthogonal MP\n";
    std::cout << "========================================\n\n";

    // Create Agile Eye configuration
    RobotArchitecture arch;
    Class1Kinematics kin(arch);

    // Test case from Table 2, Case 1
    JointAngles theta = {-0.3, -0.5, -0.4};

    std::cout << "Input joint angles (rad):\n";
    std::cout << "  θ1 = " << theta[0] << "\n";
    std::cout << "  θ2 = " << theta[1] << "\n";
    std::cout << "  θ3 = " << theta[2] << "\n\n";

    try {
        auto solutions = kin.solveFK(theta);

        std::cout << "Number of FK solutions found: " << solutions.size() << "\n\n";

        for (size_t i = 0; i < solutions.size(); i++) {
            if (solutions[i].is_valid) {
                std::cout << "Solution " << (i + 1) << ":\n";
                printMatrix(solutions[i].rotation_matrix, "Rotation Matrix");

                // Convert to Euler angles for easier interpretation
                Vector3 euler = utils::matrixToEulerAngles(solutions[i].rotation_matrix);
                std::cout << "Euler angles (ZYZ, rad): [ " << euler[0] << ", " << euler[1] << ", "
                          << euler[2] << " ]\n\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }
}

void demonstrateUtilities() {
    std::cout << "\n========================================\n";
    std::cout << "Utility Functions Test\n";
    std::cout << "========================================\n\n";

    // Test vector operations
    Vector3 v1 = {1.0, 0.0, 0.0};
    Vector3 v2 = {0.0, 1.0, 0.0};

    printVector(v1, "v1");
    printVector(v2, "v2");

    Vector3 cross_product = utils::cross(v1, v2);
    printVector(cross_product, "v1 x v2");

    double dot_product = utils::dot(v1, v2);
    std::cout << "v1 · v2 = " << dot_product << "\n\n";

    // Test quadratic solver
    std::cout << "Solving quadratic: x² - 3x + 2 = 0\n";
    auto quad_sols = utils::solveQuadratic(1.0, -3.0, 2.0);
    std::cout << "Solutions: ";
    for (double sol : quad_sols) {
        std::cout << sol << " ";
    }
    std::cout << "\n(Expected: 1 and 2)\n\n";
}

void demonstratePerformance() {
    std::cout << "\n========================================\n";
    std::cout << "Performance Test\n";
    std::cout << "========================================\n\n";

    RobotArchitecture arch;
    Class1Kinematics kin(arch);

    int num_iterations = 1000;
    JointAngles theta = {-0.3, -0.5, -0.4};

    std::cout << "Running FK " << num_iterations << " times...\n";

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_iterations; i++) {
        auto solutions = kin.solveFK(theta);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    double avg_time = duration.count() / static_cast<double>(num_iterations);

    std::cout << "Average computation time: " << avg_time << " μs\n";
    std::cout << "Total time: " << duration.count() / 1000.0 << " ms\n";
}

int main() {
    std::cout << "======================================\n";
    std::cout << "3-RRR Spherical Parallel Mechanism\n";
    std::cout << "Forward Kinematics Demo\n";
    std::cout << "======================================\n";

    try {
        demonstrateUtilities();
        demonstrateClass1();
        demonstratePerformance();

        std::cout << "\n======================================\n";
        std::cout << "Demo completed successfully!\n";
        std::cout << "======================================\n";

    } catch (const std::exception& e) {
        std::cerr << "\nFatal error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
