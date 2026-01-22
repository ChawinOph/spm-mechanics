#include <cassert>
#include <cmath>
#include <iostream>

#include "kinematics.h"

using namespace kinematics;

bool testVectorOperations() {
    std::cout << "Testing vector operations...\n";

    Vector3 v1 = {1.0, 0.0, 0.0};
    Vector3 v2 = {0.0, 1.0, 0.0};

    // Test cross product
    Vector3 cross = utils::cross(v1, v2);
    assert(utils::areEqual(cross[0], 0.0));
    assert(utils::areEqual(cross[1], 0.0));
    assert(utils::areEqual(cross[2], 1.0));

    // Test dot product
    double dot = utils::dot(v1, v2);
    assert(utils::areEqual(dot, 0.0));

    // Test norm
    double norm = utils::norm(v1);
    assert(utils::areEqual(norm, 1.0));

    std::cout << "  ✓ Vector operations passed\n";
    return true;
}

bool testQuadraticSolver() {
    std::cout << "Testing quadratic solver...\n";

    // Test: x² - 3x + 2 = 0, solutions: x = 1, 2
    auto solutions = utils::solveQuadratic(1.0, -3.0, 2.0);

    assert(solutions.size() == 2);
    assert(utils::areEqual(solutions[0], 2.0) || utils::areEqual(solutions[0], 1.0));
    assert(utils::areEqual(solutions[1], 2.0) || utils::areEqual(solutions[1], 1.0));

    std::cout << "  ✓ Quadratic solver passed\n";
    return true;
}

bool testClass1Construction() {
    std::cout << "Testing Class I construction...\n";

    try {
        RobotArchitecture arch;
        Class1Kinematics kin(arch);
        std::cout << "  ✓ Class I construction passed\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "  ✗ Class I construction failed: " << e.what() << "\n";
        return false;
    }
}

bool testClass1ForwardKinematics() {
    std::cout << "Testing Class I forward kinematics...\n";

    // user defined test cases of different joint angles
    std::vector<JointAngles> test_cases = {{0.0, 0.0, 0.0},
                                           {M_PI / 6, M_PI / 6, M_PI / 6},
                                           {-M_PI / 4, M_PI / 4, -M_PI / 4},
                                           {0.1, 0.2, -0.1}};

    // TODO: Add expected solutions for verification
    // Randomized rotational matrices, run IK and use FK to verify consistency

    try {
        RobotArchitecture arch;
        Class1Kinematics kin(arch);

        bool all_passed = true;
        int case_num = 0;

        // Test case from paper
        for (const auto& theta : test_cases) {
            case_num++;
            std::cout << " case " << case_num << " Testing FK for joint angles: [" << theta[0]
                      << ", " << theta[1] << ", " << theta[2] << "]\n";

            auto solutions = kin.solveFK(theta);
            std::cout << " Found " << solutions.size() << " solutions";

            // Class I should find up to 8 solutions
            // For now, just check that we get some valid solutions

            bool hasValidSolution = false;

            for (const auto& sol : solutions) {
                if (sol.is_valid) {
                    hasValidSolution = true;
                    break;
                }
            }

            if (hasValidSolution) {
                std::cout << "  ✓ case " << case_num << " passed\n";
            } else {
                std::cout << "  ✗ case " << case_num << " failed: no valid solutions\n";
                all_passed = false;
            }
        }

        if (all_passed) {
            std::cout << "  ✓ Class I FK passed\n";
            return true;
        } else {
            std::cout << "  ✗ Class I FK failed: no valid solutions\n";
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "  ✗ Class I FK failed: " << e.what() << "\n";
        return false;
    }
}

int main() {
    std::cout << "\n================================\n";
    std::cout << "Running Kinematics Tests\n";
    std::cout << "================================\n\n";

    int passed = 0;
    int total = 0;

    total++;
    if (testVectorOperations()) passed++;
    total++;
    if (testQuadraticSolver()) passed++;
    total++;
    if (testClass1Construction()) passed++;
    total++;
    if (testClass1ForwardKinematics()) passed++;

    std::cout << "\n================================\n";
    std::cout << "Results: " << passed << "/" << total << " tests passed\n";
    std::cout << "================================\n\n";

    return (passed == total) ? 0 : 1;
}
