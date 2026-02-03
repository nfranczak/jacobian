#include "model.hpp"
#include "urdf_parser.hpp"
#include "forward_kinematics.hpp"
#include "jacobian.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>

using namespace jacobian;

// Numerical Jacobian verification using central differences
Eigen::Matrix<double, 3, 6> computeNumericalJacobian(
    const Model& model,
    const std::array<double, 6>& q,
    double delta = 1e-7)
{
    Eigen::Matrix<double, 3, 6> J_numerical;
    Data data_plus, data_minus;

    for (int i = 0; i < 6; ++i) {
        auto q_plus = q;
        auto q_minus = q;
        q_plus[i] += delta;
        q_minus[i] -= delta;

        computeForwardKinematics(model, q_plus, data_plus);
        computeForwardKinematics(model, q_minus, data_minus);

        Eigen::Vector3d p_plus = data_plus.end_effector_transform.block<3, 1>(0, 3);
        Eigen::Vector3d p_minus = data_minus.end_effector_transform.block<3, 1>(0, 3);

        J_numerical.col(i) = (p_plus - p_minus) / (2.0 * delta);
    }

    return J_numerical;
}

bool testJacobian(const Model& model, const std::array<double, 6>& q, const std::string& test_name) {
    Data data;
    computeJacobian(model, q, data);

    auto J_numerical = computeNumericalJacobian(model, q);
    double error = (data.Jv - J_numerical).norm();

    std::cout << "Test: " << test_name << "\n";
    std::cout << "  q = [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(4) << q[i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]\n";

    std::cout << "  End-effector position: "
              << data.end_effector_transform.block<3, 1>(0, 3).transpose() << "\n";

    std::cout << "  Jacobian error (Frobenius norm): " << std::scientific << error << "\n";

    bool passed = error < 1e-6;
    std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n\n";

    if (!passed) {
        std::cout << "  Analytical Jacobian:\n" << data.Jv << "\n\n";
        std::cout << "  Numerical Jacobian:\n" << J_numerical << "\n\n";
    }

    return passed;
}

int main(int argc, char* argv[]) {
    std::string urdf_path = "urdfs/ur20FK.urdf";
    if (argc > 1) {
        urdf_path = argv[1];
    }

    std::cout << "Loading URDF: " << urdf_path << "\n\n";

    Model model;
    try {
        model = parseURDF(urdf_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse URDF: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Loaded robot: " << model.name << "\n";
    std::cout << "Total joints in chain: " << model.joints.size() << "\n";
    std::cout << "Revolute joints: " << model.num_revolute_joints << "\n\n";

    // Print joint info
    std::cout << "Kinematic chain:\n";
    for (size_t i = 0; i < model.joints.size(); ++i) {
        const auto& j = model.joints[i];
        std::cout << "  " << i << ": " << j.name
                  << " (" << (j.type == JointType::Revolute ? "revolute" : "fixed") << ")"
                  << " " << j.parent_link << " -> " << j.child_link << "\n";
    }
    std::cout << "\n";

    int tests_passed = 0;
    int tests_total = 0;

    // Test 1: Zero configuration
    {
        std::array<double, 6> q = {0, 0, 0, 0, 0, 0};
        if (testJacobian(model, q, "Zero configuration")) tests_passed++;
        tests_total++;
    }

    // Test 2: Small angles
    {
        std::array<double, 6> q = {0.1, 0.2, -0.1, 0.15, -0.05, 0.1};
        if (testJacobian(model, q, "Small angles")) tests_passed++;
        tests_total++;
    }

    // Test 3: Typical working configuration
    {
        std::array<double, 6> q = {0, -M_PI/2, M_PI/2, 0, M_PI/2, 0};
        if (testJacobian(model, q, "Typical configuration")) tests_passed++;
        tests_total++;
    }

    // Test 4: Random configurations
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    for (int t = 0; t < 5; ++t) {
        std::array<double, 6> q;
        for (int i = 0; i < 6; ++i) {
            q[i] = dist(rng);
        }
        if (testJacobian(model, q, "Random configuration " + std::to_string(t + 1))) tests_passed++;
        tests_total++;
    }

    // Summary
    std::cout << "========================================\n";
    std::cout << "Tests passed: " << tests_passed << "/" << tests_total << "\n";

    return (tests_passed == tests_total) ? 0 : 1;
}
