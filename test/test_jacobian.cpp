#include "model.hpp"
#include "urdf_parser.hpp"
#include "forward_kinematics.hpp"
#include "jacobian.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace jacobian;

// Numerical Jacobian verification using central differences
Eigen::MatrixXd computeNumericalJacobian(
    const Model& model,
    const Eigen::VectorXd& q,
    double delta = 1e-7)
{
    const size_t n = model.num_revolute_joints;
    Eigen::MatrixXd J_numerical(3, n);
    Data data_plus(model), data_minus(model);

    for (size_t i = 0; i < n; ++i) {
        Eigen::VectorXd q_plus = q;
        Eigen::VectorXd q_minus = q;
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

bool testJacobian(const Model& model, const Eigen::VectorXd& q, const std::string& test_name) {
    Data data(model);
    computeJacobian(model, q, data);

    auto J_numerical = computeNumericalJacobian(model, q);
    double error = (data.Jv - J_numerical).norm();

    std::cout << "Test: " << test_name << "\n";
    std::cout << "  q = [";
    for (int i = 0; i < q.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << q[i];
        if (i < q.size() - 1) std::cout << ", ";
    }
    std::cout << "]\n";

    std::cout << "  End-effector position: "
              << data.end_effector_transform.block<3, 1>(0, 3).transpose() << "\n";

    std::cout << "  Jacobian error (Frobenius norm): " << std::scientific << error << "\n";

    bool passed = error < 1e-6;
    std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n";

    if (!passed) {
        std::cout << "  Analytical Jacobian:\n" << data.Jv << "\n";
        std::cout << "  Numerical Jacobian:\n" << J_numerical << "\n";
    }
    std::cout << "\n";

    return passed;
}

int main() {
    Model ur20_fk = parseURDF("urdfs/ur20FK.urdf");
    Model ur20_collapsed = parseURDF("urdfs/ur20Collapsed.urdf");
    Model twolink = parseURDF("urdfs/2link.urdf");

    int tests_passed = 0;
    int tests_total = 0;

    // --- UR20 tests (6-DOF) ---

    // Zero configuration
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
        if (testJacobian(ur20_fk, q, "UR20 FK - Zero config")) tests_passed++;
        tests_total++;
    }
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
        if (testJacobian(ur20_collapsed, q, "UR20 Collapsed - Zero config")) tests_passed++;
        tests_total++;
    }

    // Small angles
    {
        Eigen::VectorXd q(6);
        q << 0.1, 0.2, -0.1, 0.15, -0.05, 0.1;
        if (testJacobian(ur20_fk, q, "UR20 FK - Small angles")) tests_passed++;
        tests_total++;
    }
    {
        Eigen::VectorXd q(6);
        q << 0.1, 0.2, -0.1, 0.15, -0.05, 0.1;
        if (testJacobian(ur20_collapsed, q, "UR20 Collapsed - Small angles")) tests_passed++;
        tests_total++;
    }

    // Typical working configuration
    {
        Eigen::VectorXd q(6);
        q << 0, -M_PI/2, M_PI/2, 0, M_PI/2, 0;
        if (testJacobian(ur20_fk, q, "UR20 FK - Typical config")) tests_passed++;
        tests_total++;
    }

    // --- 2-link velocity tests: check v = J(q) * q_dot against expected values ---

    // q=[0,0], q_dot=[1,0]: base spinning at 1 rad/s, EE is 2m away along X
    // EE traces a circle of radius 2 about Z → v = [0, 2, 0]
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd q_dot(2);
        q_dot << 1, 0;

        Data data(twolink);
        computeJacobian(twolink, q, data);
        Eigen::Vector3d v = data.Jv * q_dot;
        Eigen::Vector3d v_expected(0, 2, 0);

        double error = (v - v_expected).norm();
        bool passed = error < 1e-10;
        std::cout << "Test: 2-link velocity - q=[0,0], q_dot=[1,0]\n";
        std::cout << "  Expected v: " << v_expected.transpose() << "\n";
        std::cout << "  Got v:      " << v.transpose() << "\n";
        std::cout << "  Error: " << std::scientific << error << "\n";
        std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n\n";
        if (passed) tests_passed++;
        tests_total++;
    }

    // q=[pi/2, 0], q_dot=[1,0]: base spinning at 1 rad/s, arm points along +Y
    // EE is at (0,2,0), sweeps in -X direction → v = [-2, 0, 0]
    {
        Eigen::VectorXd q(2);
        q << M_PI / 2, 0;
        Eigen::VectorXd q_dot(2);
        q_dot << 1, 0;

        Data data(twolink);
        computeJacobian(twolink, q, data);
        Eigen::Vector3d v = data.Jv * q_dot;
        Eigen::Vector3d v_expected(-2, 0, 0);

        double error = (v - v_expected).norm();
        bool passed = error < 1e-10;
        std::cout << "Test: 2-link velocity - q=[pi/2,0], q_dot=[1,0]\n";
        std::cout << "  Expected v: " << v_expected.transpose() << "\n";
        std::cout << "  Got v:      " << v.transpose() << "\n";
        std::cout << "  Error: " << std::scientific << error << "\n";
        std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n\n";
        if (passed) tests_passed++;
        tests_total++;
    }

    // q=[0, pi/2], q_dot=[1,0]: base spinning at 1 rad/s, link2 bent 90deg up
    // EE at (1,1,0), radius from Z axis is sqrt(2) but velocity is cross product:
    // J1 col = [0,0,1] x [1,1,0] = [-1, 1, 0] → v = [-1, 1, 0]
    {
        Eigen::VectorXd q(2);
        q << 0, M_PI / 2;
        Eigen::VectorXd q_dot(2);
        q_dot << 1, 0;

        Data data(twolink);
        computeJacobian(twolink, q, data);
        Eigen::Vector3d v = data.Jv * q_dot;
        Eigen::Vector3d v_expected(-1, 1, 0);

        double error = (v - v_expected).norm();
        bool passed = error < 1e-10;
        std::cout << "Test: 2-link velocity - q=[0,pi/2], q_dot=[1,0]\n";
        std::cout << "  Expected v: " << v_expected.transpose() << "\n";
        std::cout << "  Got v:      " << v.transpose() << "\n";
        std::cout << "  Error: " << std::scientific << error << "\n";
        std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n\n";
        if (passed) tests_passed++;
        tests_total++;
    }

    // --- 2-link ground-truth tests (hand-computed Jacobian values) ---

    // q=[0,0]: links along X, EE at (2,0,0)
    //   J1 at origin, axis Z: [0,0,1] x [2,0,0] = [0, 2, 0]
    //   J2 at (1,0,0), axis Z: [0,0,1] x [1,0,0] = [0, 1, 0]
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
        Data data(twolink);
        computeJacobian(twolink, q, data);

        Eigen::MatrixXd J_expected(3, 2);
        J_expected << 0, 0,
                      2, 1,
                      0, 0;

        double error = (data.Jv - J_expected).norm();
        bool passed = error < 1e-10;
        std::cout << "Test: 2-link ground truth - q=[0,0]\n";
        std::cout << "  EE position: " << data.end_effector_transform.block<3,1>(0,3).transpose() << "\n";
        std::cout << "  Expected Jv:\n" << J_expected << "\n";
        std::cout << "  Got Jv:\n" << data.Jv << "\n";
        std::cout << "  Error: " << std::scientific << error << "\n";
        std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n\n";
        if (passed) tests_passed++;
        tests_total++;
    }

    // q=[pi/2, 0]: link1 along Y, link2 along Y, EE at (0,2,0)
    //   J1 at origin, axis Z: [0,0,1] x [0,2,0] = [-2, 0, 0]
    //   J2 at (0,1,0), axis Z: [0,0,1] x [0,1,0] = [-1, 0, 0]
    {
        Eigen::VectorXd q(2);
        q << M_PI / 2, 0;
        Data data(twolink);
        computeJacobian(twolink, q, data);

        Eigen::MatrixXd J_expected(3, 2);
        J_expected << -2, -1,
                       0,  0,
                       0,  0;

        double error = (data.Jv - J_expected).norm();
        bool passed = error < 1e-10;
        std::cout << "Test: 2-link ground truth - q=[pi/2, 0]\n";
        std::cout << "  EE position: " << data.end_effector_transform.block<3,1>(0,3).transpose() << "\n";
        std::cout << "  Expected Jv:\n" << J_expected << "\n";
        std::cout << "  Got Jv:\n" << data.Jv << "\n";
        std::cout << "  Error: " << std::scientific << error << "\n";
        std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n\n";
        if (passed) tests_passed++;
        tests_total++;
    }

    // q=[0, pi/2]: link1 along X, link2 along Y, EE at (1,1,0)
    //   J1 at origin, axis Z: [0,0,1] x [1,1,0] = [-1, 1, 0]
    //   J2 at (1,0,0), axis Z: [0,0,1] x [0,1,0] = [-1, 0, 0]
    {
        Eigen::VectorXd q(2);
        q << 0, M_PI / 2;
        Data data(twolink);
        computeJacobian(twolink, q, data);

        Eigen::MatrixXd J_expected(3, 2);
        J_expected << -1, -1,
                       1,  0,
                       0,  0;

        double error = (data.Jv - J_expected).norm();
        bool passed = error < 1e-10;
        std::cout << "Test: 2-link ground truth - q=[0, pi/2]\n";
        std::cout << "  EE position: " << data.end_effector_transform.block<3,1>(0,3).transpose() << "\n";
        std::cout << "  Expected Jv:\n" << J_expected << "\n";
        std::cout << "  Got Jv:\n" << data.Jv << "\n";
        std::cout << "  Error: " << std::scientific << error << "\n";
        std::cout << "  Result: " << (passed ? "PASSED" : "FAILED") << "\n\n";
        if (passed) tests_passed++;
        tests_total++;
    }

    // --- 2-link numerical consistency tests (2-DOF) ---

    // Zero configuration
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(2);
        if (testJacobian(twolink, q, "2-link - Zero config")) tests_passed++;
        tests_total++;
    }

    // 90 degrees on joint 1: EE at (0, 2, 0) approximately
    {
        Eigen::VectorXd q(2);
        q << M_PI / 2, 0;
        if (testJacobian(twolink, q, "2-link - q1=90deg")) tests_passed++;
        tests_total++;
    }

    // Both joints at 45 degrees
    {
        Eigen::VectorXd q(2);
        q << M_PI / 4, M_PI / 4;
        if (testJacobian(twolink, q, "2-link - q1=q2=45deg")) tests_passed++;
        tests_total++;
    }

    // Folded back: joint2 at 180 degrees, EE at origin
    {
        Eigen::VectorXd q(2);
        q << 0, M_PI;
        if (testJacobian(twolink, q, "2-link - Folded back")) tests_passed++;
        tests_total++;
    }

    // Summary
    std::cout << "========================================\n";
    std::cout << "Tests passed: " << tests_passed << "/" << tests_total << "\n";

    return (tests_passed == tests_total) ? 0 : 1;
}
