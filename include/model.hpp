#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>

namespace jacobian {

enum class JointType {
    Fixed,
    Revolute
};

struct Joint {
    std::string name;
    JointType type;
    std::string parent_link;
    std::string child_link;

    // URDF origin parameters
    Eigen::Vector3d origin_xyz;
    Eigen::Vector3d origin_rpy;  // roll, pitch, yaw

    // Rotation axis in local frame (for revolute joints)
    Eigen::Vector3d axis;

    // Precomputed static transform from origin_xyz + origin_rpy
    Eigen::Matrix4d placement;

    Joint()
        : type(JointType::Fixed)
        , origin_xyz(Eigen::Vector3d::Zero())
        , origin_rpy(Eigen::Vector3d::Zero())
        , axis(Eigen::Vector3d::UnitZ())
        , placement(Eigen::Matrix4d::Identity())
    {}
};

struct Model {
    std::string name;

    // All joints in kinematic chain order (fixed + revolute)
    std::vector<Joint> joints;

    // Indices of the 6 revolute joints within joints[]
    std::array<size_t, 6> revolute_joint_indices;

    // Number of revolute joints found (should be 6 for valid model)
    size_t num_revolute_joints = 0;

    Model() {
        revolute_joint_indices.fill(0);
    }
};

struct Data {
    // Transform from base frame to each revolute joint frame (before rotation applied)
    // These are used to extract z_i (rotation axis) and p_i (joint position) for Jacobian
    std::array<Eigen::Matrix4d, 6> joint_transforms;

    // Transform from base frame to end-effector
    Eigen::Matrix4d end_effector_transform;

    // Linear velocity Jacobian (3x6)
    Eigen::Matrix<double, 3, 6> Jv;

    Data() {
        for (auto& T : joint_transforms) {
            T.setIdentity();
        }
        end_effector_transform.setIdentity();
        Jv.setZero();
    }
};

}  // namespace jacobian
