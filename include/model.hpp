#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
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

    // Rotation axis in local frame (for revolute joints)
    Eigen::Vector3d axis;

    // Precomputed static transform from URDF origin xyz + rpy
    Eigen::Matrix4d placement;

    Joint()
        : type(JointType::Fixed)
        , axis(Eigen::Vector3d::UnitZ())
        , placement(Eigen::Matrix4d::Identity())
    {}
};

struct Model {
    std::string name;

    // All joints in kinematic chain order (fixed + revolute)
    std::vector<Joint> joints;

    // Indices of revolute joints within joints[]
    std::vector<size_t> revolute_joint_indices;
};

struct Data {
    // Transform from base frame to each revolute joint frame (before rotation applied)
    // These are used to extract z_i (rotation axis) and p_i (joint position) for Jacobian
    std::vector<Eigen::Matrix4d> joint_transforms;

    // Transform from base frame to end-effector
    Eigen::Matrix4d end_effector_transform;

    // Full Jacobian (6 x num_revolute_joints): [linear velocity; angular velocity]
    Eigen::MatrixXd J;

    // Initialize data storage sized for the given model
    explicit Data(const Model& model)
        : joint_transforms(model.revolute_joint_indices.size(), Eigen::Matrix4d::Identity())
        , end_effector_transform(Eigen::Matrix4d::Identity())
        , J(Eigen::MatrixXd::Zero(6, model.revolute_joint_indices.size()))
    {}
};

}  // namespace jacobian
