#pragma once

#include "model.hpp"
#include <array>

namespace jacobian {

// RPY (roll-pitch-yaw) to rotation matrix
// URDF uses fixed-axis XYZ convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw);

// Create 4x4 homogeneous transform from rotation matrix and translation
Eigen::Matrix4d createTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

// Rotation about arbitrary axis using Rodrigues' formula
Eigen::Matrix3d axisAngleRotation(const Eigen::Vector3d& axis, double angle);

// Compute forward kinematics
// Writes joint_transforms and end_effector_transform into data
void computeForwardKinematics(
    const Model& model,
    const std::array<double, 6>& q,
    Data& data);

}  // namespace jacobian
