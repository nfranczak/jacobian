#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace jacobian {
namespace detail {

// RPY (roll-pitch-yaw) to rotation matrix
// URDF uses fixed-axis XYZ convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw);

// Create 4x4 homogeneous transform from rotation matrix and translation
Eigen::Matrix4d createTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

// Rotation about arbitrary axis using Rodrigues' formula
Eigen::Matrix3d axisAngleRotation(const Eigen::Vector3d& axis, double angle);

}  // namespace detail
}  // namespace jacobian
