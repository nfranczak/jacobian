#pragma once

#include "model.hpp"
#include "detail/transforms.hpp"

namespace jacobian {

// Compute forward kinematics for N-DOF robot
// q must have model.revolute_joint_indices.size() elements
// Writes joint_transforms and end_effector_transform into data
void computeForwardKinematics(
    const Model& model,
    const Eigen::VectorXd& q,
    Data& data);

}  // namespace jacobian
