#pragma once

#include "model.hpp"
#include "forward_kinematics.hpp"

namespace jacobian {

// Compute full Jacobian (6 x num_revolute_joints): [linear velocity; angular velocity]
// Requires FK to have been computed first (data.joint_transforms and data.end_effector_transform must be valid)
// Writes result into data.J
void computeJacobian(const Model& model, Data& data);

// Convenience function: compute both FK and Jacobian
// q must have model.revolute_joint_indices.size() elements
void computeJacobian(
    const Model& model,
    const Eigen::VectorXd& q,
    Data& data);

}  // namespace jacobian
