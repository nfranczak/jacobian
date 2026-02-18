#pragma once

#include "model.hpp"
#include "forward_kinematics.hpp"

namespace jacobian {

// Compute linear velocity Jacobian (3 x num_revolute_joints)
// Requires FK to have been computed first (data.joint_transforms and data.end_effector_transform must be valid)
// Writes result into data.Jv
void computeLinearJacobian(const Model& model, Data& data);

// Convenience function: compute both FK and Jacobian
// q must have model.revolute_joint_indices.size() elements
void computeJacobian(
    const Model& model,
    const Eigen::VectorXd& q,
    Data& data);

}  // namespace jacobian
