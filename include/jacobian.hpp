#pragma once

#include "model.hpp"
#include "forward_kinematics.hpp"
#include <array>

namespace jacobian {

// Compute linear velocity Jacobian (3x6)
// Requires FK to have been computed first (data.joint_transforms and data.end_effector_transform must be valid)
// Writes result into data.Jv
void computeLinearJacobian(const Model& model, Data& data);

// Convenience function: compute both FK and Jacobian
void computeJacobian(
    const Model& model,
    const std::array<double, 6>& q,
    Data& data);

}  // namespace jacobian
