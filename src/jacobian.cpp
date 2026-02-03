#include "jacobian.hpp"

namespace jacobian {

void computeLinearJacobian(const Model& model, Data& data) {
    // End-effector position in base frame
    const Eigen::Vector3d p_e = data.end_effector_transform.block<3, 1>(0, 3);

    // For each revolute joint
    for (size_t i = 0; i < 6; ++i) {
        const Eigen::Matrix4d& T_i = data.joint_transforms[i];

        // Get the joint's rotation axis in local frame
        const size_t joint_idx = model.revolute_joint_indices[i];
        const Eigen::Vector3d& axis_local = model.joints[joint_idx].axis;

        // Transform axis to base frame: z_i = R_i * axis_local
        const Eigen::Vector3d z_i = T_i.block<3, 3>(0, 0) * axis_local;

        // Joint position in base frame
        const Eigen::Vector3d p_i = T_i.block<3, 1>(0, 3);

        // Jacobian column: z_i × (p_e - p_i)
        data.Jv.col(i) = z_i.cross(p_e - p_i);
    }
}

void computeJacobian(
    const Model& model,
    const std::array<double, 6>& q,
    Data& data)
{
    computeForwardKinematics(model, q, data);
    computeLinearJacobian(model, data);
}

}  // namespace jacobian
