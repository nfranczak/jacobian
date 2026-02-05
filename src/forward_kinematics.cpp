#include "forward_kinematics.hpp"
#include <cmath>

namespace jacobian {

Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw) {
    // URDF uses fixed-axis XYZ convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    Eigen::Matrix3d R;
    R << cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
         sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
        -sp,       cp * sr,                 cp * cr;

    return R;
}

Eigen::Matrix4d createTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

Eigen::Matrix3d axisAngleRotation(const Eigen::Vector3d& axis, double angle) {
    // Rodrigues' formula: R = I + sin(θ)K + (1-cos(θ))K²
    // where K is the skew-symmetric matrix of the axis
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;

    const Eigen::Vector3d a = axis.normalized();
    const double x = a.x();
    const double y = a.y();
    const double z = a.z();

    Eigen::Matrix3d R;
    R << t * x * x + c,      t * x * y - s * z,  t * x * z + s * y,
         t * x * y + s * z,  t * y * y + c,      t * y * z - s * x,
         t * x * z - s * y,  t * y * z + s * x,  t * z * z + c;

    return R;
}

void computeForwardKinematics(
    const Model& model,
    const Eigen::VectorXd& q,
    Data& data)
{
    Eigen::Matrix4d T_current = Eigen::Matrix4d::Identity();
    size_t revolute_count = 0;

    for (const auto& joint : model.joints) {
        // Apply precomputed static placement transform
        T_current = T_current * joint.placement;

        if (joint.type == JointType::Revolute) {
            // Store transform BEFORE applying joint rotation
            // This gives us the frame where z_i and p_i are defined
            data.joint_transforms[revolute_count] = T_current;

            // Apply joint rotation about local axis
            Eigen::Matrix3d R_joint = axisAngleRotation(joint.axis, q[revolute_count]);
            Eigen::Matrix4d T_joint = Eigen::Matrix4d::Identity();
            T_joint.block<3, 3>(0, 0) = R_joint;

            T_current = T_current * T_joint;
            ++revolute_count;
        }
    }

    data.end_effector_transform = T_current;
}

}  // namespace jacobian
