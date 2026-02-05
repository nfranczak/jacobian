# Linear Velocity Jacobian for 6DoF Robotic Arms

A C++ library to compute the linear velocity Jacobian (3×6) for 6DoF robotic arms with revolute joints. Loads robot kinematics from URDF files.

## Features

- URDF parsing for robot model definition
- Forward kinematics computation
- Analytical linear velocity Jacobian
- Zero-allocation runtime (Model/Data separation pattern)
- Tested against numerical differentiation

## Dependencies

- **Eigen3** (≥3.3)
- **tinyxml2** (fetched automatically if not found)
- **CMake** (≥3.16)
- **C++17** compiler

## Building

```bash
mkdir build && cd build
cmake ..
make
```

Run tests:
```bash
./test_jacobian
```

## Usage

### Basic Example

```cpp
#include "model.hpp"
#include "urdf_parser.hpp"
#include "forward_kinematics.hpp"
#include "jacobian.hpp"

using namespace jacobian;

int main() {
    // Load robot model once at startup
    Model model = parseURDF("path/to/robot.urdf");

    // Preallocate data struct (reused for each computation)
    Data data;

    // Joint configuration (radians)
    std::array<double, 6> q = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};

    // Compute FK and Jacobian
    computeJacobian(model, q, data);

    // Access results
    Eigen::Vector3d position = data.end_effector_transform.block<3,1>(0,3);
    Eigen::Matrix<double, 3, 6> Jv = data.Jv;

    // Compute end-effector velocity from joint velocities
    Eigen::Matrix<double, 6, 1> q_dot;
    q_dot << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Vector3d velocity = Jv * q_dot;

    return 0;
}
```

### Real-time Loop

```cpp
Model model = parseURDF("robot.urdf");
Data data;

while (running) {
    std::array<double, 6> q = getCurrentJointAngles();

    // No allocations here - data is reused
    computeJacobian(model, q, data);

    // Use data.Jv for control...
}
```

### Separate FK and Jacobian

```cpp
// If you only need FK
computeForwardKinematics(model, q, data);
Eigen::Vector3d p = data.end_effector_transform.block<3,1>(0,3);

// Jacobian requires FK to be computed first
computeLinearJacobian(model, data);
```

## API Reference

### Data Structures

```cpp
struct Model {
    std::string name;
    std::vector<Joint> joints;              // All joints in chain order
    std::array<size_t, 6> revolute_joint_indices;
    size_t num_revolute_joints;
};

struct Data {
    std::array<Eigen::Matrix4d, 6> joint_transforms;  // Pre-rotation frames
    Eigen::Matrix4d end_effector_transform;           // FK result
    Eigen::Matrix<double, 3, 6> Jv;                   // Linear velocity Jacobian
};
```

### Functions

| Function | Description |
|----------|-------------|
| `Model parseURDF(const std::string& path)` | Load robot from URDF file |
| `void computeForwardKinematics(const Model&, const std::array<double,6>&, Data&)` | Compute FK only |
| `void computeLinearJacobian(const Model&, Data&)` | Compute Jacobian (requires FK first) |
| `void computeJacobian(const Model&, const std::array<double,6>&, Data&)` | Compute both FK and Jacobian |

### Helper Functions

```cpp
// RPY to rotation matrix (URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll))
Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw);

// Rotation about arbitrary axis (Rodrigues' formula)
Eigen::Matrix3d axisAngleRotation(const Eigen::Vector3d& axis, double angle);

// Create 4x4 homogeneous transform
Eigen::Matrix4d createTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
```

## URDF Requirements

The library expects:
- Exactly 6 revolute joints in the kinematic chain
- Serial chain structure (no branching)
- Standard URDF format with `<joint>`, `<parent>`, `<child>`, `<origin>`, `<axis>` elements

Tested with Universal Robots UR20 URDF.

## Algorithm

The linear velocity Jacobian relates joint velocities to end-effector linear velocity:

```
v = Jv * q̇
```

For each revolute joint i, the Jacobian column is computed as:

```
Jv[:,i] = zi × (pe - pi)
```

Where:
- `zi` = rotation axis of joint i in base frame
- `pi` = position of joint i in base frame
- `pe` = end-effector position in base frame

## Testing
This is a CMake-based C++ project. You can run tests with:
cd build && ctest
Or run the test binary directly:
./build/test_jacobian
If you need to rebuild first:
cmake --build build && cd build && ctest


## License

MIT
