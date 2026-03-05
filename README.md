# Jacobian for Robotic Arms

A C++ library to compute the geometric Jacobian (linear + angular velocity) for N-DOF robotic arms with revolute joints. Loads robot kinematics from URDF files.

## Features

- URDF parsing for robot model definition
- Forward kinematics computation
- Analytical geometric Jacobian (6 x N: linear and angular velocity)
- Jacobian layout: `[linear (3); angular (3)]` (Siciliano/Spong convention, same as KDL/ROS)
- Zero-allocation runtime (Model/Data separation pattern)
- Supports arbitrary N-DOF serial chains with revolute joints
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

    // Preallocate data struct (sized to model, reused for each computation)
    Data data(model);

    // Joint configuration (radians)
    Eigen::VectorXd q(6);
    q << 0.0, -1.57, 1.57, 0.0, 1.57, 0.0;

    // Compute FK and Jacobian
    computeJacobian(model, q, data);

    // Access results
    Eigen::Vector3d position = data.end_effector_transform.block<3,1>(0,3);
    Eigen::MatrixXd J = data.J;  // 6 x N matrix (linear velocity rows 0-2, angular velocity rows 3-5)

    // Compute end-effector velocity from joint velocities
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(6);
    q_dot[0] = 0.1;
    Eigen::Vector3d linear_velocity = J.topRows(3) * q_dot;
    Eigen::Vector3d angular_velocity = J.bottomRows(3) * q_dot;

    return 0;
}
```

### Real-time Loop

```cpp
Model model = parseURDF("robot.urdf");
Data data(model);

while (running) {
    Eigen::VectorXd q = getCurrentJointAngles();

    // No allocations here - data is reused
    computeJacobian(model, q, data);

    // Use data.J for control...
}
```

### Separate FK and Jacobian

```cpp
// If you only need FK
computeForwardKinematics(model, q, data);
Eigen::Vector3d p = data.end_effector_transform.block<3,1>(0,3);

// Jacobian requires FK to be computed first
computeJacobian(model, data);
```

### Specifying End-Effector Link

For URDFs with branching kinematic trees, specify the end-effector link to ensure the correct chain is followed:

```cpp
Model model = parseURDF("robot.urdf", "flange");
```

If omitted, the parser follows the branch with the most revolute joints.

## API Reference

### Data Structures

```cpp
struct Model {
    std::string name;
    std::vector<Joint> joints;                  // All joints in chain order
    std::vector<size_t> revolute_joint_indices;  // Indices of revolute joints
};

struct Data {
    std::vector<Eigen::Matrix4d> joint_transforms;  // Pre-rotation frames
    Eigen::Matrix4d end_effector_transform;          // FK result
    Eigen::MatrixXd J;                               // Geometric Jacobian (6 x N)

    explicit Data(const Model& model);  // Preallocates based on model
};
```

### Functions

| Function | Description |
|----------|-------------|
| `Model parseURDF(const std::string& path, const std::string& end_effector_link = "")` | Load robot from URDF file |
| `void computeForwardKinematics(const Model&, const Eigen::VectorXd& q, Data&)` | Compute FK only |
| `void computeJacobian(const Model&, Data&)` | Compute Jacobian (requires FK first) |
| `void computeJacobian(const Model&, const Eigen::VectorXd& q, Data&)` | Compute both FK and Jacobian |

## URDF Requirements

The library expects:
- At least one revolute joint in the kinematic chain
- Serial chain structure (branching resolved via end-effector link parameter)
- Standard URDF format with `<joint>`, `<parent>`, `<child>`, `<origin>`, `<axis>` elements

Tested with Universal Robots UR20 URDF and a 2-link planar robot.

## Algorithm

The geometric Jacobian relates joint velocities to end-effector linear and angular velocity:

```
[v; ω] = J * q̇
```

For each revolute joint i, the Jacobian column is:

```
J[:,i] = [zi × (pe - pi); zi]
```

Where:
- `zi` = rotation axis of joint i in base frame
- `pi` = position of joint i in base frame
- `pe` = end-effector position in base frame
- Top 3 rows: linear velocity contribution
- Bottom 3 rows: angular velocity contribution

## Testing

```bash
cd build && ctest
```

Or run the test binary directly:

```bash
./build/test_jacobian
```

If you need to rebuild first:

```bash
cmake --build build && cd build && ctest
```

## License

MIT
