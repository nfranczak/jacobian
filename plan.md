# Linear Velocity Jacobian Implementation Plan

## Overview

Implement a C++ library to compute the linear velocity Jacobian (3x6) for a 6DoF robotic arm with all revolute joints. The robot is defined via URDF, and we use Eigen for linear algebra.

**Target robot:** UR20 (Universal Robots)

---

## UR20 Kinematic Chain (from URDF analysis)

```
world ──[fixed]──> base_link ──[fixed, Rz(π)]──> base_link_inertia
   ──[shoulder_pan_joint]──> shoulder_link
   ──[shoulder_lift_joint]──> upper_arm_link
   ──[elbow_joint]──> forearm_link
   ──[wrist_1_joint]──> wrist_1_link
   ──[wrist_2_joint]──> wrist_2_link
   ──[wrist_3_joint]──> wrist_3_link
   ──[fixed]──> flange (end-effector)
```

**Key observations:**
- All 6 revolute joints have `axis = [0, 0, 1]` (local Z-axis)
- Fixed joints exist in the chain and must be included in transforms
- End-effector frame is `flange`

---

## Dependencies

- **Eigen3** - Matrix operations, cross products
- **tinyxml2** - URDF parsing (lightweight, header-only option available)
- **C++17** - Standard library features

---

## Project Structure

```
jacobian/
├── include/
│   ├── model.hpp                # Model + Data structures
│   ├── urdf_parser.hpp          # URDF parsing → Model
│   ├── forward_kinematics.hpp   # FK computation → Data
│   └── jacobian.hpp             # Jacobian computation → Data
├── src/
│   ├── urdf_parser.cpp
│   ├── forward_kinematics.cpp
│   └── jacobian.cpp
├── test/
│   └── test_jacobian.cpp
├── urdfs/
│   ├── ur20FK.urdf
│   └── ur20Collapsed.urdf
├── CMakeLists.txt
└── plan.md
```

---

## Data Structures (Model/Data Separation)

Following Pinocchio's pattern, we separate:
- **Model**: Static robot description (const after loading)
- **Data**: Runtime computation results (reused across calls)

This avoids reallocation and keeps the model immutable.

---

### JointType
```cpp
enum class JointType {
    Fixed,
    Revolute
};
```

### Joint (static description)
```cpp
struct Joint {
    std::string name;
    JointType type;
    std::string parent_link;
    std::string child_link;
    Eigen::Vector3d origin_xyz;      // translation from parent
    Eigen::Vector3d origin_rpy;      // roll-pitch-yaw (XYZ fixed angles)
    Eigen::Vector3d axis;            // rotation axis in local frame (for revolute)

    // Precomputed static transform (from origin_xyz + origin_rpy)
    Eigen::Matrix4d placement;       // T_parent_to_joint_origin
};
```

### Model (static, immutable after parsing)
```cpp
struct Model {
    std::string name;

    // All joints in chain order (fixed + revolute)
    std::vector<Joint> joints;

    // Fast lookup: indices of the 6 revolute joints within joints[]
    std::array<size_t, 6> revolute_joint_indices;

    // Parent index for each joint (-1 for root)
    // Enables O(n) FK without recursion (Pinocchio pattern)
    std::vector<int> parent_indices;
};
```

### Data (mutable, reused for computations)
```cpp
struct Data {
    // Transform from base frame to each revolute joint frame (before rotation)
    // Used for Jacobian: z_i and p_i extracted from these
    std::array<Eigen::Matrix4d, 6> joint_transforms;

    // Transform from base frame to end-effector
    Eigen::Matrix4d end_effector_transform;

    // Jacobian matrix (computed on demand)
    Eigen::Matrix<double, 3, 6> Jv;

    // Preallocate to avoid repeated allocation
    Data() {
        for (auto& T : joint_transforms) T.setIdentity();
        end_effector_transform.setIdentity();
        Jv.setZero();
    }
};
```

### Usage Pattern
```cpp
// Once at startup
Model model = parseURDF("robot.urdf");

// Reused for each computation
Data data;

// Called frequently (no allocations)
computeForwardKinematics(model, q, data);
computeLinearJacobian(model, data);
```

---

## Implementation Steps

### Step 1: URDF Parser

**Goal:** Extract the full kinematic chain from URDF XML into an immutable Model.

**Approach:**
1. Parse XML using tinyxml2
2. Find ALL `<joint>` elements (both `type="fixed"` and `type="revolute"`)
3. Extract for each joint:
   - `<origin xyz="..." rpy="..."/>` - transform from parent to child
   - `<axis xyz="..."/>` - rotation axis (only for revolute)
   - `<parent link="..."/>` and `<child link="..."/>`
4. Build ordered chain by traversing parent→child relationships starting from `world` or `base_link`
5. **Precompute** static placement transforms (`Joint::placement`) during parsing
6. Build `parent_indices[]` array for O(n) FK traversal
7. Track which joints are revolute (store in `revolute_joint_indices`)

**Edge cases to handle:**
- The collapsed URDF has duplicate elements in joints - take first valid occurrence
- Default `origin` is identity if not specified
- Default `axis` is `[1,0,0]` per URDF spec (but UR20 uses `[0,0,1]`)

**Key function:**
```cpp
Model parseURDF(const std::string& urdf_path);
```

### Step 2: Forward Kinematics

**Goal:** Compute transforms for each revolute joint frame, plus end-effector. Results stored in Data struct.

**Algorithm (Pinocchio-style, no recursion):**
```
T_current = I (identity)
revolute_count = 0

For each joint in model.joints:
    # Use precomputed static placement (computed once during parsing)
    T_current = T_current * joint.placement

    If joint is revolute:
        # Store transform BEFORE applying joint rotation
        # This gives us z_i and p_i in base frame
        data.joint_transforms[revolute_count] = T_current

        # Apply joint rotation about local axis
        T_joint = rotation_about_axis(joint.axis, q[revolute_count])
        T_current = T_current * T_joint
        revolute_count++

    # Fixed joints: placement already applied, continue

# Final transform is base to end-effector
data.end_effector_transform = T_current
```

**Important:** We store transforms at each revolute joint **before** its rotation is applied. This is where `z_i` (the rotation axis) and `p_i` (joint origin) are extracted.

**Helper functions (in forward_kinematics.hpp):**
```cpp
// RPY (roll-pitch-yaw) to rotation matrix
// URDF uses fixed-axis XYZ convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw);

// Create 4x4 homogeneous transform from R and t
Eigen::Matrix4d createTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

// Rodrigues' formula: rotation about arbitrary axis
Eigen::Matrix3d axisAngleRotation(const Eigen::Vector3d& axis, double angle);
```

**Key function:**
```cpp
// Writes results into data (no allocation)
void computeForwardKinematics(
    const Model& model,
    const std::array<double, 6>& q,
    Data& data);
```

### Step 3: Linear Velocity Jacobian

**Goal:** Compute 3x6 Jacobian matrix. Results stored in `data.Jv`.

**Algorithm (from PDF):**
```
p_e = data.end_effector_transform(0:3, 3)   # end-effector position

For each revolute joint i = 0 to 5:
    T_i = data.joint_transforms[i]
    axis_local = model.joints[revolute_joint_indices[i]].axis

    # z_i: rotation axis in base frame
    # Transform local axis to world: z_i = R_i * axis_local
    z_i = T_i(0:3, 0:3) * axis_local

    # p_i: joint origin in base frame
    p_i = T_i(0:3, 3)

    # Jacobian column: cross product
    data.Jv.col(i) = z_i.cross(p_e - p_i)
```

**Key function:**
```cpp
// Writes results into data.Jv (no allocation)
// Requires FK to have been computed first
void computeLinearJacobian(const Model& model, Data& data);
```

**Convenience function (computes both FK and Jacobian):**
```cpp
void computeJacobian(
    const Model& model,
    const std::array<double, 6>& q,
    Data& data);
```

---

## Frame Convention

**URDF RPY convention:** Fixed-axis XYZ (extrinsic)
- `R = Rz(yaw) * Ry(pitch) * Rx(roll)`
- This is equivalent to intrinsic ZYX Euler angles

**Jacobian frame:**
- All vectors (z_i, p_i, p_e) expressed in base frame (`base_link` or `world`)
- The `axis` in URDF is in the joint's local frame → must transform to base frame

**UR20 specifics:**
- Base has a fixed π rotation about Z between `base_link` and `base_link_inertia`
- All revolute axes are local Z `[0, 0, 1]`

---

## Verification Strategy

1. **Dimension check:** `data.Jv` must be 3x6

2. **Zero configuration test:**
   - At q=[0,0,0,0,0,0], compute FK manually and verify p_e
   - Verify first Jacobian column: J[:,0] should be `z_0 × (p_e - p_0)`

3. **Numerical differentiation (primary verification):**
   ```cpp
   double delta = 1e-7;
   Data data_plus, data_minus;
   Eigen::Matrix<double, 3, 6> J_numerical;

   for (int i = 0; i < 6; ++i) {
       auto q_plus = q;  q_plus[i] += delta;
       auto q_minus = q; q_minus[i] -= delta;

       computeForwardKinematics(model, q_plus, data_plus);
       computeForwardKinematics(model, q_minus, data_minus);

       Eigen::Vector3d p_plus = data_plus.end_effector_transform.block<3,1>(0,3);
       Eigen::Vector3d p_minus = data_minus.end_effector_transform.block<3,1>(0,3);

       J_numerical.col(i) = (p_plus - p_minus) / (2.0 * delta);
   }

   // Compare
   double error = (data.Jv - J_numerical).norm();
   assert(error < 1e-6);
   ```

4. **Multiple configurations:** Test at q=[0,...], q=[π/4,...], random q values

---

## API Usage Example

```cpp
#include "model.hpp"
#include "urdf_parser.hpp"
#include "forward_kinematics.hpp"
#include "jacobian.hpp"

int main() {
    // ===== SETUP (once at startup) =====
    Model model = parseURDF("urdfs/ur20FK.urdf");
    Data data;  // Preallocated, reused

    // ===== RUNTIME (called frequently) =====
    std::array<double, 6> q = {0.0, -1.57, 1.57, 0.0, 1.57, 0.0};

    // Option 1: Compute FK and Jacobian together
    computeJacobian(model, q, data);

    // Option 2: Compute separately
    // computeForwardKinematics(model, q, data);
    // computeLinearJacobian(model, data);

    // Access results from data struct
    Eigen::Vector3d p_e = data.end_effector_transform.block<3,1>(0,3);
    std::cout << "End-effector position: " << p_e.transpose() << std::endl;

    // Use Jacobian: v = Jv * q_dot
    Eigen::Matrix<double, 6, 1> q_dot;
    q_dot << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;  // only joint 1 moving
    Eigen::Vector3d v = data.Jv * q_dot;
    std::cout << "End-effector velocity: " << v.transpose() << std::endl;

    // ===== NEXT ITERATION (reuses data, no allocation) =====
    q[0] += 0.01;
    computeJacobian(model, q, data);
    // ...

    return 0;
}
```

---

## UR20 Joint Parameters (from URDF)

| Joint | Parent → Child | Origin XYZ | Origin RPY | Axis |
|-------|---------------|------------|------------|------|
| shoulder_pan | base_link_inertia → shoulder | (0, 0, 0.2363) | (0, 0, 0) | Z |
| shoulder_lift | shoulder → upper_arm | (0, 0, 0) | (π/2, 0, 0) | Z |
| elbow | upper_arm → forearm | (-0.862, 0, 0) | (0, 0, 0) | Z |
| wrist_1 | forearm → wrist_1 | (-0.7287, 0, 0.201) | (0, 0, 0) | Z |
| wrist_2 | wrist_1 → wrist_2 | (0, -0.1593, 0) | (π/2, 0, 0) | Z |
| wrist_3 | wrist_2 → wrist_3 | (0, 0.1543, 0) | (π/2, π, π) | Z |

Plus fixed joints: base_link-base_link_inertia (Rz(π)), wrist_3-flange

---

## Future Extensions (out of scope for now)

- Angular velocity Jacobian (J_omega) to get full 6x6 geometric Jacobian
- Jacobian time derivative for dynamics
- Singularity detection (rank/condition number)
- Support for prismatic joints
- Variable DoF (templated)
