#pragma once

#include "model.hpp"
#include <string>

namespace jacobian {

// Parse URDF file and return a Model
// If end_effector_link is provided, the parser follows the chain toward that link
// at branching points, ensuring deterministic traversal.
// If empty (default), the parser follows the longest chain from the root.
// Throws std::runtime_error on parse failure
Model parseURDF(const std::string& urdf_path, const std::string& end_effector_link = "");

}  // namespace jacobian
