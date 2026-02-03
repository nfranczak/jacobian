#pragma once

#include "model.hpp"
#include <string>

namespace jacobian {

// Parse URDF file and return a Model
// Throws std::runtime_error on parse failure
Model parseURDF(const std::string& urdf_path);

}  // namespace jacobian
