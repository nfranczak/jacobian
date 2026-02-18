#include "urdf_parser.hpp"
#include "detail/transforms.hpp"
#include <tinyxml2.h>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <sstream>

namespace jacobian {

namespace {

// Parse "x y z" string into Vector3d
Eigen::Vector3d parseVector3(const char* str) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (str) {
        std::istringstream iss(str);
        iss >> v.x() >> v.y() >> v.z();
    }
    return v;
}

JointType parseJointType(const char* type_str) {
    if (!type_str) return JointType::Fixed;
    std::string type(type_str);
    if (type == "revolute" || type == "continuous") {
        return JointType::Revolute;
    }
    return JointType::Fixed;
}

// Check if target link is reachable from the given link
bool isReachable(const std::string& link, const std::string& target,
                 const std::unordered_map<std::string, std::vector<Joint>>& joints_by_parent) {
    if (link == target) return true;
    auto it = joints_by_parent.find(link);
    if (it == joints_by_parent.end()) return false;
    for (const auto& joint : it->second) {
        if (isReachable(joint.child_link, target, joints_by_parent)) return true;
    }
    return false;
}

// Count the number of revolute joints reachable from a link (used to pick the longest chain)
size_t chainDepth(const std::string& link,
                  const std::unordered_map<std::string, std::vector<Joint>>& joints_by_parent) {
    auto it = joints_by_parent.find(link);
    if (it == joints_by_parent.end()) return 0;
    size_t max_depth = 0;
    for (const auto& joint : it->second) {
        size_t depth = (joint.type == JointType::Revolute ? 1 : 0)
                     + chainDepth(joint.child_link, joints_by_parent);
        max_depth = std::max(max_depth, depth);
    }
    return max_depth;
}

}  // anonymous namespace

Model parseURDF(const std::string& urdf_path, const std::string& end_effector_link) {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to load URDF file: " + urdf_path);
    }

    const auto* robot = doc.FirstChildElement("robot");
    if (!robot) {
        throw std::runtime_error("No <robot> element found in URDF");
    }

    Model model;
    model.name = robot->Attribute("name") ? robot->Attribute("name") : "unnamed";

    // Collect all joints keyed by parent link
    std::unordered_map<std::string, std::vector<Joint>> joints_by_parent;

    // Track all child links so we can find the root
    std::unordered_set<std::string> child_links;
    std::unordered_set<std::string> parent_links;

    for (const auto* joint_elem = robot->FirstChildElement("joint");
         joint_elem;
         joint_elem = joint_elem->NextSiblingElement("joint"))
    {
        Joint joint;
        joint.name = joint_elem->Attribute("name") ? joint_elem->Attribute("name") : "";
        joint.type = parseJointType(joint_elem->Attribute("type"));

        if (const auto* parent = joint_elem->FirstChildElement("parent")) {
            joint.parent_link = parent->Attribute("link") ? parent->Attribute("link") : "";
        }
        if (const auto* child = joint_elem->FirstChildElement("child")) {
            joint.child_link = child->Attribute("link") ? child->Attribute("link") : "";
        }

        // Parse origin
        Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d origin_rpy = Eigen::Vector3d::Zero();
        if (const auto* origin = joint_elem->FirstChildElement("origin")) {
            origin_xyz = parseVector3(origin->Attribute("xyz"));
            origin_rpy = parseVector3(origin->Attribute("rpy"));
        }

        // Parse axis (default [1,0,0] per URDF spec)
        joint.axis = Eigen::Vector3d::UnitX();
        if (const auto* axis_elem = joint_elem->FirstChildElement("axis")) {
            joint.axis = parseVector3(axis_elem->Attribute("xyz"));
            joint.axis.normalize();
        }

        // Precompute static placement transform
        Eigen::Matrix3d R = detail::rpyToRotationMatrix(origin_rpy.x(), origin_rpy.y(), origin_rpy.z());
        joint.placement = detail::createTransform(R, origin_xyz);

        parent_links.insert(joint.parent_link);
        child_links.insert(joint.child_link);
        joints_by_parent[joint.parent_link].push_back(joint);
    }

    // Find root link: a parent link that is never a child of any joint
    std::string current_link;
    for (const auto& pl : parent_links) {
        if (child_links.find(pl) == child_links.end()) {
            current_link = pl;
            break;
        }
    }
    if (current_link.empty() && !joints_by_parent.empty()) {
        current_link = joints_by_parent.begin()->first;
    }

    // Traverse the kinematic chain, resolving branching deterministically
    while (joints_by_parent.find(current_link) != joints_by_parent.end()) {
        const auto& children = joints_by_parent[current_link];
        if (children.empty()) break;

        const Joint* chosen = nullptr;

        if (children.size() == 1) {
            chosen = &children[0];
        } else if (!end_effector_link.empty()) {
            // Pick the branch that leads to the specified end-effector
            for (const auto& joint : children) {
                if (isReachable(joint.child_link, end_effector_link, joints_by_parent)) {
                    chosen = &joint;
                    break;
                }
            }
            if (!chosen) {
                throw std::runtime_error(
                    "End-effector link '" + end_effector_link
                    + "' is not reachable from link '" + current_link + "'");
            }
        } else {
            // No end-effector specified: follow the branch with the most revolute joints
            size_t best_depth = 0;
            for (const auto& joint : children) {
                size_t depth = (joint.type == JointType::Revolute ? 1 : 0)
                             + chainDepth(joint.child_link, joints_by_parent);
                if (depth > best_depth) {
                    best_depth = depth;
                    chosen = &joint;
                }
            }
            if (!chosen) chosen = &children[0];
        }

        model.joints.push_back(*chosen);

        if (chosen->type == JointType::Revolute) {
            model.revolute_joint_indices.push_back(model.joints.size() - 1);
        }

        current_link = chosen->child_link;
    }

    if (model.revolute_joint_indices.empty()) {
        throw std::runtime_error("No revolute joints found in URDF");
    }

    return model;
}

}  // namespace jacobian
