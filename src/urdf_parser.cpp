#include "urdf_parser.hpp"
#include "forward_kinematics.hpp"
#include <tinyxml2.h>
#include <stdexcept>
#include <unordered_map>
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

// Parse joint type string
JointType parseJointType(const char* type_str) {
    if (!type_str) return JointType::Fixed;
    std::string type(type_str);
    if (type == "revolute" || type == "continuous") {
        return JointType::Revolute;
    }
    return JointType::Fixed;
}

// Find first child element with given name (handles duplicates in malformed URDF)
const tinyxml2::XMLElement* findFirstChild(
    const tinyxml2::XMLElement* parent,
    const char* name)
{
    return parent->FirstChildElement(name);
}

}  // anonymous namespace

Model parseURDF(const std::string& urdf_path) {
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

    // First pass: collect all joints into a map keyed by parent link
    std::unordered_map<std::string, std::vector<Joint>> joints_by_parent;
    std::unordered_map<std::string, Joint> joints_by_name;

    for (const auto* joint_elem = robot->FirstChildElement("joint");
         joint_elem;
         joint_elem = joint_elem->NextSiblingElement("joint"))
    {
        Joint joint;
        joint.name = joint_elem->Attribute("name") ? joint_elem->Attribute("name") : "";
        joint.type = parseJointType(joint_elem->Attribute("type"));

        // Parse parent and child links (take first occurrence)
        if (const auto* parent = findFirstChild(joint_elem, "parent")) {
            joint.parent_link = parent->Attribute("link") ? parent->Attribute("link") : "";
        }
        if (const auto* child = findFirstChild(joint_elem, "child")) {
            joint.child_link = child->Attribute("link") ? child->Attribute("link") : "";
        }

        // Parse origin (take first occurrence)
        if (const auto* origin = findFirstChild(joint_elem, "origin")) {
            joint.origin_xyz = parseVector3(origin->Attribute("xyz"));
            joint.origin_rpy = parseVector3(origin->Attribute("rpy"));
        }

        // Parse axis (take first occurrence, default to [1,0,0] per URDF spec)
        joint.axis = Eigen::Vector3d::UnitX();  // URDF default
        if (const auto* axis_elem = findFirstChild(joint_elem, "axis")) {
            joint.axis = parseVector3(axis_elem->Attribute("xyz"));
            joint.axis.normalize();
        }

        // Precompute static placement transform
        Eigen::Matrix3d R = rpyToRotationMatrix(
            joint.origin_rpy.x(),
            joint.origin_rpy.y(),
            joint.origin_rpy.z());
        joint.placement = createTransform(R, joint.origin_xyz);

        joints_by_parent[joint.parent_link].push_back(joint);
        joints_by_name[joint.name] = joint;
    }

    // Second pass: build ordered kinematic chain starting from root
    // Find root link (typically "world" or "base_link")
    std::string current_link = "world";
    if (joints_by_parent.find(current_link) == joints_by_parent.end()) {
        current_link = "base_link";
    }

    // Traverse the chain
    while (joints_by_parent.find(current_link) != joints_by_parent.end()) {
        const auto& children = joints_by_parent[current_link];
        if (children.empty()) break;

        // Take first child joint (assumes serial chain)
        const Joint& joint = children[0];
        model.joints.push_back(joint);

        // Track revolute joints
        if (joint.type == JointType::Revolute) {
            if (model.num_revolute_joints < 6) {
                model.revolute_joint_indices[model.num_revolute_joints] = model.joints.size() - 1;
            }
            model.num_revolute_joints++;
        }

        current_link = joint.child_link;
    }

    if (model.num_revolute_joints != 6) {
        throw std::runtime_error(
            "Expected 6 revolute joints, found " + std::to_string(model.num_revolute_joints));
    }

    return model;
}

}  // namespace jacobian
