#include "urdf_parser.hpp"
#include "forward_kinematics.hpp"
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
        Eigen::Matrix3d R = rpyToRotationMatrix(origin_rpy.x(), origin_rpy.y(), origin_rpy.z());
        joint.placement = createTransform(R, origin_xyz);

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

    // Traverse the serial chain
    while (joints_by_parent.find(current_link) != joints_by_parent.end()) {
        const auto& children = joints_by_parent[current_link];
        if (children.empty()) break;

        const Joint& joint = children[0];
        model.joints.push_back(joint);

        if (joint.type == JointType::Revolute) {
            model.revolute_joint_indices.push_back(model.joints.size() - 1);
            model.num_revolute_joints++;
        }

        current_link = joint.child_link;
    }

    if (model.num_revolute_joints == 0) {
        throw std::runtime_error("No revolute joints found in URDF");
    }

    return model;
}

}  // namespace jacobian
