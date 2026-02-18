#include "robot_model.h"
#include <iostream>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <spdlog/spdlog.h>

using namespace tinyxml2;

RobotModel::RobotModel() {}
RobotModel::~RobotModel() {}

bool RobotModel::LoadURDF(const std::string& configPath) {
    XMLDocument doc;
    if (doc.LoadFile(configPath.c_str()) != XML_SUCCESS) {
        spdlog::error("Failed to load URDF: {}", configPath);
        return false;
    }

    XMLElement* root = doc.RootElement();
    if (!root || std::string(root->Name()) != "robot") {
        spdlog::error("Invalid URDF: Root is not 'robot'");
        return false;
    }

    // Parse Links
    XMLElement* link = root->FirstChildElement("link");
    while (link) {
        ParseLink(link);
        link = link->NextSiblingElement("link");
    }

    // Parse Joints
    XMLElement* joint = root->FirstChildElement("joint");
    while (joint) {
        ParseJoint(joint);
        joint = joint->NextSiblingElement("joint");
    }

    spdlog::info("Loaded Robot: {} Links, {} Joints", m_links.size(), m_joints.size());
    return true;
}

void RobotModel::ParseLink(XMLElement* linkXml) {
    RobotLink link;
    link.name = linkXml->Attribute("name");

    XMLElement* visual = linkXml->FirstChildElement("visual");
    if (visual) {
        XMLElement* geometry = visual->FirstChildElement("geometry");
        if (geometry) {
            XMLElement* mesh = geometry->FirstChildElement("mesh");
            if (mesh) {
                link.visualMeshPath = mesh->Attribute("filename");
                // Load Mesh with Assimp here in real implementation
                // Assimp::Importer importer;
                // const aiScene* scene = importer.ReadFile(link.visualMeshPath, aiProcess_Triangulate | aiProcess_FlipUVs);
            }
        }
        
        XMLElement* origin = visual->FirstChildElement("origin");
        if (origin) {
            link.visualOrigin = ParsePose(origin);
        } else {
            link.visualOrigin = glm::mat4(1.0f);
        }
    }
    m_links.push_back(link);
}

void RobotModel::ParseJoint(XMLElement* jointXml) {
    RobotJoint joint;
    joint.name = jointXml->Attribute("name");
    joint.type = jointXml->Attribute("type");

    XMLElement* parent = jointXml->FirstChildElement("parent");
    if (parent) joint.parentLinkName = parent->Attribute("link");

    XMLElement* child = jointXml->FirstChildElement("child");
    if (child) joint.childLinkName = child->Attribute("link");
    
    XMLElement* limit = jointXml->FirstChildElement("limit");
    if (limit) {
        joint.limits.lower = limit->FloatAttribute("lower");
        joint.limits.upper = limit->FloatAttribute("upper");
    }

    XMLElement* origin = jointXml->FirstChildElement("origin");
    if (origin) {
        joint.origin = ParsePose(origin);
    } else {
        joint.origin = glm::mat4(1.0f);
    }
    
    // Axis...
    
    m_joints.push_back(joint);
}

glm::mat4 RobotModel::ParsePose(XMLElement* originXml) {
    // Parse xyz="x y z" rpy="r p y"
    // Return glm::mat4
    return glm::mat4(1.0f); // Placeholder
}

void RobotModel::Update(float deltaTime) {
    // Update transforms based on joint angles
}
