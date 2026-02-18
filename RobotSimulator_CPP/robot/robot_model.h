#pragma once

#include <string>
#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <tinyxml2.h>

struct RobotJoint {
    std::string name;
    std::string type; // revolute, fixed, prismatic
    glm::vec3 axis;
    struct { float lower, upper; } limits;
    glm::mat4 origin; // Transform from parent link
    std::string parentLinkName;
    std::string childLinkName;
    float currentAngle = 0.0f;
};

struct RobotLink {
    std::string name;
    std::string visualMeshPath;
    glm::mat4 visualOrigin;
    // Pointers to rendering resources (mesh buffers) would go here
};

class RobotModel {
public:
    RobotModel();
    ~RobotModel();

    bool LoadURDF(const std::string& configPath);
    void Update(float deltaTime);
    void SetJointPosition(const std::string& jointName, float angle);
    
    // Getters for rendering
    const std::vector<RobotLink>& GetLinks() const { return m_links; }
    glm::mat4 GetGlobalTransform(const std::string& linkName);

private:
    void ParseLink(tinyxml2::XMLElement* linkXml);
    void ParseJoint(tinyxml2::XMLElement* jointXml);
    glm::mat4 ParsePose(tinyxml2::XMLElement* originXml);

    std::vector<RobotLink> m_links;
    std::vector<RobotJoint> m_joints;
    
    // Joint map for fast lookup
    // Hierarchy tree for easy traversal
};
