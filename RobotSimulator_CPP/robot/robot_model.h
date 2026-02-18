#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <tinyxml2.h>

struct MeshData {
    std::vector<float> vertices;   // x,y,z per vertex
    std::vector<float> normals;    // nx,ny,nz per vertex
    std::vector<uint32_t> indices;
    bool loaded = false;
};

struct RobotJoint {
    std::string name;
    std::string type; // revolute, fixed, prismatic, continuous
    glm::vec3 axis = glm::vec3(0, 0, 1);
    struct { float lower = 0, upper = 0; } limits;
    glm::mat4 origin = glm::mat4(1.0f); // Transform from parent link
    std::string parentLinkName;
    std::string childLinkName;
    float currentAngle = 0.0f;
};

struct RobotLink {
    std::string name;
    std::string visualMeshPath;
    glm::vec3 visualScale = glm::vec3(1.0f);
    glm::mat4 visualOrigin = glm::mat4(1.0f);
    MeshData mesh;
};

class RobotModel {
public:
    RobotModel();
    ~RobotModel();

    bool LoadURDF(const std::string& configPath);
    bool LoadMesh(const std::string& meshPath, MeshData& outMesh);
    void Update(float deltaTime);
    void SetJointPosition(const std::string& jointName, float angle);
    
    // Getters
    const std::vector<RobotLink>& GetLinks() const { return m_links; }
    const std::vector<RobotJoint>& GetJoints() const { return m_joints; }
    std::vector<RobotJoint>& GetJointsMutable() { return m_joints; }
    
    glm::mat4 GetGlobalTransform(const std::string& linkName) const;
    glm::mat4 GetEndEffectorPose() const;
    
    int GetDOF() const;
    std::vector<float> GetJointAngles() const;
    void SetJointAngles(const std::vector<float>& angles);

    // Supported mesh formats (via Assimp):
    // STL, OBJ, DAE (Collada), FBX, STEP (limited), 3DS, PLY, glTF
    static const std::vector<std::string>& GetSupportedFormats();

private:
    void ParseLink(tinyxml2::XMLElement* linkXml, const std::string& basePath);
    void ParseJoint(tinyxml2::XMLElement* jointXml);
    glm::mat4 ParsePose(tinyxml2::XMLElement* originXml);
    glm::vec3 ParseVec3(const char* str);
    void BuildKinematicChain();

    std::vector<RobotLink> m_links;
    std::vector<RobotJoint> m_joints;
    
    // Fast lookup
    std::unordered_map<std::string, size_t> m_linkIndex;
    std::unordered_map<std::string, size_t> m_jointIndex;
    
    // Kinematic chain (ordered from base to tip)
    std::vector<size_t> m_chainJointIndices;
    std::string m_baseLinkName;
};
