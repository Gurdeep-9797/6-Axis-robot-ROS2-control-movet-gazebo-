#include "robot_model.h"
#include <iostream>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <spdlog/spdlog.h>
#include <glm/gtc/matrix_transform.hpp>

using namespace tinyxml2;
namespace fs = std::filesystem;

// Supported 3D formats via Assimp
static const std::vector<std::string> s_supportedFormats = {
    ".stl", ".obj", ".dae", ".fbx", ".3ds", ".ply",
    ".gltf", ".glb", ".step", ".stp", ".iges", ".igs"
};

const std::vector<std::string>& RobotModel::GetSupportedFormats() {
    return s_supportedFormats;
}

RobotModel::RobotModel() {}
RobotModel::~RobotModel() {}

// ─────────────────────────────────────────────────────────
// URDF Loading
// ─────────────────────────────────────────────────────────

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

    // Clear previous data
    m_links.clear();
    m_joints.clear();
    m_linkIndex.clear();
    m_jointIndex.clear();

    // Resolve base path for mesh file references
    std::string basePath = fs::path(configPath).parent_path().string();

    // Parse Links
    XMLElement* link = root->FirstChildElement("link");
    while (link) {
        ParseLink(link, basePath);
        link = link->NextSiblingElement("link");
    }

    // Parse Joints
    XMLElement* joint = root->FirstChildElement("joint");
    while (joint) {
        ParseJoint(joint);
        joint = joint->NextSiblingElement("joint");
    }

    // Build kinematic chain
    BuildKinematicChain();

    spdlog::info("Loaded Robot: {} Links, {} Joints, {} DOF", 
                 m_links.size(), m_joints.size(), GetDOF());
    return true;
}

// ─────────────────────────────────────────────────────────
// Parsing
// ─────────────────────────────────────────────────────────

glm::vec3 RobotModel::ParseVec3(const char* str) {
    glm::vec3 v(0.0f);
    if (!str) return v;
    std::istringstream ss(str);
    ss >> v.x >> v.y >> v.z;
    return v;
}

glm::mat4 RobotModel::ParsePose(XMLElement* originXml) {
    if (!originXml) return glm::mat4(1.0f);

    glm::mat4 T(1.0f);

    // Parse translation: xyz="x y z"
    const char* xyzStr = originXml->Attribute("xyz");
    if (xyzStr) {
        glm::vec3 pos = ParseVec3(xyzStr);
        T = glm::translate(T, pos);
    }

    // Parse rotation: rpy="roll pitch yaw" (ZYX Euler convention)
    const char* rpyStr = originXml->Attribute("rpy");
    if (rpyStr) {
        glm::vec3 rpy = ParseVec3(rpyStr);
        // Apply in ZYX order (yaw, pitch, roll)
        T = glm::rotate(T, rpy.z, glm::vec3(0, 0, 1)); // Yaw
        T = glm::rotate(T, rpy.y, glm::vec3(0, 1, 0)); // Pitch
        T = glm::rotate(T, rpy.x, glm::vec3(1, 0, 0)); // Roll
    }

    return T;
}

void RobotModel::ParseLink(XMLElement* linkXml, const std::string& basePath) {
    RobotLink link;
    link.name = linkXml->Attribute("name");

    XMLElement* visual = linkXml->FirstChildElement("visual");
    if (visual) {
        XMLElement* geometry = visual->FirstChildElement("geometry");
        if (geometry) {
            // Try mesh first
            XMLElement* mesh = geometry->FirstChildElement("mesh");
            if (mesh) {
                std::string filename = mesh->Attribute("filename");
                
                // Resolve package:// URIs
                if (filename.substr(0, 10) == "package://") {
                    size_t slash = filename.find('/', 10);
                    if (slash != std::string::npos) {
                        filename = basePath + "/" + filename.substr(slash + 1);
                    }
                } else if (!fs::path(filename).is_absolute()) {
                    filename = basePath + "/" + filename;
                }
                
                link.visualMeshPath = filename;
                
                const char* scaleStr = mesh->Attribute("scale");
                if (scaleStr) {
                    link.visualScale = ParseVec3(scaleStr);
                }

                if (fs::exists(filename)) {
                    LoadMesh(filename, link.mesh);
                } else {
                    spdlog::warn("Mesh file not found: {}", filename);
                }
            }

            // Cylinder primitive
            XMLElement* cyl = geometry->FirstChildElement("cylinder");
            if (cyl && !link.mesh.loaded) {
                float radius = cyl->FloatAttribute("radius", 0.05f);
                float length = cyl->FloatAttribute("length", 0.1f);
                GenerateCylinder(link.mesh, radius, length, 24);
                link.visualMeshPath = "[cylinder]";
            }

            // Box primitive
            XMLElement* box = geometry->FirstChildElement("box");
            if (box && !link.mesh.loaded) {
                glm::vec3 size(0.1f);
                const char* sizeStr = box->Attribute("size");
                if (sizeStr) size = ParseVec3(sizeStr);
                GenerateBox(link.mesh, size);
                link.visualMeshPath = "[box]";
            }

            // Sphere primitive
            XMLElement* sph = geometry->FirstChildElement("sphere");
            if (sph && !link.mesh.loaded) {
                float radius = sph->FloatAttribute("radius", 0.05f);
                GenerateSphere(link.mesh, radius, 16, 12);
                link.visualMeshPath = "[sphere]";
            }
        }
        
        XMLElement* origin = visual->FirstChildElement("origin");
        link.visualOrigin = ParsePose(origin);
    }
    
    m_linkIndex[link.name] = m_links.size();
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
    joint.origin = ParsePose(origin);
    
    // Parse axis
    XMLElement* axisXml = jointXml->FirstChildElement("axis");
    if (axisXml) {
        const char* xyzStr = axisXml->Attribute("xyz");
        if (xyzStr) {
            joint.axis = ParseVec3(xyzStr);
        }
    }

    m_jointIndex[joint.name] = m_joints.size();
    m_joints.push_back(joint);
}

// ─────────────────────────────────────────────────────────
// Assimp Mesh Loading (STL, OBJ, DAE, FBX, etc.)
// ─────────────────────────────────────────────────────────

bool RobotModel::LoadMesh(const std::string& meshPath, MeshData& outMesh) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(meshPath,
        aiProcess_Triangulate |
        aiProcess_GenNormals |
        aiProcess_JoinIdenticalVertices |
        aiProcess_OptimizeMeshes
    );

    if (!scene || !scene->mRootNode || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) {
        spdlog::error("Assimp failed to load: {} — {}", meshPath, importer.GetErrorString());
        return false;
    }

    // Flatten all meshes into a single buffer
    outMesh.vertices.clear();
    outMesh.normals.clear();
    outMesh.indices.clear();

    uint32_t vertexOffset = 0;
    for (unsigned int m = 0; m < scene->mNumMeshes; m++) {
        aiMesh* mesh = scene->mMeshes[m];
        
        for (unsigned int v = 0; v < mesh->mNumVertices; v++) {
            outMesh.vertices.push_back(mesh->mVertices[v].x);
            outMesh.vertices.push_back(mesh->mVertices[v].y);
            outMesh.vertices.push_back(mesh->mVertices[v].z);

            if (mesh->HasNormals()) {
                outMesh.normals.push_back(mesh->mNormals[v].x);
                outMesh.normals.push_back(mesh->mNormals[v].y);
                outMesh.normals.push_back(mesh->mNormals[v].z);
            }
        }

        for (unsigned int f = 0; f < mesh->mNumFaces; f++) {
            aiFace& face = mesh->mFaces[f];
            for (unsigned int idx = 0; idx < face.mNumIndices; idx++) {
                outMesh.indices.push_back(vertexOffset + face.mIndices[idx]);
            }
        }
        
        vertexOffset += mesh->mNumVertices;
    }

    outMesh.loaded = true;
    spdlog::info("Loaded mesh: {} ({} verts, {} tris)", 
                 meshPath, outMesh.vertices.size() / 3, outMesh.indices.size() / 3);
    return true;
}

// ─────────────────────────────────────────────────────────
// Kinematic Chain
// ─────────────────────────────────────────────────────────

void RobotModel::BuildKinematicChain() {
    // Find the base link (a link that is never a child)
    std::unordered_map<std::string, bool> isChild;
    for (auto& j : m_joints) {
        isChild[j.childLinkName] = true;
    }
    
    for (auto& l : m_links) {
        if (!isChild[l.name]) {
            m_baseLinkName = l.name;
            break;
        }
    }

    // Build ordered chain from base to tip (movable joints only)
    m_chainJointIndices.clear();
    std::string currentLink = m_baseLinkName;
    
    bool found = true;
    while (found) {
        found = false;
        for (size_t i = 0; i < m_joints.size(); i++) {
            if (m_joints[i].parentLinkName == currentLink) {
                if (m_joints[i].type != "fixed") {
                    m_chainJointIndices.push_back(i);
                }
                currentLink = m_joints[i].childLinkName;
                found = true;
                break;
            }
        }
    }
    
    spdlog::info("Kinematic chain: {} movable joints, base='{}'", 
                 m_chainJointIndices.size(), m_baseLinkName);
}

// ─────────────────────────────────────────────────────────
// Getters & Setters
// ─────────────────────────────────────────────────────────

int RobotModel::GetDOF() const {
    return static_cast<int>(m_chainJointIndices.size());
}

std::vector<float> RobotModel::GetJointAngles() const {
    std::vector<float> angles;
    for (size_t idx : m_chainJointIndices) {
        angles.push_back(m_joints[idx].currentAngle);
    }
    return angles;
}

void RobotModel::SetJointAngles(const std::vector<float>& angles) {
    for (size_t i = 0; i < angles.size() && i < m_chainJointIndices.size(); i++) {
        RobotJoint& j = m_joints[m_chainJointIndices[i]];
        // Clamp to limits (if revolute)
        if (j.type == "revolute") {
            j.currentAngle = std::clamp(angles[i], j.limits.lower, j.limits.upper);
        } else {
            j.currentAngle = angles[i]; // continuous, prismatic
        }
    }
}

void RobotModel::SetJointPosition(const std::string& jointName, float angle) {
    auto it = m_jointIndex.find(jointName);
    if (it != m_jointIndex.end()) {
        m_joints[it->second].currentAngle = angle;
    }
}

glm::mat4 RobotModel::GetGlobalTransform(const std::string& linkName) const {
    glm::mat4 T(1.0f);
    
    // Walk the chain from base to the requested link
    std::string current = m_baseLinkName;
    for (auto& j : m_joints) {
        // Apply joint origin transform
        T = T * j.origin;
        
        // Apply joint rotation/translation
        if (j.type == "revolute" || j.type == "continuous") {
            T = glm::rotate(T, j.currentAngle, j.axis);
        } else if (j.type == "prismatic") {
            T = glm::translate(T, j.axis * j.currentAngle);
        }
        
        if (j.childLinkName == linkName) {
            return T;
        }
    }
    
    return T;
}

glm::mat4 RobotModel::GetEndEffectorPose() const {
    if (m_chainJointIndices.empty()) return glm::mat4(1.0f);
    
    // The last joint's child link is the end effector
    const RobotJoint& lastJoint = m_joints[m_chainJointIndices.back()];
    return GetGlobalTransform(lastJoint.childLinkName);
}

void RobotModel::Update(float deltaTime) {
    // Future: smooth joint transitions, physics, etc.
}

// ---------------------------------------------------------
// Procedural Primitive Geometry
// ---------------------------------------------------------

void RobotModel::GenerateCylinder(MeshData& mesh, float radius, float length, int segments) {
    mesh.vertices.clear();
    mesh.normals.clear();
    mesh.indices.clear();

    float halfLen = length * 0.5f;

    // Side vertices
    for (int i = 0; i <= segments; i++) {
        float angle = (float)i / (float)segments * 6.28318f;
        float cx = cosf(angle) * radius;
        float cy = sinf(angle) * radius;
        float nx = cosf(angle);
        float ny = sinf(angle);

        // Bottom vertex
        mesh.vertices.push_back(cx); mesh.vertices.push_back(cy); mesh.vertices.push_back(-halfLen);
        mesh.normals.push_back(nx); mesh.normals.push_back(ny); mesh.normals.push_back(0);

        // Top vertex
        mesh.vertices.push_back(cx); mesh.vertices.push_back(cy); mesh.vertices.push_back(halfLen);
        mesh.normals.push_back(nx); mesh.normals.push_back(ny); mesh.normals.push_back(0);
    }

    // Side indices
    for (int i = 0; i < segments; i++) {
        uint32_t b = i * 2;
        mesh.indices.push_back(b); mesh.indices.push_back(b + 1); mesh.indices.push_back(b + 2);
        mesh.indices.push_back(b + 1); mesh.indices.push_back(b + 3); mesh.indices.push_back(b + 2);
    }

    // Top cap center
    uint32_t topCenter = (uint32_t)(mesh.vertices.size() / 3);
    mesh.vertices.push_back(0); mesh.vertices.push_back(0); mesh.vertices.push_back(halfLen);
    mesh.normals.push_back(0); mesh.normals.push_back(0); mesh.normals.push_back(1);

    // Bottom cap center
    uint32_t botCenter = (uint32_t)(mesh.vertices.size() / 3);
    mesh.vertices.push_back(0); mesh.vertices.push_back(0); mesh.vertices.push_back(-halfLen);
    mesh.normals.push_back(0); mesh.normals.push_back(0); mesh.normals.push_back(-1);

    // Cap vertices and indices
    uint32_t capStart = (uint32_t)(mesh.vertices.size() / 3);
    for (int i = 0; i <= segments; i++) {
        float angle = (float)i / (float)segments * 6.28318f;
        float cx = cosf(angle) * radius;
        float cy = sinf(angle) * radius;

        // Top cap vertex
        mesh.vertices.push_back(cx); mesh.vertices.push_back(cy); mesh.vertices.push_back(halfLen);
        mesh.normals.push_back(0); mesh.normals.push_back(0); mesh.normals.push_back(1);

        // Bottom cap vertex
        mesh.vertices.push_back(cx); mesh.vertices.push_back(cy); mesh.vertices.push_back(-halfLen);
        mesh.normals.push_back(0); mesh.normals.push_back(0); mesh.normals.push_back(-1);
    }

    for (int i = 0; i < segments; i++) {
        uint32_t v = capStart + i * 2;
        mesh.indices.push_back(topCenter); mesh.indices.push_back(v); mesh.indices.push_back(v + 2);
        mesh.indices.push_back(botCenter); mesh.indices.push_back(v + 3); mesh.indices.push_back(v + 1);
    }

    mesh.loaded = true;
    spdlog::info("Generated cylinder: r={}, l={}, {} verts", radius, length, mesh.vertices.size() / 3);
}

void RobotModel::GenerateBox(MeshData& mesh, const glm::vec3& size) {
    mesh.vertices.clear();
    mesh.normals.clear();
    mesh.indices.clear();

    float hx = size.x * 0.5f, hy = size.y * 0.5f, hz = size.z * 0.5f;

    // 6 faces, 4 verts each = 24 vertices
    struct V { float x, y, z, nx, ny, nz; };
    V verts[24] = {
        // Front (+Z)
        {-hx,-hy, hz, 0, 0, 1}, { hx,-hy, hz, 0, 0, 1}, { hx, hy, hz, 0, 0, 1}, {-hx, hy, hz, 0, 0, 1},
        // Back (-Z)
        { hx,-hy,-hz, 0, 0,-1}, {-hx,-hy,-hz, 0, 0,-1}, {-hx, hy,-hz, 0, 0,-1}, { hx, hy,-hz, 0, 0,-1},
        // Right (+X)
        { hx,-hy, hz, 1, 0, 0}, { hx,-hy,-hz, 1, 0, 0}, { hx, hy,-hz, 1, 0, 0}, { hx, hy, hz, 1, 0, 0},
        // Left (-X)
        {-hx,-hy,-hz,-1, 0, 0}, {-hx,-hy, hz,-1, 0, 0}, {-hx, hy, hz,-1, 0, 0}, {-hx, hy,-hz,-1, 0, 0},
        // Top (+Y)
        {-hx, hy, hz, 0, 1, 0}, { hx, hy, hz, 0, 1, 0}, { hx, hy,-hz, 0, 1, 0}, {-hx, hy,-hz, 0, 1, 0},
        // Bottom (-Y)
        {-hx,-hy,-hz, 0,-1, 0}, { hx,-hy,-hz, 0,-1, 0}, { hx,-hy, hz, 0,-1, 0}, {-hx,-hy, hz, 0,-1, 0},
    };

    for (auto& v : verts) {
        mesh.vertices.push_back(v.x); mesh.vertices.push_back(v.y); mesh.vertices.push_back(v.z);
        mesh.normals.push_back(v.nx); mesh.normals.push_back(v.ny); mesh.normals.push_back(v.nz);
    }

    for (uint32_t face = 0; face < 6; face++) {
        uint32_t b = face * 4;
        mesh.indices.push_back(b); mesh.indices.push_back(b+1); mesh.indices.push_back(b+2);
        mesh.indices.push_back(b); mesh.indices.push_back(b+2); mesh.indices.push_back(b+3);
    }

    mesh.loaded = true;
    spdlog::info("Generated box: {}x{}x{}", size.x, size.y, size.z);
}

void RobotModel::GenerateSphere(MeshData& mesh, float radius, int rings, int sectors) {
    mesh.vertices.clear();
    mesh.normals.clear();
    mesh.indices.clear();

    for (int r = 0; r <= rings; r++) {
        float phi = 3.14159f * (float)r / (float)rings;
        for (int s = 0; s <= sectors; s++) {
            float theta = 6.28318f * (float)s / (float)sectors;
            float x = sinf(phi) * cosf(theta);
            float y = cosf(phi);
            float z = sinf(phi) * sinf(theta);

            mesh.vertices.push_back(x * radius);
            mesh.vertices.push_back(y * radius);
            mesh.vertices.push_back(z * radius);
            mesh.normals.push_back(x);
            mesh.normals.push_back(y);
            mesh.normals.push_back(z);
        }
    }

    for (int r = 0; r < rings; r++) {
        for (int s = 0; s < sectors; s++) {
            uint32_t a = r * (sectors + 1) + s;
            uint32_t b = a + sectors + 1;
            mesh.indices.push_back(a); mesh.indices.push_back(b); mesh.indices.push_back(a + 1);
            mesh.indices.push_back(b); mesh.indices.push_back(b + 1); mesh.indices.push_back(a + 1);
        }
    }

    mesh.loaded = true;
    spdlog::info("Generated sphere: r={}, {} verts", radius, mesh.vertices.size() / 3);
}
