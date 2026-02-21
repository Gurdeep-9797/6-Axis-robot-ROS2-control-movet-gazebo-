#include "scene_renderer.h"
#include "../ui/ui_context.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cstring>

SceneRenderer::SceneRenderer(ID3D12Device* device) : m_device(device) {
}

SceneRenderer::~SceneRenderer() {}

void SceneRenderer::BuildFromRobot(RobotModel* robot, ID3D12GraphicsCommandList* cmdList) {
    if (!robot) return;

    const auto& links = robot->GetLinks();
    m_nodes.resize(links.size());

    for (size_t i = 0; i < links.size(); i++) {
        m_nodes[i].hasMesh = false;
        
        if (links[i].mesh.loaded && !links[i].mesh.vertices.empty()) {
            
            std::vector<Vertex3D> vbuffer;
            std::vector<uint32_t> ibuffer = links[i].mesh.indices;
            
            size_t vCount = links[i].mesh.vertices.size() / 3;
            bool hasNormals = (links[i].mesh.normals.size() == links[i].mesh.vertices.size());
            
            for (size_t v = 0; v < vCount; v++) {
                Vertex3D vert;
                vert.position = glm::vec3(
                    links[i].mesh.vertices[v*3],
                    links[i].mesh.vertices[v*3+1],
                    links[i].mesh.vertices[v*3+2]
                );
                
                if (hasNormals) {
                    vert.normal = glm::vec3(
                        links[i].mesh.normals[v*3],
                        links[i].mesh.normals[v*3+1],
                        links[i].mesh.normals[v*3+2]
                    );
                } else {
                    vert.normal = glm::vec3(0,1,0);
                }
                
                // Light gray/metallic 
                vert.color = glm::vec4(0.8f, 0.85f, 0.9f, 1.0f);
                
                vbuffer.push_back(vert);
            }
            
            m_nodes[i].mesh = std::make_unique<DX12Mesh>();
            if (m_nodes[i].mesh->Create(m_device, cmdList, vbuffer, ibuffer)) {
                m_nodes[i].hasMesh = true;
            }
        }
    }

    BuildGrid(cmdList);
    BuildAxis(cmdList);
    BuildDynamicBox(cmdList);
}

void SceneRenderer::BuildGrid(ID3D12GraphicsCommandList* cmdList) {
    std::vector<Vertex3D> verts;
    std::vector<uint32_t> inds;
    
    int size = 10;
    float step = 0.5f;
    
    uint32_t idx = 0;
    for (int i = -size; i <= size; i++) {
        float p = (float)i * step;
        
        // Z line
        Vertex3D v1, v2;
        v1.position = glm::vec3(p, 0.0f, -size * step);
        v2.position = glm::vec3(p, 0.0f, size * step);
        v1.normal = v2.normal = glm::vec3(0,0,0); // Flat line
        v1.color = v2.color = glm::vec4(0.3f, 0.3f, 0.35f, 1.0f);
        verts.push_back(v1); verts.push_back(v2);
        inds.push_back(idx++); inds.push_back(idx++);
        
        // X line
        v1.position = glm::vec3(-size * step, 0.0f, p);
        v2.position = glm::vec3(size * step, 0.0f, p);
        verts.push_back(v1); verts.push_back(v2);
        inds.push_back(idx++); inds.push_back(idx++);
    }
    
    m_gridMesh = std::make_unique<DX12Mesh>();
    m_gridMesh->Create(m_device, cmdList, verts, inds);
}

void SceneRenderer::BuildAxis(ID3D12GraphicsCommandList* cmdList) {
    std::vector<Vertex3D> verts;
    std::vector<uint32_t> inds;
    
    float len = 0.15f; // 15cm axis lines
    
    Vertex3D o, x, y, z;
    o.position = glm::vec3(0,0,0); o.normal = glm::vec3(0); o.color = glm::vec4(1,1,1,1);
    
    x.position = glm::vec3(len,0,0); x.normal = glm::vec3(0); x.color = glm::vec4(1,0,0,1); // Red X
    y.position = glm::vec3(0,len,0); y.normal = glm::vec3(0); y.color = glm::vec4(0,1,0,1); // Green Y
    z.position = glm::vec3(0,0,len); z.normal = glm::vec3(0); z.color = glm::vec4(0,0,1,1); // Blue Z
    
    verts = { o, x, o, y, o, z };
    inds = { 0, 1, 2, 3, 4, 5 };
    
    m_axisMesh = std::make_unique<DX12Mesh>();
    m_axisMesh->Create(m_device, cmdList, verts, inds);
}

void SceneRenderer::BuildDynamicBox(ID3D12GraphicsCommandList* cmdList) {
    std::vector<Vertex3D> verts;
    std::vector<uint32_t> inds;
    
    // A 1x1x1 unit box centered at origin
    float h = 0.5f;
    struct V { float x, y, z, nx, ny, nz; };
    V rawVerts[24] = {
        // Front (+Z)
        {-h,-h, h, 0, 0, 1}, { h,-h, h, 0, 0, 1}, { h, h, h, 0, 0, 1}, {-h, h, h, 0, 0, 1},
        // Back (-Z)
        { h,-h,-h, 0, 0,-1}, {-h,-h,-h, 0, 0,-1}, {-h, h,-h, 0, 0,-1}, { h, h,-h, 0, 0,-1},
        // Right (+X)
        { h,-h, h, 1, 0, 0}, { h,-h,-h, 1, 0, 0}, { h, h,-h, 1, 0, 0}, { h, h, h, 1, 0, 0},
        // Left (-X)
        {-h,-h,-h,-1, 0, 0}, {-h,-h, h,-1, 0, 0}, {-h, h, h,-1, 0, 0}, {-h, h,-h,-1, 0, 0},
        // Top (+Y)
        {-h, h, h, 0, 1, 0}, { h, h, h, 0, 1, 0}, { h, h,-h, 0, 1, 0}, {-h, h,-h, 0, 1, 0},
        // Bottom (-Y)
        {-h,-h,-h, 0,-1, 0}, { h,-h,-h, 0,-1, 0}, { h,-h, h, 0,-1, 0}, {-h,-h, h, 0,-1, 0},
    };

    for (auto& v : rawVerts) {
        Vertex3D vert;
        vert.position = glm::vec3(v.x, v.y, v.z);
        vert.normal = glm::vec3(v.nx, v.ny, v.nz);
        vert.color = glm::vec4(0.8f, 0.3f, 0.1f, 0.6f); // Semi-transparent orange/red for obstacles
        verts.push_back(vert);
    }

    for (uint32_t face = 0; face < 6; face++) {
        uint32_t b = face * 4;
        inds.push_back(b); inds.push_back(b+1); inds.push_back(b+2);
        inds.push_back(b); inds.push_back(b+2); inds.push_back(b+3);
    }
    
    m_dynamicBoxMesh = std::make_unique<DX12Mesh>();
    m_dynamicBoxMesh->Create(m_device, cmdList, verts, inds);
}

void SceneRenderer::DrawAxes(ID3D12GraphicsCommandList* cmdList, const glm::mat4& transform, UINT8* cbvData) {
    if (!m_axisMesh || !m_axisMesh->IsValid()) return;
    
    // Update constant buffer Model matrix for this axis
    memcpy(cbvData, &transform[0][0], sizeof(glm::mat4));
    
    D3D12_VERTEX_BUFFER_VIEW vbv = m_axisMesh->GetVertexBufferView();
    D3D12_INDEX_BUFFER_VIEW ibv = m_axisMesh->GetIndexBufferView();
    
    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
    cmdList->IASetVertexBuffers(0, 1, &vbv);
    cmdList->IASetIndexBuffer(&ibv);
    cmdList->DrawIndexedInstanced(m_axisMesh->GetIndexCount(), 1, 0, 0, 0);
}

void SceneRenderer::Draw(ID3D12GraphicsCommandList* cmdList, RobotModel* robot, UIContext* ui, UINT8* cbvData) {
    if (!cbvData) return;

    // 1. Draw Grid
    if (m_gridMesh && m_gridMesh->IsValid()) {
        glm::mat4 identity(1.0f);
        memcpy(cbvData, &identity[0][0], sizeof(glm::mat4));
        
        D3D12_VERTEX_BUFFER_VIEW vbv = m_gridMesh->GetVertexBufferView();
        D3D12_INDEX_BUFFER_VIEW ibv = m_gridMesh->GetIndexBufferView();
        cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        cmdList->IASetVertexBuffers(0, 1, &vbv);
        cmdList->IASetIndexBuffer(&ibv);
        cmdList->DrawIndexedInstanced(m_gridMesh->GetIndexCount(), 1, 0, 0, 0);
    }
    
    // Global Origin Axis
    DrawAxes(cmdList, glm::mat4(1.0f), cbvData);

    if (!robot) return;

    // 2. Draw Robot Links & Joint Axes
    const auto& links = robot->GetLinks();
    
    for (size_t i = 0; i < links.size() && i < m_nodes.size(); i++) {
        glm::mat4 globalTransform = robot->GetGlobalTransform(links[i].name);
        
        // Draw Datum Axis for this joint
        DrawAxes(cmdList, globalTransform, cbvData);
        
        // Draw Mesh
        if (m_nodes[i].hasMesh && m_nodes[i].mesh->IsValid()) {
            // Update constant buffer with Link's global transform
            memcpy(cbvData, &globalTransform[0][0], sizeof(glm::mat4));
            
            D3D12_VERTEX_BUFFER_VIEW vbv = m_nodes[i].mesh->GetVertexBufferView();
            D3D12_INDEX_BUFFER_VIEW ibv = m_nodes[i].mesh->GetIndexBufferView();
            
            cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            cmdList->IASetVertexBuffers(0, 1, &vbv);
            cmdList->IASetIndexBuffer(&ibv);
            cmdList->DrawIndexedInstanced(m_nodes[i].mesh->GetIndexCount(), 1, 0, 0, 0);
        }
    }
    
    // 3. Draw Workspace Obstacles
    if (ui && m_dynamicBoxMesh && m_dynamicBoxMesh->IsValid()) {
        const auto& obstacles = ui->m_obstacles;
        
        D3D12_VERTEX_BUFFER_VIEW vbv = m_dynamicBoxMesh->GetVertexBufferView();
        D3D12_INDEX_BUFFER_VIEW ibv = m_dynamicBoxMesh->GetIndexBufferView();
        
        cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        cmdList->IASetVertexBuffers(0, 1, &vbv);
        cmdList->IASetIndexBuffer(&ibv);
        
        for (const auto& obs : obstacles) {
            if (!obs.active) continue;
            
            // Transform matrix: Translation * Scale
            glm::mat4 t = glm::translate(glm::mat4(1.0f), obs.position);
            glm::mat4 s = glm::scale(glm::mat4(1.0f), obs.size);
            glm::mat4 model = t * s;
            
            memcpy(cbvData, &model[0][0], sizeof(glm::mat4));
            cmdList->DrawIndexedInstanced(m_dynamicBoxMesh->GetIndexCount(), 1, 0, 0, 0);
        }
    }
}
