#pragma once

#include <directx/d3d12.h>
#include <wrl.h>
#include <vector>
#include <memory>
#include "../robot/robot_model.h"
#include "dx12_mesh.h"

using Microsoft::WRL::ComPtr;

class SceneRenderer {
public:
    SceneRenderer(ID3D12Device* device);
    ~SceneRenderer();

    // Convert internal MeshData into GPU DX12Mesh objects
    void BuildFromRobot(RobotModel* robot, ID3D12GraphicsCommandList* cmdList);

    // Draw the scene (meshes, debug lines, grid, obstacles)
    void Draw(ID3D12GraphicsCommandList* cmdList, RobotModel* robot, class UIContext* ui, UINT8* cbvData);

private:
    ID3D12Device* m_device;
    
    // One DX12 geometry buffer per robot link
    struct RenderNode {
        std::unique_ptr<DX12Mesh> mesh;
        bool hasMesh;
    };
    std::vector<RenderNode> m_nodes;

    // Built-in geometry
    std::unique_ptr<DX12Mesh> m_gridMesh;
    std::unique_ptr<DX12Mesh> m_axisMesh; // For coordinate datum

    void BuildGrid(ID3D12GraphicsCommandList* cmdList);
    void BuildAxis(ID3D12GraphicsCommandList* cmdList);
    void DrawAxes(ID3D12GraphicsCommandList* cmdList, const glm::mat4& transform, UINT8* cbvData);
    
    std::unique_ptr<DX12Mesh> m_dynamicBoxMesh; // Used to draw any size box
    void BuildDynamicBox(ID3D12GraphicsCommandList* cmdList);
};
