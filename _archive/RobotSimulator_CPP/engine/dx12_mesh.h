#pragma once

#include <directx/d3d12.h>
#include <directx/d3dx12.h>
#include <dxgi1_6.h>
#include <wrl.h>
#include <glm/glm.hpp>
#include <vector>

using Microsoft::WRL::ComPtr;

// 3D Vertex structure matching shaders.hlsl
struct Vertex3D {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec4 color;
};

// Constant Buffer for Scene/Lighting
struct SceneConstantBuffer {
    glm::mat4 model;
    glm::mat4 view;
    glm::mat4 projection;
    glm::vec4 ambientColor;
    glm::vec4 directionalLightDir;
    glm::vec4 directionalLightColor;
};

// Represents a renderable 3D mesh in DX12
class DX12Mesh {
public:
    DX12Mesh();
    ~DX12Mesh();

    bool Create(ID3D12Device* device, ID3D12GraphicsCommandList* cmdList,
                const std::vector<Vertex3D>& vertices, const std::vector<uint32_t>& indices);

    D3D12_VERTEX_BUFFER_VIEW GetVertexBufferView() const { return m_vertexBufferView; }
    D3D12_INDEX_BUFFER_VIEW GetIndexBufferView() const { return m_indexBufferView; }
    UINT GetIndexCount() const { return m_indexCount; }
    bool IsValid() const { return m_vertexBuffer != nullptr && m_indexBuffer != nullptr; }

private:
    ComPtr<ID3D12Resource> m_vertexBuffer;
    ComPtr<ID3D12Resource> m_indexBuffer;
    ComPtr<ID3D12Resource> m_vertexUploadBuffer;
    ComPtr<ID3D12Resource> m_indexUploadBuffer;

    D3D12_VERTEX_BUFFER_VIEW m_vertexBufferView;
    D3D12_INDEX_BUFFER_VIEW m_indexBufferView;
    UINT m_indexCount;
};
