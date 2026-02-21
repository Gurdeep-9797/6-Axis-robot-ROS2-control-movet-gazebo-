#include "dx12_mesh.h"
#include <stdexcept>

DX12Mesh::DX12Mesh() : m_indexCount(0) {
    m_vertexBufferView = {};
    m_indexBufferView = {};
}

DX12Mesh::~DX12Mesh() {}

bool DX12Mesh::Create(ID3D12Device* device, ID3D12GraphicsCommandList* cmdList,
                      const std::vector<Vertex3D>& vertices, const std::vector<uint32_t>& indices) 
{
    if (vertices.empty() || indices.empty()) return false;

    // 1. Create Vertex Buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex3D));

    // Create the default (GPU) buffer
    CD3DX12_HEAP_PROPERTIES defaultHeap(D3D12_HEAP_TYPE_DEFAULT);
    CD3DX12_RESOURCE_DESC vDesc = CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize);
    device->CreateCommittedResource(
        &defaultHeap, D3D12_HEAP_FLAG_NONE, &vDesc,
        D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&m_vertexBuffer));

    // Create the upload (CPU) buffer
    CD3DX12_HEAP_PROPERTIES uploadHeap(D3D12_HEAP_TYPE_UPLOAD);
    device->CreateCommittedResource(
        &uploadHeap, D3D12_HEAP_FLAG_NONE, &vDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&m_vertexUploadBuffer));

    // Copy data to upload buffer
    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0); // We do not intend to read this resource on the CPU.
    m_vertexUploadBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    m_vertexUploadBuffer->Unmap(0, nullptr);

    // Schedule copy from upload to default
    cmdList->CopyBufferRegion(m_vertexBuffer.Get(), 0, m_vertexUploadBuffer.Get(), 0, vertexBufferSize);

    // Transition from copy dest to vertex buffer state
    CD3DX12_RESOURCE_BARRIER vBarrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_vertexBuffer.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER);
    cmdList->ResourceBarrier(1, &vBarrier);

    m_vertexBufferView.BufferLocation = m_vertexBuffer->GetGPUVirtualAddress();
    m_vertexBufferView.StrideInBytes = sizeof(Vertex3D);
    m_vertexBufferView.SizeInBytes = vertexBufferSize;

    // 2. Create Index Buffer
    m_indexCount = static_cast<UINT>(indices.size());
    const UINT indexBufferSize = m_indexCount * sizeof(uint32_t);

    CD3DX12_RESOURCE_DESC iDesc = CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize);
    device->CreateCommittedResource(
        &defaultHeap, D3D12_HEAP_FLAG_NONE, &iDesc,
        D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&m_indexBuffer));

    device->CreateCommittedResource(
        &uploadHeap, D3D12_HEAP_FLAG_NONE, &iDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&m_indexUploadBuffer));

    UINT8* pIndexDataBegin;
    m_indexUploadBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    m_indexUploadBuffer->Unmap(0, nullptr);

    cmdList->CopyBufferRegion(m_indexBuffer.Get(), 0, m_indexUploadBuffer.Get(), 0, indexBufferSize);

    CD3DX12_RESOURCE_BARRIER iBarrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_indexBuffer.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_INDEX_BUFFER);
    cmdList->ResourceBarrier(1, &iBarrier);

    m_indexBufferView.BufferLocation = m_indexBuffer->GetGPUVirtualAddress();
    m_indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    m_indexBufferView.SizeInBytes = indexBufferSize;

    return true;
}
