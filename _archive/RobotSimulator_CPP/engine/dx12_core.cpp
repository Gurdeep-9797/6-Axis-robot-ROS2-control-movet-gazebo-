#include "dx12_core.h"
#include <d3dcompiler.h>
#include <iostream>
#include <vector>
#include <stdexcept>
#include "dx12_mesh.h"

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

DX12Core::DX12Core() : m_frameIndex(0), m_fenceValue(0), m_cbvDataBegin(nullptr), m_fenceEvent(nullptr) {
}

bool DX12Core::Initialize(HWND hwnd, int width, int height) {
    HMODULE hModule = LoadLibraryA("d3dcompiler_47.dll");
    if (!hModule) {
        MessageBoxA(nullptr, "Missing d3dcompiler_47.dll on this system. Cannot compile shaders dynamically.", "DX12 Runtime Error", MB_ICONERROR | MB_OK);
        return false;
    }

    if (!InitDevice()) return false;
    if (!InitCommandQueue()) return false;
    if (!InitSwapChain(hwnd, width, height)) return false;
    if (!InitDescriptorHeaps()) return false;
    if (!InitRenderTargets()) return false;
    if (!InitDepthBuffer(width, height)) return false;
    if (!InitRootSignature()) return false;
    if (!InitShaders()) return false;
    if (!InitPipelineState()) return false;
    if (!InitConstantBuffer()) return false;
    if (!CreateCommandList()) return false;

    // Create sync objects
    if (FAILED(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)))) {
        MessageBoxA(nullptr, "Failed to create DX12 sync fence.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    m_fenceValue = 1;
    m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (m_fenceEvent == nullptr) {
        MessageBoxA(nullptr, "Failed to create DX12 sync fence event.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    return true;
}

DX12Core::~DX12Core() {
    WaitForPreviousFrame();
    if (m_fenceEvent) {
        CloseHandle(m_fenceEvent);
    }
}

bool DX12Core::InitDevice() {
    UINT dxgiFactoryFlags = 0;
// Unconditionally enable debug layer for troubleshooting!
    ComPtr<ID3D12Debug> debugController;
    if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController)))) {
        debugController->EnableDebugLayer();
        dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
    }

    if (FAILED(CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&m_factory)))) {
        MessageBoxA(nullptr, "Failed to create DXGI factory", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    if (FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&m_device)))) {
        ComPtr<IDXGIAdapter> warpAdapter;
        m_factory->EnumWarpAdapter(IID_PPV_ARGS(&warpAdapter));
        if (FAILED(D3D12CreateDevice(warpAdapter.Get(), D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&m_device)))) {
            MessageBoxA(nullptr, "Failed to create DX12 device (Hardware and WARP unsupported or disabled)", "DX12 Init Error", MB_ICONERROR | MB_OK);
            return false;
        }
    }
    return true;
}

bool DX12Core::InitCommandQueue() {
    D3D12_COMMAND_QUEUE_DESC queueDesc = {};
    queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;

    if (FAILED(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_commandQueue)))) {
        MessageBoxA(nullptr, "Failed to create DX12 command queue", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    return true;
}

bool DX12Core::InitSwapChain(HWND hwnd, int width, int height) {
    DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
    swapChainDesc.BufferCount = FrameCount;
    swapChainDesc.Width = width;
    swapChainDesc.Height = height;
    swapChainDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    swapChainDesc.SampleDesc.Count = 1;

    ComPtr<IDXGISwapChain1> swapChain;
    if (FAILED(m_factory->CreateSwapChainForHwnd(
        m_commandQueue.Get(),
        hwnd,
        &swapChainDesc,
        nullptr,
        nullptr,
        &swapChain
    ))) {
        MessageBoxA(nullptr, "Failed to create DXGI swap chain", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    swapChain.As(&m_swapChain);
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
    return true;
}

bool DX12Core::InitDescriptorHeaps() {
    D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
    rtvHeapDesc.NumDescriptors = FrameCount;
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
    if (FAILED(m_device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)))) {
        MessageBoxA(nullptr, "Failed to create RTV descriptor heap", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

    // DSV Heap
    D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
    dsvHeapDesc.NumDescriptors = 1;
    dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
    dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
    if (FAILED(m_device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&m_dsvHeap)))) {
        MessageBoxA(nullptr, "Failed to create DSV descriptor heap", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    // SRV Heap (for ImGui and Constant Buffer)
    D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
    srvHeapDesc.NumDescriptors = 2; // index 0 for ImGui, index 1 for CBV
    srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(m_device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)))) {
        MessageBoxA(nullptr, "Failed to create SRV descriptor heap", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    return true;
}

bool DX12Core::InitRenderTargets() {
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart());

    for (UINT i = 0; i < FrameCount; i++) {
        if (FAILED(m_swapChain->GetBuffer(i, IID_PPV_ARGS(&m_renderTargets[i])))) {
            MessageBoxA(nullptr, "Failed to get swap chain buffer", "DX12 Init Error", MB_ICONERROR | MB_OK);
            return false;
        }
        m_device->CreateRenderTargetView(m_renderTargets[i].Get(), nullptr, rtvHandle);
        rtvHandle.Offset(1, m_rtvDescriptorSize);
    }
    return true;
}

bool DX12Core::InitDepthBuffer(int width, int height) {
    if (width <= 0) width = 1;
    if (height <= 0) height = 1;

    CD3DX12_HEAP_PROPERTIES heapProps(D3D12_HEAP_TYPE_DEFAULT);
    CD3DX12_RESOURCE_DESC depthDesc = CD3DX12_RESOURCE_DESC::Tex2D(
        DXGI_FORMAT_D32_FLOAT, width, height, 1, 0, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL);

    D3D12_CLEAR_VALUE clearValue = {};
    clearValue.Format = DXGI_FORMAT_D32_FLOAT;
    clearValue.DepthStencil.Depth = 1.0f;
    clearValue.DepthStencil.Stencil = 0;

    if (FAILED(m_device->CreateCommittedResource(
        &heapProps, D3D12_HEAP_FLAG_NONE, &depthDesc,
        D3D12_RESOURCE_STATE_DEPTH_WRITE, &clearValue, IID_PPV_ARGS(&m_depthStencil)))) {
        MessageBoxA(nullptr, "Failed to create depth stencil buffer", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
    dsvDesc.Format = DXGI_FORMAT_D32_FLOAT;
    dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    dsvDesc.Flags = D3D12_DSV_FLAG_NONE;

    m_device->CreateDepthStencilView(m_depthStencil.Get(), &dsvDesc, m_dsvHeap->GetCPUDescriptorHandleForHeapStart());
    return true;
}

bool DX12Core::InitRootSignature() {
    CD3DX12_DESCRIPTOR_RANGE1 ranges[1];
    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 1, 0, 0, D3D12_DESCRIPTOR_RANGE_FLAG_DATA_STATIC);

    CD3DX12_ROOT_PARAMETER1 rootParameters[1];
    rootParameters[0].InitAsDescriptorTable(1, &ranges[0], D3D12_SHADER_VISIBILITY_ALL);

    D3D12_ROOT_SIGNATURE_FLAGS rootSignatureFlags =
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS;

    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc;
    rootSignatureDesc.Init_1_1(_countof(rootParameters), rootParameters, 0, nullptr, rootSignatureFlags);

    ComPtr<ID3DBlob> signature;
    ComPtr<ID3DBlob> error;
    if (FAILED(D3DX12SerializeVersionedRootSignature(&rootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1_1, &signature, &error))) {
        std::string err = "Failed to serialize versioned root signature.\n\n";
        if (error) err += (char*)error->GetBufferPointer();
        MessageBoxA(nullptr, err.c_str(), "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    
    if (FAILED(m_device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&m_rootSignature)))) {
        MessageBoxA(nullptr, "Failed to create root signature.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    return true;
}

bool DX12Core::InitShaders() {
#if defined(_DEBUG)
    UINT compileFlags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;
#else
    UINT compileFlags = 0;
#endif

    // Compute absolute path dynamically to prevent C0000005 file-not-found shader crashes
    char pathBytes[MAX_PATH];
    GetModuleFileNameA(NULL, pathBytes, MAX_PATH);
    std::string exePath(pathBytes);
    std::string exeDir = exePath.substr(0, exePath.find_last_of("\\/"));
    std::string shaderPath = exeDir + "/assets/shaders.hlsl";
    std::wstring wShaderPath(shaderPath.begin(), shaderPath.end());

    ComPtr<ID3DBlob> errorBlob;

    HRESULT hr = D3DCompileFromFile(wShaderPath.c_str(), nullptr, D3D_COMPILE_STANDARD_FILE_INCLUDE, "VSMain", "vs_5_0", compileFlags, 0, &m_vertexShader, &errorBlob);
    if (FAILED(hr)) {
        std::string err = "Failed to compile Vertex Shader: " + shaderPath + "\n\n";
        if (errorBlob) err += (char*)errorBlob->GetBufferPointer();
        MessageBoxA(nullptr, err.c_str(), "Vertex Shader Compilation Error", MB_ICONERROR | MB_OK);
        return false;
    }

    errorBlob.Reset();
    hr = D3DCompileFromFile(wShaderPath.c_str(), nullptr, D3D_COMPILE_STANDARD_FILE_INCLUDE, "PSMain", "ps_5_0", compileFlags, 0, &m_pixelShader, &errorBlob);
    if (FAILED(hr)) {
        std::string err = "Failed to compile Pixel Shader: " + shaderPath + "\n\n";
        if (errorBlob) err += (char*)errorBlob->GetBufferPointer();
        MessageBoxA(nullptr, err.c_str(), "Pixel Shader Compilation Error", MB_ICONERROR | MB_OK);
        return false;
    }

    if (!m_vertexShader || !m_pixelShader) {
        MessageBoxA(nullptr, "Shader compilation failed silently. Check logs for details.", "DX12 Initialization Error", MB_ICONERROR | MB_OK);
        return false;
    }
    return true;
}

bool DX12Core::InitPipelineState() {
    if (!m_rootSignature || !m_vertexShader || !m_pixelShader) {
        MessageBoxA(nullptr, "Cannot build Pipeline State Object: Missing Shaders or Root Signature", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
    };

    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = { inputElementDescs, _countof(inputElementDescs) };
    psoDesc.pRootSignature = m_rootSignature.Get();
    psoDesc.VS = CD3DX12_SHADER_BYTECODE(m_vertexShader.Get());
    psoDesc.PS = CD3DX12_SHADER_BYTECODE(m_pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_BACK;
    psoDesc.RasterizerState.AntialiasedLineEnable = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    
    psoDesc.DepthStencilState.DepthEnable = TRUE;
    psoDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
    psoDesc.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    psoDesc.DepthStencilState.StencilEnable = FALSE;
    
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = 1;

    if (FAILED(m_device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_pipelineState)))) {
        MessageBoxA(nullptr, "Failed to create ID3D12PipelineState (CreateGraphicsPipelineState aborted).", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    return true;
}

bool DX12Core::InitConstantBuffer() {
    const UINT cbSize = (sizeof(SceneConstantBuffer) + 255) & ~255; // 256-byte aligned

    CD3DX12_HEAP_PROPERTIES uploadHeap(D3D12_HEAP_TYPE_UPLOAD);
    CD3DX12_RESOURCE_DESC cbDesc = CD3DX12_RESOURCE_DESC::Buffer(cbSize);

    if (FAILED(m_device->CreateCommittedResource(
        &uploadHeap, D3D12_HEAP_FLAG_NONE, &cbDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&m_constantBuffer)))) {
        MessageBoxA(nullptr, "Failed to create Constant Buffer resource.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    // Map and keep mapped
    CD3DX12_RANGE readRange(0, 0);
    if (FAILED(m_constantBuffer->Map(0, &readRange, reinterpret_cast<void**>(&m_cbvDataBegin)))) {
        MessageBoxA(nullptr, "Failed to map Constant Buffer.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }

    // Create CBV descriptor (offset 1, ImGui is 0)
    D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
    cbvDesc.BufferLocation = m_constantBuffer->GetGPUVirtualAddress();
    cbvDesc.SizeInBytes = cbSize;

    CD3DX12_CPU_DESCRIPTOR_HANDLE cbvHandle(m_srvHeap->GetCPUDescriptorHandleForHeapStart());
    cbvHandle.Offset(1, m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV));
    m_device->CreateConstantBufferView(&cbvDesc, cbvHandle);
    
    return true;
}

bool DX12Core::CreateCommandList() {
    if (FAILED(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_commandAllocator)))) {
        MessageBoxA(nullptr, "Failed to create command allocator.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    
    if (FAILED(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_commandAllocator.Get(), m_pipelineState.Get(), IID_PPV_ARGS(&m_commandList)))) {
        MessageBoxA(nullptr, "Failed to create command list.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    
    if (FAILED(m_commandList->Close())) {
        MessageBoxA(nullptr, "Failed to close command list during initialization.", "DX12 Init Error", MB_ICONERROR | MB_OK);
        return false;
    }
    
    return true;
}

void DX12Core::WaitForPreviousFrame() {
    const UINT64 fence = m_fenceValue;
    m_commandQueue->Signal(m_fence.Get(), fence);
    m_fenceValue++;

    if (m_fence->GetCompletedValue() < fence) {
        m_fence->SetEventOnCompletion(fence, m_fenceEvent);
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
}

void DX12Core::Render(std::function<void(ID3D12GraphicsCommandList*)> renderCallback) {
    std::cout << "DX12: Resetting allocator..." << std::endl;
    if (FAILED(m_commandAllocator->Reset())) return;
    std::cout << "DX12: Resetting command list..." << std::endl;
    if (FAILED(m_commandList->Reset(m_commandAllocator.Get(), m_pipelineState.Get()))) return;

    // Set Root Signature and Descriptors
    std::cout << "DX12: Setting descriptor heaps..." << std::endl;
    m_commandList->SetGraphicsRootSignature(m_rootSignature.Get());
    ID3D12DescriptorHeap* ppHeaps[] = { m_srvHeap.Get() };
    m_commandList->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);

    CD3DX12_GPU_DESCRIPTOR_HANDLE cbvSrvHandle(m_srvHeap->GetGPUDescriptorHandleForHeapStart());
    cbvSrvHandle.Offset(1, m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV));
    m_commandList->SetGraphicsRootDescriptorTable(0, cbvSrvHandle); // CBV is at table param 0, offset 1

    // Transition back buffer to render target
    CD3DX12_RESOURCE_BARRIER barrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_renderTargets[m_frameIndex].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
    m_commandList->ResourceBarrier(1, &barrier);

    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_frameIndex, m_rtvDescriptorSize);
    CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(m_dsvHeap->GetCPUDescriptorHandleForHeapStart());
    m_commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);

    const float clearColor[] = { 0.117f, 0.133f, 0.157f, 1.0f }; // Industrial Dark Gray
    m_commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    m_commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

    // Call external 3D rendering (Draw meshes here)
    if (renderCallback) {
        renderCallback(m_commandList.Get());
    }

    // Transition to Present
    barrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_renderTargets[m_frameIndex].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
    m_commandList->ResourceBarrier(1, &barrier);

    std::cout << "DX12: Closing command list..." << std::endl;
    m_commandList->Close();

    std::cout << "DX12: Executing command lists..." << std::endl;
    ID3D12CommandList* ppCommandLists[] = { m_commandList.Get() };
    m_commandQueue->ExecuteCommandLists(_countof(ppCommandLists), ppCommandLists);
    
    std::cout << "DX12: Presenting swap chain..." << std::endl;
    m_swapChain->Present(1, 0);

    std::cout << "DX12: Waiting for previous frame..." << std::endl;
    WaitForPreviousFrame();
    std::cout << "DX12: Frame complete." << std::endl;
}

void DX12Core::Resize(int width, int height) {
    WaitForPreviousFrame();
    
    for (int i = 0; i < FrameCount; ++i) {
        m_renderTargets[i].Reset();
    }
    m_depthStencil.Reset();

    m_swapChain->ResizeBuffers(FrameCount, width, height, DXGI_FORMAT_UNKNOWN, 0);
    m_frameIndex = 0;

    InitRenderTargets();
    InitDepthBuffer(width, height);
}

void DX12Core::FlushCommandList() {
    m_commandList->Close();
    ID3D12CommandList* ppCommandLists[] = { m_commandList.Get() };
    m_commandQueue->ExecuteCommandLists(_countof(ppCommandLists), ppCommandLists);
    WaitForPreviousFrame();
    // Leave command list in CLOSED state — Render() will reset it
}

