#include "dx12_core.h"
#include <d3dcompiler.h>
#include <iostream>

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

DX12Core::DX12Core(HWND hwnd, int width, int height) : m_frameIndex(0), m_fenceValue(0) {
    InitDevice();
    InitCommandQueue();
    InitSwapChain(hwnd, width, height);
    InitDescriptorHeaps();
    InitRenderTargets();
    CreateCommandList();

    // Create sync objects
    if (FAILED(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)))) {
        throw std::runtime_error("Failed to create fence");
    }
    m_fenceValue = 1;
    m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
    if (m_fenceEvent == nullptr) {
        throw std::runtime_error("Failed to create fence event");
    }
}

DX12Core::~DX12Core() {
    WaitForPreviousFrame();
    CloseHandle(m_fenceEvent);
}

void DX12Core::InitDevice() {
    UINT dxgiFactoryFlags = 0;
#if defined(_DEBUG)
    ComPtr<ID3D12Debug> debugController;
    if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController)))) {
        debugController->EnableDebugLayer();
        dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
    }
#endif

    if (FAILED(CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&m_factory)))) {
        throw std::runtime_error("Failed to create DXGI factory");
    }

    if (FAILED(D3D12CreateDevice(nullptr, D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&m_device)))) {
        // Fallback to WARP if hardware not available (unlikely for DX12 ready PC)
        ComPtr<IDXGIAdapter> warpAdapter;
        m_factory->EnumWarpAdapter(IID_PPV_ARGS(&warpAdapter));
        if (FAILED(D3D12CreateDevice(warpAdapter.Get(), D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&m_device)))) {
            throw std::runtime_error("Failed to create DX12 device");
        }
    }
}

void DX12Core::InitCommandQueue() {
    D3D12_COMMAND_QUEUE_DESC queueDesc = {};
    queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;

    if (FAILED(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_commandQueue)))) {
        throw std::runtime_error("Failed to create command queue");
    }
}

void DX12Core::InitSwapChain(HWND hwnd, int width, int height) {
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
        throw std::runtime_error("Failed to create swap chain");
    }

    swapChain.As(&m_swapChain);
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
}

void DX12Core::InitDescriptorHeaps() {
    D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
    rtvHeapDesc.NumDescriptors = FrameCount;
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;

    if (FAILED(m_device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)))) {
        throw std::runtime_error("Failed to create RTV descriptor heap");
    }

    m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

    // SRV Heap (for ImGui)
    D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
    srvHeapDesc.NumDescriptors = 1;
    srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(m_device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)))) {
        throw std::runtime_error("Failed to create SRV descriptor heap");
    }
}

void DX12Core::InitRenderTargets() {
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart());

    for (UINT i = 0; i < FrameCount; i++) {
        if (FAILED(m_swapChain->GetBuffer(i, IID_PPV_ARGS(&m_renderTargets[i])))) {
            throw std::runtime_error("Failed to get swap chain buffer");
        }
        m_device->CreateRenderTargetView(m_renderTargets[i].Get(), nullptr, rtvHandle);
        rtvHandle.Offset(1, m_rtvDescriptorSize);
    }
}

void DX12Core::CreateCommandList() {
    if (FAILED(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&m_commandAllocator)))) {
        throw std::runtime_error("Failed to create command allocator");
    }

    if (FAILED(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_commandAllocator.Get(), nullptr, IID_PPV_ARGS(&m_commandList)))) {
        throw std::runtime_error("Failed to create command list");
    }

    m_commandList->Close();
}

void DX12Core::WaitForPreviousFrame() {
    // WAITING FOR THE FRAME TO COMPLETE BEFORE CONTINUING IS NOT BEST PRACTICE FOR PERFORMANCE,
    // BUT SIMPLEST FOR SYNCHRONIZATION.
    
    const UINT64 fence = m_fenceValue;
    if (FAILED(m_commandQueue->Signal(m_fence.Get(), fence))) {
        throw std::runtime_error("Failed to signal fence");
    }
    m_fenceValue++;

    if (m_fence->GetCompletedValue() < fence) {
        if (FAILED(m_fence->SetEventOnCompletion(fence, m_fenceEvent))) {
            throw std::runtime_error("Failed to set fence event");
        }
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }

    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
}

void DX12Core::Render(std::function<void(ID3D12GraphicsCommandList*)> renderCallback) {
    // Basic Render Loop
    // 1. Reset Command Allocator and List
    // 2. Clear Render Target
    // 3. Close List
    // 4. Calculate Command List
    // 5. Present

    if (FAILED(m_commandAllocator->Reset())) return;
    if (FAILED(m_commandList->Reset(m_commandAllocator.Get(), nullptr))) return;

    // Transition back buffer to render target
    CD3DX12_RESOURCE_BARRIER barrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_renderTargets[m_frameIndex].Get(),
        D3D12_RESOURCE_STATE_PRESENT,
        D3D12_RESOURCE_STATE_RENDER_TARGET
    );
    m_commandList->ResourceBarrier(1, &barrier);

    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_frameIndex, m_rtvDescriptorSize);
    m_commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, nullptr);

    // Clear background (Industrial Dark Gray)
    const float clearColor[] = { 0.117f, 0.133f, 0.157f, 1.0f }; // RGB(30, 34, 40)
    m_commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);

    // Provide descriptor heap for ImGui if needed (assuming user sets it, or we set it here if we know it)
    // Actually, ImGui_ImplDX12_RenderDrawData needs the heap set. 
    // We should let the callback handle that, or set it if we have one.
    ID3D12DescriptorHeap* heaps[] = { m_srvHeap.Get() };
    m_commandList->SetDescriptorHeaps(1, heaps);

    // Execute Callback (ImGui, Scene, etc)
    if (renderCallback) {
        renderCallback(m_commandList.Get());
    }

    // Transition back to present
    barrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_renderTargets[m_frameIndex].Get(),
        D3D12_RESOURCE_STATE_RENDER_TARGET,
        D3D12_RESOURCE_STATE_PRESENT
    );
    m_commandList->ResourceBarrier(1, &barrier);

    m_commandList->Close();

    ID3D12CommandList* ppCommandLists[] = { m_commandList.Get() };
    m_commandQueue->ExecuteCommandLists(_countof(ppCommandLists), ppCommandLists);

    m_swapChain->Present(1, 0); // VSync ON

    WaitForPreviousFrame();
}

void DX12Core::Resize(int width, int height) {
    WaitForPreviousFrame();
    
    // Release buffers
    for (int i = 0; i < FrameCount; ++i) {
        m_renderTargets[i].Reset();
    }

    m_swapChain->ResizeBuffers(FrameCount, width, height, DXGI_FORMAT_UNKNOWN, 0);
    m_frameIndex = 0;

    InitRenderTargets();
}
