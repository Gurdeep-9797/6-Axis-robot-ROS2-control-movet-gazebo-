#pragma once

#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <functional>

// Use Microsoft::WRL::ComPtr for smart pointer management
using Microsoft::WRL::ComPtr;

class DX12Core {
public:
    DX12Core(HWND hwnd, int width, int height);
    ~DX12Core();

    void Render(std::function<void(ID3D12GraphicsCommandList*)> renderCallback = nullptr);
    void Resize(int width, int height);

    // Getters for ImGui and other systems
    ID3D12Device* GetDevice() const { return m_device.Get(); }
    ID3D12CommandQueue* GetCommandQueue() const { return m_commandQueue.Get(); }
    ID3D12DescriptorHeap* GetSRVHeap() const { return m_srvHeap.Get(); }
    DXGI_FORMAT GetBackHeaderFormat() const { return DXGI_FORMAT_R8G8B8A8_UNORM; }

private:
    void InitDevice();
    void InitCommandQueue();
    void InitSwapChain(HWND hwnd, int width, int height);
    void InitDescriptorHeaps();
    void InitRenderTargets();
    void CreateCommandList();
    
    // Core DX12 Objects
    ComPtr<ID3D12Device> m_device;
    ComPtr<IDXGIFactory4> m_factory;
    ComPtr<ID3D12CommandQueue> m_commandQueue;
    ComPtr<IDXGISwapChain3> m_swapChain;
    ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
    ComPtr<ID3D12DescriptorHeap> m_srvHeap;
    ComPtr<ID3D12CommandAllocator> m_commandAllocator;
    ComPtr<ID3D12GraphicsCommandList> m_commandList;

    // Synchronization
    ComPtr<ID3D12Fence> m_fence;
    HANDLE m_fenceEvent;
    UINT64 m_fenceValue;

    // Frame Resources
    static const UINT FrameCount = 3; // Triple buffering
    ComPtr<ID3D12Resource> m_renderTargets[FrameCount];
    UINT m_rtvDescriptorSize;
    UINT m_frameIndex;

    void WaitForPreviousFrame();
};
