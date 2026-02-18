#pragma once

#include "../engine/dx12_core.h"
#include <imgui.h>
#include <backends/imgui_impl_win32.h>
#include <backends/imgui_impl_dx12.h>

class UIContext {
public:
    UIContext(HWND hwnd, DX12Core* dx12);
    ~UIContext();

    void BeginFrame();
    void Render();
    void EndFrame();

private:
    DX12Core* m_dx12;
    void ApplyCNCTheme();
    void RenderCNCPanel();
    void RenderMenu();
};
