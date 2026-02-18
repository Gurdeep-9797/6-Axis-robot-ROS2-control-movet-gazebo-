#pragma once

#include "../engine/dx12_core.h"
#include <imgui.h>
#include <imgui_impl_win32.h>
#include <imgui_impl_dx12.h>

class PipelineView;
class CalibrationPanel;
class MotionEngine; // Forward declaration

class UIContext {
public:
    UIContext(HWND hwnd, DX12Core* dx12, MotionEngine* motion);
    ~UIContext();

    void BeginFrame();
    void Render();
    void EndFrame();

private:
    DX12Core* m_dx12;
    MotionEngine* m_motion;
    PipelineView* m_pipelineView;
    CalibrationPanel* m_calibrationPanel;
    
    void ApplyCNCTheme();
    void RenderCNCPanel();
    void RenderDiagnosticsPanel();
    void RenderMenu();
};
