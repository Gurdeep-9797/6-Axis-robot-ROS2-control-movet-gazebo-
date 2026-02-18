#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <imgui.h>

// Forward declarations — avoid pulling DX12 headers into every UI file
class DX12Core;
class PipelineView;
class CalibrationPanel;
class MotionEngine;

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
