#pragma once

#include "ui_context.h"
#include "../simulation/motion_engine.h"

class PipelineView {
public:
    PipelineView(MotionEngine* motion);
    ~PipelineView();

    void Render();

private:
    MotionEngine* m_motion;
    
    // Helper to draw connection lines
    void DrawConnection(const ImVec2& p1, const ImVec2& p2, bool active);
    void DrawNode(const char* label, const ImVec2& pos, bool active);
};
