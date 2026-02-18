#include "pipeline_view.h"
#include <string>

PipelineView::PipelineView(MotionEngine* motion) : m_motion(motion) {}
PipelineView::~PipelineView() {}

void PipelineView::Render() {
    ImGui::Begin("Pipeline View");

    MotionEngine::PipelineState state = m_motion->GetPipelineState();
    MotionEngine::ControlMode mode = m_motion->GetMode();

    ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Layout positions
    float startX = canvas_p0.x + 50;
    float startY = canvas_p0.y + 50;
    float spacingX = 150;

    ImVec2 pInput(startX, startY);
    ImVec2 pPlanner(startX + spacingX, startY);
    ImVec2 pOutput(startX + spacingX * 2, startY);
    ImVec2 pFeedback(startX + spacingX * 2, startY + 100);
    ImVec2 pFK(startX + spacingX, startY + 100);
    
    // Draw Nodes
    DrawNode("Input", pInput, state.inputActive);
    DrawNode("Planner", pPlanner, state.plannerActive);
    
    // Split for Output based on Mode
    std::string outLabel = (mode == MotionEngine::ControlMode::Simulation) ? "Sim (ROS)" : "Real (ESP32)";
    DrawNode(outLabel.c_str(), pOutput, state.outputActive);
    
    std::string fbLabel = (mode == MotionEngine::ControlMode::Simulation) ? "JointState" : "Encoders";
    DrawNode(fbLabel.c_str(), pFeedback, state.feedbackActive);
    
    DrawNode("FK / Vis", pFK, true);

    // Draw Connections
    DrawConnection(pInput, pPlanner, state.inputActive);
    DrawConnection(pPlanner, pOutput, state.outputActive);
    
    // Feedback loop
    // Output -> Feedback -> FK
    DrawConnection(pOutput, pFeedback, state.feedbackActive);
    DrawConnection(pFeedback, pFK, true);

    ImGui::End();
}

void PipelineView::DrawNode(const char* label, const ImVec2& pos, bool active) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 size(100, 40);
    ImU32 color = active ? IM_COL32(0, 200, 50, 255) : IM_COL32(80, 80, 80, 255);
    
    draw_list->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), color, 5.0f);
    draw_list->AddText(ImVec2(pos.x + 10, pos.y + 10), IM_COL32(255, 255, 255, 255), label);
}

void PipelineView::DrawConnection(const ImVec2& p1, const ImVec2& p2, bool active) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 offset(100, 20); // Center of node
    ImU32 color = active ? IM_COL32(0, 255, 0, 255) : IM_COL32(100, 100, 100, 255);
    draw_list->AddLine(ImVec2(p1.x + offset.x, p1.y + offset.y), ImVec2(p2.x, p2.y + offset.y), color, 2.0f);
}
