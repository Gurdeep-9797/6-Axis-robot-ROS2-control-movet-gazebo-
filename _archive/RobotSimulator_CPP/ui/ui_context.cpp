#include "ui_context.h"

// Heavy includes only in .cpp
#include "../engine/dx12_core.h"
#include <imgui_impl_win32.h>
#include <imgui_impl_dx12.h>
#include "pipeline_view.h"
#include "calibration_panel.h"
#include "../robot/robot_model.h"
#include "../simulation/motion_engine.h"
#include "../engine/scene_renderer.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

UIContext::UIContext(HWND hwnd, DX12Core* dx12, MotionEngine* motion, RobotModel* robot, SceneRenderer* scene) 
    : m_dx12(dx12), m_motion(motion), m_robot(robot), m_scene(scene) 
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ApplyCNCTheme();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX12_Init(dx12->GetDevice(), 3,
        dx12->GetBackHeaderFormat(), dx12->GetSRVHeap(),
        dx12->GetSRVHeap()->GetCPUDescriptorHandleForHeapStart(),
        dx12->GetSRVHeap()->GetGPUDescriptorHandleForHeapStart());
        
    m_pipelineView = new PipelineView(motion);
    m_calibrationPanel = new CalibrationPanel(motion);
}

UIContext::~UIContext() {
    delete m_pipelineView;
    delete m_calibrationPanel;
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void UIContext::BeginFrame() {
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
    ImGui::DockSpaceOverViewport(
        0,
        ImGui::GetMainViewport(),
        ImGuiDockNodeFlags_PassthruCentralNode
    );
}

void UIContext::Render() {
    RenderCNCPanel();
    RenderJointControl();
    RenderPresetPanel();
    RenderPlannerPanel();
    RenderDiagnosticsPanel();
    RenderRobotInfo();
    if (m_pipelineView) m_pipelineView->Render();
    if (m_calibrationPanel) m_calibrationPanel->Render();
}

void UIContext::EndFrame() {
    ImGui::Render();
    
    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault(NULL, (void*)m_dx12->GetCommandList());
    }
}

void UIContext::ApplyCNCTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.FramePadding = ImVec2(8, 4);
    style.ItemSpacing = ImVec2(8, 6);
    
    // Dark industrial theme
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]       = ImVec4(0.08f, 0.08f, 0.10f, 1.0f);
    colors[ImGuiCol_Header]         = ImVec4(0.18f, 0.18f, 0.22f, 1.0f);
    colors[ImGuiCol_HeaderHovered]  = ImVec4(0.25f, 0.25f, 0.30f, 1.0f);
    colors[ImGuiCol_Button]         = ImVec4(0.15f, 0.15f, 0.20f, 1.0f);
    colors[ImGuiCol_ButtonHovered]  = ImVec4(0.22f, 0.22f, 0.28f, 1.0f);
    colors[ImGuiCol_ButtonActive]   = ImVec4(0.0f, 0.45f, 0.85f, 1.0f);
    colors[ImGuiCol_FrameBg]        = ImVec4(0.12f, 0.12f, 0.15f, 1.0f);
    colors[ImGuiCol_SliderGrab]     = ImVec4(0.0f, 0.5f, 1.0f, 1.0f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.0f, 0.65f, 1.0f, 1.0f);
    colors[ImGuiCol_CheckMark]      = ImVec4(0.0f, 0.7f, 0.3f, 1.0f);
    colors[ImGuiCol_Separator]      = ImVec4(0.3f, 0.3f, 0.35f, 0.5f);
    colors[ImGuiCol_TitleBg]        = ImVec4(0.06f, 0.06f, 0.08f, 1.0f);
    colors[ImGuiCol_TitleBgActive]  = ImVec4(0.10f, 0.10f, 0.14f, 1.0f);
    colors[ImGuiCol_Tab]            = ImVec4(0.12f, 0.12f, 0.16f, 1.0f);
    colors[ImGuiCol_TabSelected]    = ImVec4(0.18f, 0.18f, 0.25f, 1.0f);
}

// ─────────────────────────────────────────────────────────
// CNC Control Panel
// ─────────────────────────────────────────────────────────
void UIContext::RenderCNCPanel() {
    ImGui::Begin("CNC Control");
    
    // Status indicator
    std::string status = m_motion->GetStatusText();
    bool moving = m_motion->IsMoving();
    
    ImVec4 statusColor = moving ? ImVec4(0.0f, 0.8f, 0.2f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
    ImGui::TextColored(statusColor, "Status: %s", status.c_str());
    
    if (moving) {
        // Pulse animation
        float pulse = (sinf((float)ImGui::GetTime() * 4.0f) + 1.0f) * 0.5f;
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, pulse, 0, 1), " [ACTIVE]");
    }
    
    ImGui::Separator();
    
    // Mode selector
    const char* modes[] = { "Simulation", "Real Hardware" };
    int currentMode = (int)m_motion->GetMode();
    if (ImGui::Combo("Mode", &currentMode, modes, 2)) {
        m_motion->SetMode((MotionEngine::ControlMode)currentMode);
    }
    
    ImGui::Separator();
    
    // Main control buttons
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.35f, 0.0f, 1.0f));
    if (ImGui::Button("HOME", ImVec2(100, 45))) {
        m_motion->Home();
    }
    ImGui::PopStyleColor();
    
    ImGui::SameLine();
    
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.2f, 0.5f, 1.0f));
    if (ImGui::Button("READY", ImVec2(100, 45))) {
        m_motion->Ready();
    }
    ImGui::PopStyleColor();
    
    ImGui::SameLine();
    
    if (moving) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.0f, 0.0f, 1.0f));
        if (ImGui::Button("STOP", ImVec2(100, 45))) {
            m_motion->Home(); // Emergency: go home
        }
        ImGui::PopStyleColor();
    }
    
    ImGui::End();
}

// ─────────────────────────────────────────────────────────
// Joint Control Panel (Sliders)
// ─────────────────────────────────────────────────────────
void UIContext::RenderJointControl() {
    ImGui::Begin("Joint Control");
    
    if (!m_robot || m_robot->GetDOF() == 0) {
        ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "No robot loaded");
        ImGui::End();
        return;
    }
    
    auto& joints = m_robot->GetJointsMutable();
    auto& chainIndices = m_robot->GetJoints(); // for limits
    int dof = m_robot->GetDOF();
    auto angles = m_robot->GetJointAngles();
    
    ImGui::Text("DOF: %d joints", dof);
    ImGui::Separator();
    
    bool changed = false;
    
    for (int i = 0; i < dof && i < (int)angles.size(); i++) {
        ImGui::PushID(i);
        
        // Find the actual joint for limits
        const auto& allJoints = m_robot->GetJoints();
        float lo = -3.14159f, hi = 3.14159f;
        
        // Search for the i-th movable joint
        int movableIdx = 0;
        for (size_t j = 0; j < allJoints.size(); j++) {
            if (allJoints[j].type != "fixed") {
                if (movableIdx == i) {
                    lo = allJoints[j].limits.lower;
                    hi = allJoints[j].limits.upper;
                    break;
                }
                movableIdx++;
            }
        }
        
        // Joint label
        float degVal = angles[i] * (180.0f / (float)M_PI);
        ImGui::Text("J%d", i + 1);
        ImGui::SameLine(40);
        
        // Slider
        char label[32];
        snprintf(label, sizeof(label), "##j%d", i);
        float val = angles[i];
        if (ImGui::SliderFloat(label, &val, lo, hi, "%.3f rad")) {
            angles[i] = val;
            changed = true;
        }
        
        // Degree readout
        ImGui::SameLine();
        ImGui::Text("%.1f deg", val * (180.0f / (float)M_PI));
        
        ImGui::PopID();
    }
    
    if (changed && !m_motion->IsMoving()) {
        m_robot->SetJointAngles(angles);
    }
    
    // End effector position
    ImGui::Separator();
    glm::mat4 ee = m_robot->GetEndEffectorPose();
    ImGui::Text("End Effector:");
    ImGui::Text("  X: %.3f  Y: %.3f  Z: %.3f", ee[3][0], ee[3][1], ee[3][2]);
    
    ImGui::End();
}

// ─────────────────────────────────────────────────────────
// Preset Movements Panel
// ─────────────────────────────────────────────────────────
void UIContext::RenderPresetPanel() {
    ImGui::Begin("Preset Movements");
    
    const auto& presets = m_motion->GetPresets();
    bool moving = m_motion->IsMoving();
    
    ImGui::Text("Click to run a preset movement sequence:");
    ImGui::Separator();
    
    for (int i = 0; i < (int)presets.size(); i++) {
        ImGui::PushID(i);
        
        // Color code by type
        ImVec4 btnColor;
        switch (i) {
            case 0: btnColor = ImVec4(0.0f, 0.4f, 0.0f, 1.0f); break; // Home = green
            case 1: btnColor = ImVec4(0.0f, 0.2f, 0.5f, 1.0f); break; // Ready = blue
            case 2: btnColor = ImVec4(0.5f, 0.3f, 0.0f, 1.0f); break; // Wave = orange
            case 3: btnColor = ImVec4(0.4f, 0.0f, 0.4f, 1.0f); break; // Pick = purple
            default: btnColor = ImVec4(0.2f, 0.2f, 0.2f, 1.0f); break;
        }
        
        ImGui::PushStyleColor(ImGuiCol_Button, btnColor);
        
        char btnText[64];
        snprintf(btnText, sizeof(btnText), "%s (%d waypoints)", 
                 presets[i].name.c_str(), (int)presets[i].waypoints.size());
        
        if (ImGui::Button(btnText, ImVec2(-1, 40))) {
            if (!moving) {
                m_motion->RunPreset(i);
            }
        }
        
        ImGui::PopStyleColor();
        ImGui::PopID();
    }
    
    if (moving) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Motion in progress...");
    }
    
    ImGui::End();
}

// ─────────────────────────────────────────────────────────
// Diagnostics Panel
// ─────────────────────────────────────────────────────────
void UIContext::RenderDiagnosticsPanel() {
    if (ImGui::Begin("Diagnostics", nullptr)) {
            
        if (ImGui::BeginTable("ErrorTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupColumn("Joint");
            ImGui::TableSetupColumn("Ref (Rad)");
            ImGui::TableSetupColumn("Act (Rad)");
            ImGui::TableSetupColumn("Err (Rad)");
            ImGui::TableSetupColumn("Err (%)");
            ImGui::TableHeadersRow();

            std::vector<float> ref = m_motion->GetReferenceJoints();
            std::vector<float> act = m_motion->GetCurrentJoints();
            std::vector<float> err = m_motion->GetJointErrors();

            for (size_t i = 0; i < 6; i++) {
                ImGui::TableNextRow();
                
                ImGui::TableNextColumn();
                ImGui::Text("J%d", (int)(i + 1));
                
                ImGui::TableNextColumn();
                float r = (i < ref.size()) ? ref[i] : 0.0f;
                ImGui::Text("%.4f", r);
                
                ImGui::TableNextColumn();
                float a = (i < act.size()) ? act[i] : 0.0f;
                ImGui::Text("%.4f", a);
                
                ImGui::TableNextColumn();
                float e = (i < err.size()) ? err[i] : 0.0f;
                if (e > 0.05f) ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "%.4f", e);
                else ImGui::Text("%.4f", e);

                ImGui::TableNextColumn();
                float pct = (std::abs(r) > 0.001f) ? (e / std::abs(r)) * 100.0f : 0.0f;
                ImGui::Text("%.1f%%", pct);
            }
            ImGui::EndTable();
        }
        
        ImGui::Separator();
        ImGui::Text("Pipeline State:");
        auto state = m_motion->GetPipelineState();
        ImGui::Text("Link: %s", state.outputActive ? "ACTIVE" : "IDLE");
    }
    ImGui::End();
}

// ─────────────────────────────────────────────────────────
// Robot Info Panel
// ─────────────────────────────────────────────────────────
void UIContext::RenderRobotInfo() {
    ImGui::Begin("Robot Info");
    
    if (!m_robot) {
        ImGui::Text("No robot loaded");
        ImGui::End();
        return;
    }
    
    const auto& links = m_robot->GetLinks();
    const auto& joints = m_robot->GetJoints();
    
    ImGui::Text("Links: %d", (int)links.size());
    ImGui::Text("Joints: %d", (int)joints.size());
    ImGui::Text("DOF: %d", m_robot->GetDOF());
    
    ImGui::Separator();
    
    if (ImGui::TreeNode("Link Details")) {
        for (auto& l : links) {
            ImGui::BulletText("%s [%s]", l.name.c_str(), 
                l.mesh.loaded ? "mesh OK" : "no mesh");
        }
        ImGui::TreePop();
    }
    
    if (ImGui::TreeNode("Joint Details")) {
        for (auto& j : joints) {
            ImGui::BulletText("%s [%s] %.1f..%.1f deg | cur: %.1f deg", 
                j.name.c_str(), j.type.c_str(),
                j.limits.lower * (180.0f / (float)M_PI),
                j.limits.upper * (180.0f / (float)M_PI),
                j.currentAngle * (180.0f / (float)M_PI));
        }
        ImGui::TreePop();
    }
    
    ImGui::End();
}

// ─────────────────────────────────────────────────────────
// Planner Panel & Workspace Obstacles
// ─────────────────────────────────────────────────────────
void UIContext::RenderPlannerPanel() {
    ImGui::Begin("Planner & Workspace");

    if (ImGui::BeginTabBar("PlannerTabs")) {
        
        // 1. Task Queue Tab
        if (ImGui::BeginTabItem("Task Queue")) {
            
            ImGui::Text("Automated Motion Planner");
            ImGui::Separator();
            
            // Queue List
            if (ImGui::BeginListBox("##Queue", ImVec2(-FLT_MIN, 150))) {
                for (size_t i = 0; i < m_taskQueue.size(); i++) {
                    const bool is_selected = false;
                    char buf[128];
                    snprintf(buf, sizeof(buf), "%d. %s [%.1fs]", (int)i+1, m_taskQueue[i].name.c_str(), m_taskQueue[i].time);
                    if (ImGui::Selectable(buf, is_selected)) {
                        // Select logic
                    }
                }
                ImGui::EndListBox();
            }
            
            // Queue Controls
            if (ImGui::Button("Add Current Pose to Queue")) {
                PlannerTask task;
                task.name = "MoveToPose_" + std::to_string(m_taskQueue.size() + 1);
                task.targetJoints = m_robot->GetJointAngles();
                task.time = 2.0f;
                m_taskQueue.push_back(task);
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Clear Queue")) {
                m_taskQueue.clear();
            }
            
            ImGui::Separator();
            
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0.4f, 0, 1));
            if (ImGui::Button("EXECUTE PLAN", ImVec2(-FLT_MIN, 40))) {
                // To safely execute, we could clear MotionEngine's queue and inject these
                // For now, it's a UI scaffold for the "Planner Tasks" feature request
            }
            ImGui::PopStyleColor();
            
            ImGui::EndTabItem();
        }
        
        // 2. Obstacles Tab
        if (ImGui::BeginTabItem("Workspace Obstacles")) {
            
            ImGui::Text("Define collision geometry in workspace:");
            ImGui::Separator();
            
            // List existing obstacles
            for (size_t i = 0; i < m_obstacles.size(); i++) {
                ImGui::PushID((int)i);
                ImGui::Checkbox(m_obstacles[i].name.c_str(), &m_obstacles[i].active);
                if (m_obstacles[i].active) {
                    ImGui::Indent();
                    ImGui::DragFloat3("Pos (m)", &m_obstacles[i].position.x, 0.05f);
                    ImGui::DragFloat3("Size (m)", &m_obstacles[i].size.x, 0.05f, 0.01f, 5.0f);
                    ImGui::Unindent();
                }
                ImGui::PopID();
                ImGui::Separator();
            }
            
            if (ImGui::Button("Add Box Obstacle")) {
                Obstacle o;
                o.name = "Obstacle " + std::to_string(m_obstacles.size() + 1);
                o.active = true;
                o.position = glm::vec3(0.5f, 0.5f, 0.5f);
                o.size = glm::vec3(0.2f, 0.2f, 0.2f);
                m_obstacles.push_back(o);
                
                // TODO: Pass to SceneRenderer to actually draw it
            }
            
            ImGui::EndTabItem();
        }
        
        ImGui::EndTabBar();
    }
    
    ImGui::End();
}

void UIContext::RenderMenu() {}
