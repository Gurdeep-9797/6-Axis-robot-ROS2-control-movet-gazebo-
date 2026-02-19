#include "ui_context.h"

// Heavy includes — only in the .cpp, NOT in header
#include "../engine/dx12_core.h"
#include <imgui_impl_win32.h>
#include <imgui_impl_dx12.h>
#include "pipeline_view.h"
#include "calibration_panel.h"

UIContext::UIContext(HWND hwnd, DX12Core* dx12, MotionEngine* motion) 
    : m_dx12(dx12), m_motion(motion) 
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
    ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
}

void UIContext::Render() {
    RenderCNCPanel();
    RenderDiagnosticsPanel();
    if (m_pipelineView) m_pipelineView->Render();
    if (m_calibrationPanel) m_calibrationPanel->Render();
}

void UIContext::EndFrame() {
    ImGui::Render();
    // Rendering handled by DX12Core callback
    
    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault(NULL, (void*)m_dx12->GetCommandList());
    }
}

void UIContext::ApplyCNCTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.13f, 1.0f);
    style.Colors[ImGuiCol_Header] = ImVec4(0.2f, 0.2f, 0.25f, 1.0f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.15f, 0.15f, 0.2f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.2f, 0.2f, 0.25f, 1.0f);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.0f, 0.4f, 0.8f, 1.0f);
}

void UIContext::RenderCNCPanel() {
    ImGui::Begin("CNC Control");
    ImGui::Text("Status: IDLE");
    ImGui::Separator();
    
    if (ImGui::Button("HOME", ImVec2(80, 50))) {}
    ImGui::SameLine();
    if (ImGui::Button("START", ImVec2(80, 50))) {}
    
    ImGui::Separator();
    ImGui::Text("JOG");
    ImGui::Button("X+", ImVec2(60, 40)); ImGui::SameLine();
    ImGui::Button("X-", ImVec2(60, 40));
    
    ImGui::End();
}

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
                ImGui::Text("%.4f", ref[i]);
                
                ImGui::TableNextColumn();
                ImGui::Text("%.4f", act[i]);
                
                ImGui::TableNextColumn();
                // Color code error based on threshold
                if (err[i] > 0.05f) ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "%.4f", err[i]);
                else ImGui::Text("%.4f", err[i]);

                ImGui::TableNextColumn();
                float pct = (std::abs(ref[i]) > 0.001f) ? (err[i] / std::abs(ref[i])) * 100.0f : 0.0f;
                ImGui::Text("%.1f%%", pct);
            }
            ImGui::EndTable();
        }
        
        // Connection Info
        ImGui::Separator();
        ImGui::Text("Pipeline State:");
        auto state = m_motion->GetPipelineState();
        ImGui::Text("Link: %s", state.outputActive ? "ACTIVE" : "IDLE");
        
    }
    ImGui::End();
}

void UIContext::RenderMenu() {}
