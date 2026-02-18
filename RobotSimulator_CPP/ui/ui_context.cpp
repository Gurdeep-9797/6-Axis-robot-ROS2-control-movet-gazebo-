#include "ui_context.h"

UIContext::UIContext(HWND hwnd, DX12Core* dx12) : m_dx12(dx12) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
    
    ApplyCNCTheme();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX12_Init(
        dx12->GetDevice(),
        3, // FrameCount
        dx12->GetBackHeaderFormat(),
        dx12->GetSRVHeap(),
        dx12->GetSRVHeap()->GetCPUDescriptorHandleForHeapStart(),
        dx12->GetSRVHeap()->GetGPUDescriptorHandleForHeapStart()
    );
}

UIContext::~UIContext() {
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void UIContext::ApplyCNCTheme() {
    auto& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.18f, 0.20f, 0.23f, 1.0f); // RGB(45, 50, 58)
    style.Colors[ImGuiCol_Button] = ImVec4(0.25f, 0.27f, 0.30f, 1.0f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.1f, 0.6f, 0.8f, 1.0f); // Blue highlight
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.08f, 0.5f, 0.7f, 1.0f);
}

void UIContext::BeginFrame() {
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
    
    // Create DockSpace
    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
}

void UIContext::RenderCNCPanel() {
    ImGui::Begin("CNC Control Panel");
    
    ImGui::Text("STATUS: IDLE");
    ImGui::Separator();
    
    // Jog Buttons
    if (ImGui::Button("X-", ImVec2(60, 60))) { /* Jog Logic */ }
    ImGui::SameLine();
    if (ImGui::Button("X+", ImVec2(60, 60))) { /* Jog Logic */ }
    
    ImGui::End();
}

void UIContext::Render() {
    RenderCNCPanel();
    ImGui::ShowDemoWindow(); // For testing
}

void UIContext::EndFrame() {
    ImGui::Render();
    // Actual rendering happens in DX12Core::Render loop, we just recording commands there
    // But wait, ImGui_ImplDX12_RenderDrawData needs a command list.
    // DX12Core needs to accept a callback or handle ImGui rendering.
}
