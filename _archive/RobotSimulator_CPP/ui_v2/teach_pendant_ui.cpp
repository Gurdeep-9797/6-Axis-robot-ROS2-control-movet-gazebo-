#include "teach_pendant_ui.h"

#include "../engine/dx12_core.h"
#include <imgui_impl_win32.h>
#include <imgui_impl_dx12.h>
#include "../robot/robot_model.h"
#include "../simulation/motion_engine.h"
#include "../engine/scene_renderer.h"
#include <cmath>
#include <sstream>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────
TeachPendantUI::TeachPendantUI(HWND hwnd, DX12Core* dx12, MotionEngine* motion, RobotModel* robot, SceneRenderer* scene)
    : m_dx12(dx12), m_motion(motion), m_robot(robot), m_scene(scene)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ApplyFRTheme();

    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX12_Init(dx12->GetDevice(), 3,
        dx12->GetBackHeaderFormat(), dx12->GetSRVHeap(),
        dx12->GetSRVHeap()->GetCPUDescriptorHandleForHeapStart(),
        dx12->GetSRVHeap()->GetGPUDescriptorHandleForHeapStart());

    // Wire program execution to motion engine
    m_program.SetExecutionCallback([this](const ProgramInstruction& instr) {
        if (!m_motion || !m_robot) return;
        
        switch (instr.type) {
            case InstructionType::PTP: {
                auto* pt = m_program.FindPoint(instr.pointName);
                if (pt) m_motion->MoveTo(pt->joints, instr.speed / 100.0f);
                break;
            }
            case InstructionType::LIN: {
                auto* pt = m_program.FindPoint(instr.pointName);
                if (pt) m_motion->MoveTo(pt->joints, instr.speed / 100.0f);
                break;
            }
            case InstructionType::SetDO: {
                m_program.GetIO().outputs[instr.digitalOutput % 6] = (instr.doValue != 0);
                break;
            }
            default: break;
        }
    });

    // Load a sample program for demonstration
    m_program.NewProgram("luexpo.lua");
    m_program.AddWait(5000);
    m_program.AddSetDO(3, 0);
    m_program.AddDoFile("/fruser/WELDO");
    m_program.AddDoFileEnd();
    m_program.AddLIN("l1", 60, 0);
    m_program.AddLIN("l7", 100, -1);
    m_program.AddLIN("l8", 100, -1);
    m_program.AddDoFile("/fruser/WELDO");
    m_program.AddDoFileEnd();
    m_program.AddLIN("l9", 10, -1);
    m_program.AddDoFile("/fruser/WELDO");
    m_program.AddDoFileEnd();
    m_program.AddLIN("l1", 70, -1);
}

TeachPendantUI::~TeachPendantUI() {
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void TeachPendantUI::BeginFrame() {
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
    ImGui::DockSpaceOverViewport(
        0,
        ImGui::GetMainViewport(),
        ImGuiDockNodeFlags_PassthruCentralNode
    );
}

void TeachPendantUI::Render() {
    // ── Full-screen layout ──
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::Begin("##TeachPendant", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse);

    float totalW = viewport->Size.x;
    float totalH = viewport->Size.y;
    float topBarH = 36.0f;
    float bottomBarH = 50.0f;
    float sidebarW = 160.0f;
    float paletteW = 90.0f;
    float editorW = 280.0f;
    float rightPanelW = totalW - sidebarW - paletteW - editorW;
    float bodyH = totalH - topBarH - bottomBarH;

    // Zone 1 — Top Toolbar
    ImGui::SetCursorPos(ImVec2(0, 0));
    ImGui::BeginChild("##TopToolbar", ImVec2(totalW, topBarH), true);
    RenderTopToolbar();
    ImGui::EndChild();

    // Zone 2 — Left Sidebar
    ImGui::SetCursorPos(ImVec2(0, topBarH));
    ImGui::BeginChild("##LeftSidebar", ImVec2(sidebarW, bodyH), true);
    RenderLeftSidebar();
    ImGui::EndChild();

    // Zone 3 — Command Palette
    ImGui::SetCursorPos(ImVec2(sidebarW, topBarH));
    ImGui::BeginChild("##CommandPalette", ImVec2(paletteW, bodyH), true);
    RenderCommandPalette();
    ImGui::EndChild();

    // Zone 4 — Program Editor
    ImGui::SetCursorPos(ImVec2(sidebarW + paletteW, topBarH));
    ImGui::BeginChild("##ProgramEditor", ImVec2(editorW, bodyH), true);
    RenderProgramEditor();
    ImGui::EndChild();

    // Right panel: split vertically into top (tabs + joints + 3D) and bottom (point mgr + IO)
    float rpX = sidebarW + paletteW + editorW;
    float rpTopH = bodyH * 0.65f;
    float rpBotH = bodyH * 0.35f;

    // Zone 5+6+7 — Operation Tabs + Joints + 3D Viewport
    ImGui::SetCursorPos(ImVec2(rpX, topBarH));
    ImGui::BeginChild("##RightTop", ImVec2(rightPanelW, rpTopH), true);
    {
        // Split right-top: left half = tabs+joints, right half = 3D viewport
        float halfW = rightPanelW * 0.55f;
        float vpW = rightPanelW * 0.45f;

        ImGui::BeginChild("##TabsJoints", ImVec2(halfW, rpTopH - 10), false);
        RenderOperationTabs();
        ImGui::Separator();
        RenderJointPanel();
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##3DViewport", ImVec2(vpW - 15, rpTopH - 10), true);
        Render3DViewport();
        ImGui::EndChild();
    }
    ImGui::EndChild();

    // Zone 8+9+10 — TCP, Point Manager, Data Readouts
    ImGui::SetCursorPos(ImVec2(rpX, topBarH + rpTopH));
    ImGui::BeginChild("##RightBottom", ImVec2(rightPanelW, rpBotH), true);
    {
        float thirdW = rightPanelW / 3.0f;

        ImGui::BeginChild("##TCPData", ImVec2(thirdW, rpBotH - 10), false);
        RenderTCPDisplay();
        RenderToolData();
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##PointMgr", ImVec2(thirdW, rpBotH - 10), false);
        RenderPointManager();
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##DataReadouts", ImVec2(thirdW - 15, rpBotH - 10), false);
        RenderDataReadouts();
        ImGui::EndChild();
    }
    ImGui::EndChild();

    // Zone 11 — Bottom Bar
    ImGui::SetCursorPos(ImVec2(0, topBarH + bodyH));
    ImGui::BeginChild("##BottomBar", ImVec2(totalW, bottomBarH), true);
    RenderBottomBar();
    ImGui::EndChild();

    // Settings modal
    if (m_showSettings) RenderSettingsModal();

    ImGui::End(); // ##TeachPendant

    // Update program execution
    m_program.Update(1.0f / 60.0f);

    // Update error tracking
    if (m_motion && !m_simulatorMode) {
        auto sim = m_motion->GetCurrentJoints();
        auto ref = m_motion->GetReferenceJoints();
        float totalErr = 0.0f;
        for (int i = 0; i < 6; i++) {
            float s = (i < (int)sim.size()) ? sim[i] : 0.0f;
            float r = (i < (int)ref.size()) ? ref[i] : 0.0f;
            m_jointErrors[i] = std::abs(s - r) * (180.0f / (float)M_PI);
            totalErr += m_jointErrors[i];
        }
        m_totalErrorPct = totalErr / 6.0f;
    }
}

void TeachPendantUI::EndFrame() {
    ImGui::Render();
    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault(NULL, (void*)m_dx12->GetCommandList());
    }
}

// ═════════════════════════════════════════════════════════
// THEME — FR-HMI Dark/Green industrial palette
// ═════════════════════════════════════════════════════════
void TeachPendantUI::ApplyFRTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 2.0f;
    style.ScrollbarRounding = 2.0f;
    style.FramePadding = ImVec2(6, 3);
    style.ItemSpacing = ImVec2(6, 4);
    style.WindowPadding = ImVec2(6, 6);
    style.ChildRounding = 0.0f;
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;

    ImVec4* c = style.Colors;
    // Main background
    c[ImGuiCol_WindowBg]          = ImVec4(0.13f, 0.14f, 0.15f, 1.0f);
    c[ImGuiCol_ChildBg]           = ImVec4(0.15f, 0.16f, 0.17f, 1.0f);
    c[ImGuiCol_PopupBg]           = ImVec4(0.12f, 0.12f, 0.14f, 0.98f);

    // Headers
    c[ImGuiCol_Header]            = ImVec4(0.22f, 0.23f, 0.25f, 1.0f);
    c[ImGuiCol_HeaderHovered]     = ImVec4(0.28f, 0.30f, 0.32f, 1.0f);
    c[ImGuiCol_HeaderActive]      = ImVec4(0.18f, 0.50f, 0.25f, 1.0f);

    // Buttons
    c[ImGuiCol_Button]            = ImVec4(0.20f, 0.22f, 0.24f, 1.0f);
    c[ImGuiCol_ButtonHovered]     = ImVec4(0.28f, 0.38f, 0.30f, 1.0f);
    c[ImGuiCol_ButtonActive]      = ImVec4(0.15f, 0.55f, 0.25f, 1.0f);

    // Frames (input fields, sliders)
    c[ImGuiCol_FrameBg]           = ImVec4(0.10f, 0.11f, 0.12f, 1.0f);
    c[ImGuiCol_FrameBgHovered]    = ImVec4(0.18f, 0.20f, 0.22f, 1.0f);
    c[ImGuiCol_FrameBgActive]     = ImVec4(0.15f, 0.40f, 0.22f, 1.0f);

    // Sliders
    c[ImGuiCol_SliderGrab]        = ImVec4(0.20f, 0.65f, 0.30f, 1.0f);
    c[ImGuiCol_SliderGrabActive]  = ImVec4(0.25f, 0.80f, 0.35f, 1.0f);

    // Tabs
    c[ImGuiCol_Tab]               = ImVec4(0.18f, 0.20f, 0.22f, 1.0f);
    c[ImGuiCol_TabHovered]        = ImVec4(0.25f, 0.50f, 0.32f, 1.0f);
    c[ImGuiCol_TabSelected]       = ImVec4(0.18f, 0.45f, 0.28f, 1.0f);

    // Title / Separator / Border
    c[ImGuiCol_TitleBg]           = ImVec4(0.08f, 0.09f, 0.10f, 1.0f);
    c[ImGuiCol_TitleBgActive]     = ImVec4(0.12f, 0.14f, 0.15f, 1.0f);
    c[ImGuiCol_Separator]         = ImVec4(0.30f, 0.32f, 0.34f, 0.5f);
    c[ImGuiCol_Border]            = ImVec4(0.25f, 0.27f, 0.30f, 0.5f);

    // Text
    c[ImGuiCol_Text]              = ImVec4(0.90f, 0.92f, 0.94f, 1.0f);
    c[ImGuiCol_TextDisabled]      = ImVec4(0.50f, 0.52f, 0.54f, 1.0f);

    // Scrollbar
    c[ImGuiCol_ScrollbarBg]       = ImVec4(0.10f, 0.10f, 0.12f, 1.0f);
    c[ImGuiCol_ScrollbarGrab]     = ImVec4(0.25f, 0.27f, 0.30f, 1.0f);
    c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.35f, 0.38f, 0.40f, 1.0f);

    // Check mark
    c[ImGuiCol_CheckMark]         = ImVec4(0.20f, 0.80f, 0.35f, 1.0f);
}

// ═════════════════════════════════════════════════════════
// ZONE 1 — TOP TOOLBAR
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderTopToolbar() {
    // FR Logo
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 0.6f, 0.3f, 1.0f));
    ImGui::Text("FR");
    ImGui::PopStyleColor();
    ImGui::SameLine();

    // Hamburger menu
    if (ImGui::Button("=")) { /* toggle menu */ }
    ImGui::SameLine(0, 15);

    // Transport controls
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.25f, 1.0f));
    ImVec2 tbtnSz(26, 22);

    // Lightning (auto-run)
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.0f, 1.0f));
    if (ImGui::Button("Z", tbtnSz)) { m_program.Start(); }
    ImGui::PopStyleColor();
    ImGui::SameLine(0, 2);

    // Play
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.2f, 0.8f, 0.3f, 1.0f));
    if (ImGui::Button(">", tbtnSz)) { m_program.Start(); }
    ImGui::PopStyleColor();
    ImGui::SameLine(0, 2);

    // Stop
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.2f, 0.2f, 1.0f));
    if (ImGui::Button("#", tbtnSz)) { m_program.Stop(); }
    ImGui::PopStyleColor();
    ImGui::SameLine(0, 2);

    // Record
    if (ImGui::Button("O", tbtnSz)) { /* record point */ }
    ImGui::SameLine(0, 2);

    // Pause
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.7f, 0.1f, 1.0f));
    if (ImGui::Button("||", tbtnSz)) { m_program.Pause(); }
    ImGui::PopStyleColor();
    ImGui::PopStyleColor(); // button bg

    ImGui::SameLine(0, 20);

    // File toolbar icons (save, open, etc.)
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.20f, 0.22f, 1.0f));
    ImVec2 fSz(22, 22);
    if (ImGui::Button("S", fSz)) { /* save */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("D", fSz)) { /* download */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("U", fSz)) { /* upload */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("P", fSz)) { /* print */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("C", fSz)) { /* copy */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("X", fSz)) { /* cut */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("V", fSz)) { /* paste */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("+", fSz)) { /* add */ } ImGui::SameLine(0, 1);
    if (ImGui::Button("-", fSz)) { /* remove */ }
    ImGui::PopStyleColor();

    ImGui::SameLine(0, 25);

    // Status badges
    bool isRunning = m_program.IsRunning();
    ImVec4 runColor = isRunning ? ImVec4(0.2f, 0.8f, 0.3f, 1.0f) : ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, runColor);
    ImGui::Button(isRunning ? "Running" : "Stopped");
    ImGui::PopStyleColor();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.8f, 1.0f, 1.0f));
    ImGui::Text("toolcoord1");
    ImGui::SameLine();
    ImGui::Text("wobj0");
    ImGui::SameLine();
    ImGui::Text("exaxis0");
    ImGui::PopStyleColor();

    ImGui::SameLine(0, 10);

    // Speed override
    ImGui::PushItemWidth(40);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.9f, 0.3f, 1.0f));
    ImGui::Text("%.0f", m_speedOverride);
    ImGui::PopStyleColor();
    ImGui::PopItemWidth();

    ImGui::SameLine(0, 10);

    // Gear / Warning / Language
    if (ImGui::Button("*")) { m_showSettings = true; }
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.7f, 0.0f, 1.0f));
    ImGui::Text("!");
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::Text("EN");

    ImGui::SameLine(ImGui::GetWindowWidth() - 40);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.1f, 0.1f, 1.0f));
    ImGui::Button("O", ImVec2(30, 22));
    ImGui::PopStyleColor();
}

// ═════════════════════════════════════════════════════════
// ZONE 2 — LEFT SIDEBAR
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderLeftSidebar() {
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.18f, 0.40f, 0.25f, 1.0f));

    // Initialize
    bool initOpen = ImGui::TreeNodeEx("Initialize", ImGuiTreeNodeFlags_SpanAvailWidth);
    if (initOpen) {
        ImGui::Text("  System Init");
        ImGui::Text("  Zero Point");
        ImGui::TreePop();
    }

    ImGui::Spacing();

    // Teaching (expanded by default)
    bool teachOpen = ImGui::TreeNodeEx("Teaching", ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_DefaultOpen);
    if (teachOpen) {
        const char* teachItems[] = { "Program Teaching", "Graphical Program", "Node Graph", "Manage Teaching" };
        for (int i = 0; i < 4; i++) {
            bool selected = (m_selectedTeachingTab == i);
            if (selected) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 0.9f, 0.4f, 1.0f));
            }
            if (ImGui::Selectable(teachItems[i], selected)) {
                m_selectedTeachingTab = i;
            }
            if (selected) ImGui::PopStyleColor();
        }
        ImGui::TreePop();
    }

    ImGui::Spacing();

    // Status
    if (ImGui::TreeNodeEx("Status", ImGuiTreeNodeFlags_SpanAvailWidth)) {
        ImGui::Text("  Robot Status");
        ImGui::Text("  Error Log");
        ImGui::TreePop();
    }

    ImGui::Spacing();

    // Auxiliary
    if (ImGui::TreeNodeEx("Auxiliary", ImGuiTreeNodeFlags_SpanAvailWidth)) {
        ImGui::Text("  Utilities");
        ImGui::Text("  Backup");
        ImGui::TreePop();
    }

    ImGui::Spacing();

    // Settings button
    if (ImGui::Button("X Settings", ImVec2(-1, 28))) {
        m_showSettings = true;
    }

    ImGui::PopStyleColor(); // Header

    // ── Zone 12 — Mode Toggle + Error % ──
    ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 120);
    ImGui::Separator();
    RenderModeToggle();
}

// ═════════════════════════════════════════════════════════
// ZONE 3 — COMMAND PALETTE (Logic + Motion)
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderCommandPalette() {
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 6));
    float btnW = 38.0f;
    float btnH = 32.0f;
    ImVec2 bsz(btnW, btnH);

    // Dropdown for filter
    ImGui::Text("All");
    ImGui::Separator();

    // ── Logic Commands ──
    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "Logic command");

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.25f, 0.35f, 1.0f));
    if (ImGui::Button("While", bsz)) {} ImGui::SameLine();
    if (ImGui::Button("If", bsz)) {}
    if (ImGui::Button("Goto", bsz)) {} ImGui::SameLine();
    if (ImGui::Button("Wait", bsz)) { m_program.AddWait(1000); }
    if (ImGui::Button("Pause", bsz)) {} ImGui::SameLine();
    if (ImGui::Button("DoFile", bsz)) {}
    if (ImGui::Button("Var", bsz)) {}
    ImGui::PopStyleColor();

    ImGui::Spacing();
    ImGui::Separator();

    // ── Motion Commands ──
    ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f), "Motion command");

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.35f, 0.22f, 1.0f));
    if (ImGui::Button("PTP", bsz)) { m_program.AddPTP("p1", m_speedOverride); }
    ImGui::SameLine();
    if (ImGui::Button("LIN", bsz)) { m_program.AddLIN("l1", m_speedOverride, 0); }
    if (ImGui::Button("ARC", bsz)) { m_program.AddARC("a1", "v1", m_speedOverride); }
    ImGui::SameLine();
    if (ImGui::Button("Circle", bsz)) { m_program.AddCircle("c1", m_speedOverride); }
    if (ImGui::Button("Spiral", bsz)) { m_program.AddSpiral("s1", m_speedOverride); }
    ImGui::SameLine();
    if (ImGui::Button("N-Spi", bsz)) {}
    if (ImGui::Button("H-Spi", bsz)) {}
    ImGui::SameLine();
    if (ImGui::Button("Spline", bsz)) {}
    if (ImGui::Button("N-Spl", bsz)) {}
    ImGui::SameLine();
    if (ImGui::Button("Weave", bsz)) {}
    ImGui::PopStyleColor();

    ImGui::PopStyleVar();
}

// ═════════════════════════════════════════════════════════
// ZONE 4 — PROGRAM EDITOR (Work Tree)
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderProgramEditor() {
    // File tab bar
    ImGui::PushStyleColor(ImGuiCol_Tab, ImVec4(0.20f, 0.22f, 0.25f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TabSelected, ImVec4(0.25f, 0.30f, 0.35f, 1.0f));
    if (ImGui::BeginTabBar("##ProgramTabs")) {
        if (ImGui::BeginTabItem(m_program.GetFilename().c_str())) {
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::PopStyleColor(2);

    ImGui::Separator();

    // Instruction list
    const auto& instrs = m_program.GetInstructions();
    int currentExec = m_program.GetCurrentLine();

    ImGui::BeginChild("##InstrList", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
    for (int i = 0; i < (int)instrs.size(); i++) {
        const auto& instr = instrs[i];
        
        ImGui::PushID(i);

        // Highlight current execution line (yellow)
        bool isCurrent = (m_program.IsRunning() && i == currentExec);
        if (isCurrent) {
            ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.55f, 0.55f, 0.0f, 0.7f));
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.60f, 0.60f, 0.0f, 0.8f));
        }

        // Breakpoint indicator
        if (instr.isBreakpoint) {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "*");
        } else {
            ImGui::Text(" ");
        }
        ImGui::SameLine();

        // Line number + arrows
        char lineLabel[64];
        snprintf(lineLabel, sizeof(lineLabel), "%d>> %s", instr.lineNum, instr.displayText.c_str());

        if (ImGui::Selectable(lineLabel, isCurrent, ImGuiSelectableFlags_AllowDoubleClick)) {
            // Double-click to toggle breakpoint
            if (ImGui::IsMouseDoubleClicked(0)) {
                m_program.GetInstructionsMutable()[i].isBreakpoint = !instr.isBreakpoint;
            }
        }

        if (isCurrent) {
            ImGui::PopStyleColor(2);
        }

        ImGui::PopID();
    }
    ImGui::EndChild();
}

// ═════════════════════════════════════════════════════════
// ZONE 5 — OPERATION & STATUS TABS
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderOperationTabs() {
    // Tab Bar 1: Joint | Base | Tool | Wobj | Move
    const char* opTabs[] = { "Joint", "Base", "Tool", "Wobj", "Move" };
    ImGui::PushStyleColor(ImGuiCol_Tab, ImVec4(0.25f, 0.28f, 0.30f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TabSelected, ImVec4(0.18f, 0.50f, 0.28f, 1.0f));
    if (ImGui::BeginTabBar("##OpTabs")) {
        for (int i = 0; i < 5; i++) {
            if (ImGui::BeginTabItem(opTabs[i])) {
                m_selectedOperationTab = i;
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
    ImGui::PopStyleColor(2);

    // Tab Bar 2: Exaxis | IO | TPD | FT | RCM
    const char* subTabs[] = { "Exaxis", "IO", "TPD", "FT", "RCM" };
    ImGui::PushStyleColor(ImGuiCol_Tab, ImVec4(0.22f, 0.24f, 0.28f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TabSelected, ImVec4(0.20f, 0.45f, 0.55f, 1.0f));
    if (ImGui::BeginTabBar("##SubTabs")) {
        for (int i = 0; i < 5; i++) {
            if (ImGui::BeginTabItem(subTabs[i])) {
                m_selectedSubTab = i;
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
    ImGui::PopStyleColor(2);

    ImGui::Spacing();

    // Speed / Acceleration / Threshold
    ImGui::PushItemWidth(60);
    ImGui::Text("Speed");       ImGui::SameLine(80); ImGui::InputFloat("##speed", &m_speedOverride, 0, 0, "%.0f"); ImGui::SameLine(); ImGui::Text("%%");
    ImGui::Text("Acceleration");ImGui::SameLine(80); ImGui::InputFloat("##accel", &m_accelerationPct, 0, 0, "%.0f"); ImGui::SameLine(); ImGui::Text("d/s2");
    ImGui::Text("Threshold");   ImGui::SameLine(80); ImGui::InputFloat("##thresh", &m_threshold, 0, 0, "%.0f");
    ImGui::PopItemWidth();

    ImGui::Spacing();

    // Single / Multi toggle
    ImGui::PushStyleColor(ImGuiCol_Button, m_jogMode == 0 ? ImVec4(0.15f, 0.50f, 0.28f, 1.0f) : ImVec4(0.20f, 0.22f, 0.25f, 1.0f));
    if (ImGui::Button("Single", ImVec2(55, 22))) m_jogMode = 0;
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, m_jogMode == 1 ? ImVec4(0.15f, 0.50f, 0.28f, 1.0f) : ImVec4(0.20f, 0.22f, 0.25f, 1.0f));
    if (ImGui::Button("Multi", ImVec2(55, 22))) m_jogMode = 1;
    ImGui::PopStyleColor();
}

// ═════════════════════════════════════════════════════════
// ZONE 6 — JOINT CONTROL (J1–J6)
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderJointPanel() {
    if (!m_robot || m_robot->GetDOF() == 0) {
        ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "No robot loaded");
        return;
    }

    auto angles = m_robot->GetJointAngles();
    int dof = std::min(m_robot->GetDOF(), 6);

    for (int i = 0; i < dof; i++) {
        ImGui::PushID(i);

        float degVal = angles[i] * (180.0f / (float)M_PI);
        char label[8];
        snprintf(label, sizeof(label), "J%d", i + 1);

        // J-label
        ImGui::Text("%s", label);
        ImGui::SameLine();

        // Minus button (green circle style)
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.50f, 0.22f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.65f, 0.30f, 1.0f));
        char minusId[16]; snprintf(minusId, sizeof(minusId), "-%s", label);
        if (ImGui::Button(minusId, ImVec2(24, 22))) {
            angles[i] -= m_jogStep * ((float)M_PI / 180.0f);
            m_robot->SetJointAngles(angles);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();

        // Slider
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x - 95);
        char sliderId[16]; snprintf(sliderId, sizeof(sliderId), "##sl%d", i);
        if (ImGui::SliderFloat(sliderId, &degVal, -180.0f, 180.0f, "")) {
            angles[i] = degVal * ((float)M_PI / 180.0f);
            m_robot->SetJointAngles(angles);
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();

        // Plus button
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.50f, 0.22f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.65f, 0.30f, 1.0f));
        char plusId[16]; snprintf(plusId, sizeof(plusId), "+%s", label);
        if (ImGui::Button(plusId, ImVec2(24, 22))) {
            angles[i] += m_jogStep * ((float)M_PI / 180.0f);
            m_robot->SetJointAngles(angles);
        }
        ImGui::PopStyleColor(2);
        ImGui::SameLine();

        // Numeric readout
        ImGui::Text("%8.3f", degVal);

        ImGui::PopID();
    }
}

// ═════════════════════════════════════════════════════════
// ZONE 7 — 3D ROBOT VIEWPORT
// ═════════════════════════════════════════════════════════
void TeachPendantUI::Render3DViewport() {
    ImVec2 avail = ImGui::GetContentRegionAvail();

    // Draw a tinted background representing the 3D viewport
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    draw->AddRectFilled(p, ImVec2(p.x + avail.x, p.y + avail.y),
        IM_COL32(20, 30, 35, 255));

    // Robot info overlay
    ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "3D Viewport");
    ImGui::TextColored(ImVec4(0.4f, 0.6f, 0.4f, 1.0f), "[Live DX12 Robot Render]");

    if (m_robot && m_robot->GetDOF() > 0) {
        // Show joint compactly
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.7f, 0.9f, 0.7f, 0.8f), "Joints");
        auto ang = m_robot->GetJointAngles();
        for (int i = 0; i < std::min((int)ang.size(), 6); i++) {
            ImGui::Text("J%d: %.1f", i+1, ang[i] * 180.0f / (float)M_PI);
            if (i < 5) ImGui::SameLine(0, 8);
        }

        // Draw colored XYZ axis indicators
        ImGui::Spacing();
        float axisY = ImGui::GetCursorScreenPos().y;
        float axisX = ImGui::GetCursorScreenPos().x + avail.x * 0.5f;
        float axLen = 30.0f;
        // X = Red
        draw->AddLine(ImVec2(axisX, axisY), ImVec2(axisX + axLen, axisY), IM_COL32(255, 50, 50, 255), 2.0f);
        draw->AddText(ImVec2(axisX + axLen + 2, axisY - 6), IM_COL32(255, 80, 80, 255), "X");
        // Y = Green
        draw->AddLine(ImVec2(axisX, axisY), ImVec2(axisX, axisY - axLen), IM_COL32(50, 255, 50, 255), 2.0f);
        draw->AddText(ImVec2(axisX - 10, axisY - axLen - 14), IM_COL32(80, 255, 80, 255), "Y");
        // Z = Blue
        draw->AddLine(ImVec2(axisX, axisY), ImVec2(axisX - axLen * 0.7f, axisY + axLen * 0.7f), IM_COL32(80, 80, 255, 255), 2.0f);
        draw->AddText(ImVec2(axisX - axLen * 0.7f - 10, axisY + axLen * 0.7f), IM_COL32(100, 100, 255, 255), "Z");
    }
}

// ═════════════════════════════════════════════════════════
// ZONE 8 — TCP & TOOL DATA
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderTCPDisplay() {
    ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.0f, 1.0f), "TCP");
    ImGui::Separator();

    // Get end effector pose
    if (m_robot && m_robot->GetDOF() > 0) {
        glm::mat4 eePose = m_robot->GetEndEffectorPose();
        m_tcpPos = glm::vec3(eePose[3]);

        // Extract Euler angles (simplified)
        m_tcpRot.x = atan2(eePose[2][1], eePose[2][2]) * 180.0f / (float)M_PI;
        m_tcpRot.y = asin(-eePose[2][0]) * 180.0f / (float)M_PI;
        m_tcpRot.z = atan2(eePose[1][0], eePose[0][0]) * 180.0f / (float)M_PI;
    }

    ImGui::Text("X  %9.3f", m_tcpPos.x * 1000.0f); // mm
    ImGui::Text("Y  %9.3f", m_tcpPos.y * 1000.0f);
    ImGui::Text("Z  %9.3f", m_tcpPos.z * 1000.0f);
    ImGui::Text("RX %9.3f", m_tcpRot.x);
    ImGui::Text("RY %9.3f", m_tcpRot.y);
    ImGui::Text("RZ %9.3f", m_tcpRot.z);
}

void TeachPendantUI::RenderToolData() {
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.9f, 0.8f, 0.5f, 1.0f), "Tool");
    ImGui::Separator();

    ImGui::Text("Fx  0.000  Fy  0.000  Fz  0.000");
    ImGui::Text("Tx  0.000  Ty  0.000  Tz  0.000");
    ImGui::Text("Act_State  0");
}

// ═════════════════════════════════════════════════════════
// ZONE 9 — POINT MANAGER
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderPointManager() {
    ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.5f, 1.0f), "Teach Points");
    ImGui::Separator();

    ImGui::Text("Prefix");
    ImGui::SameLine(60);
    ImGui::PushItemWidth(-1);
    ImGui::InputText("##prefix", m_pointPrefix, sizeof(m_pointPrefix));
    ImGui::PopItemWidth();

    ImGui::Text("Point");
    ImGui::SameLine(60);
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x - 50);
    ImGui::InputText("##ptname", m_pointName, sizeof(m_pointName));
    ImGui::PopItemWidth();
    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.40f, 0.65f, 1.0f));
    if (ImGui::Button("Add", ImVec2(42, 22))) {
        if (m_robot && strlen(m_pointName) > 0) {
            TeachPoint tp;
            tp.prefix = m_pointPrefix;
            tp.name = std::string(m_pointPrefix) + std::string(m_pointName);
            tp.joints = m_robot->GetJointAngles();
            
            glm::mat4 ee = m_robot->GetEndEffectorPose();
            tp.tcpPosition = glm::vec3(ee[3]);
            
            m_program.AddTeachPoint(tp);
            m_pointName[0] = '\0';
        }
    }
    ImGui::PopStyleColor();

    ImGui::Spacing();

    // Sensor dropdown
    const char* sensors[] = { "None", "Laser", "Camera", "Force/Torque" };
    ImGui::Text("Sensor");
    ImGui::SameLine(60);
    ImGui::PushItemWidth(-1);
    ImGui::Combo("##sensor", &m_selectedSensor, sensors, 4);
    ImGui::PopItemWidth();

    ImGui::Spacing();
    ImGui::Separator();

    // List saved points
    const auto& points = m_program.GetTeachPoints();
    if (!points.empty()) {
        ImGui::TextColored(ImVec4(0.5f, 0.7f, 0.5f, 1.0f), "Saved: %d", (int)points.size());
        for (const auto& pt : points) {
            ImGui::BulletText("%s", pt.name.c_str());
        }
    }
}

// ═════════════════════════════════════════════════════════
// ZONE 10 — DATA READOUTS
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderDataReadouts() {
    // Line-Num
    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.5f, 1.0f), "Line-Num");
    ImGui::Separator();
    ImGui::Text("Num   %d,0,0,%d", m_program.GetCurrentLine(), (int)m_program.GetInstructions().size());

    ImGui::Spacing();

    // Speed readout
    ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "Speed");
    ImGui::Separator();
    float tcpSpeed = 0.0f; // Could compute from joint velocities
    ImGui::Text("%.3f mm/s", tcpSpeed);

    ImGui::Spacing();

    // Wobj
    ImGui::Text("Wobj  : 0.000mm");

    ImGui::Spacing();

    // Conveyor
    ImGui::TextColored(ImVec4(0.8f, 0.7f, 0.5f, 1.0f), "Conveyor");
    ImGui::Separator();
    ImGui::Text("speed : 0.000mm/s");

    ImGui::Spacing();

    // CtrlBox — Digital I/O Grid
    ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.5f, 1.0f), "CtrlBox");
    ImGui::Separator();

    const auto& io = m_program.GetIO();
    for (int i = 0; i < 6; i++) {
        ImGui::PushID(i + 100);
        ImVec4 color = io.outputs[i] ? ImVec4(0.2f, 0.9f, 0.3f, 1.0f) : ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::Text("D%02d", i);
        ImGui::PopStyleColor();
        if (i % 2 == 0) ImGui::SameLine(55);
        ImGui::PopID();
    }
}

// ═════════════════════════════════════════════════════════
// ZONE 11 — BOTTOM TOOLBAR
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderBottomBar() {
    // FR-HMI Logo (orange area)
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    draw->AddRectFilled(p, ImVec2(p.x + 170, p.y + 38), IM_COL32(230, 120, 20, 255), 4.0f);
    draw->AddText(ImVec2(p.x + 15, p.y + 10), IM_COL32(255, 255, 255, 255), "FR-HMI");

    ImGui::SetCursorPosX(190);

    // Start button (green)
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.55f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.70f, 0.25f, 1.0f));
    if (ImGui::Button("> Start", ImVec2(80, 36))) {
        m_program.Start();
    }
    ImGui::PopStyleColor(2);

    ImGui::SameLine(0, 10);

    // Stop button (red)
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.12f, 0.12f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.80f, 0.15f, 0.15f, 1.0f));
    if (ImGui::Button("# Stop", ImVec2(80, 36))) {
        m_program.Stop();
        if (m_motion) m_motion->Home();
    }
    ImGui::PopStyleColor(2);

    // Function keys (right side)
    float fkX = ImGui::GetWindowWidth() - 230;
    ImGui::SameLine(fkX);

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.27f, 0.30f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.38f, 0.42f, 1.0f));
    ImVec2 fksz(48, 36);
    if (ImGui::Button("F1", fksz)) {} ImGui::SameLine(0, 4);
    if (ImGui::Button("F2", fksz)) {} ImGui::SameLine(0, 4);
    if (ImGui::Button("F3", fksz)) {} ImGui::SameLine(0, 4);
    if (ImGui::Button("F4", fksz)) {}
    ImGui::PopStyleColor(2);
}

// ═════════════════════════════════════════════════════════
// ZONE 12 — MODE TOGGLE + ERROR %
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderModeToggle() {
    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "Mode");

    // Toggle button
    ImVec4 simColor = m_simulatorMode ? ImVec4(0.15f, 0.55f, 0.28f, 1.0f) : ImVec4(0.25f, 0.27f, 0.30f, 1.0f);
    ImVec4 realColor = !m_simulatorMode ? ImVec4(0.55f, 0.25f, 0.15f, 1.0f) : ImVec4(0.25f, 0.27f, 0.30f, 1.0f);

    ImGui::PushStyleColor(ImGuiCol_Button, simColor);
    if (ImGui::Button("SIM", ImVec2(60, 22))) {
        m_simulatorMode = true;
        if (m_motion) m_motion->SetMode(MotionEngine::ControlMode::Simulation);
    }
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, realColor);
    if (ImGui::Button("REAL", ImVec2(60, 22))) {
        m_simulatorMode = false;
        if (m_motion) m_motion->SetMode(MotionEngine::ControlMode::RealHardware);
    }
    ImGui::PopStyleColor();

    // Error % display (only when in Real mode)
    if (!m_simulatorMode) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "Error %%");
        ImGui::Separator();
        for (int i = 0; i < 6; i++) {
            ImVec4 errColor = (m_jointErrors[i] > 5.0f) ? ImVec4(1, 0.2f, 0.2f, 1) : ImVec4(0.3f, 0.9f, 0.3f, 1);
            ImGui::TextColored(errColor, "J%d: %.2f", i + 1, m_jointErrors[i]);
        }
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Avg: %.2f%%", m_totalErrorPct);
    }
}

// ═════════════════════════════════════════════════════════
// SETTINGS MODAL
// ═════════════════════════════════════════════════════════
void TeachPendantUI::RenderSettingsModal() {
    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Settings", &m_showSettings);

    ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.6f, 1.0f), "Robot Configuration");
    ImGui::Separator();

    // Robot selection
    static int robotType = 0;
    const char* robots[] = { "FR-6 (6-Axis)", "FR-4 (4-Axis)", "Custom URDF" };
    ImGui::Combo("Robot Model", &robotType, robots, 3);

    ImGui::Spacing();

    // Connection
    ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.6f, 1.0f), "Communication");
    ImGui::Separator();

    if (m_motion) {
        auto& calib = m_motion->GetCalibration();
        char connStr[128];
        strncpy(connStr, calib.connectionString.c_str(), sizeof(connStr));
        connStr[sizeof(connStr) - 1] = '\0';
        if (ImGui::InputText("Port", connStr, sizeof(connStr))) {
            calib.connectionString = connStr;
        }
    }

    static char rosUri[128] = "ws://localhost:9090";
    ImGui::InputText("ROS URI", rosUri, sizeof(rosUri));

    ImGui::Spacing();

    // Jog settings
    ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.6f, 1.0f), "Jog Settings");
    ImGui::Separator();
    ImGui::SliderFloat("Step (deg)", &m_jogStep, 0.1f, 10.0f, "%.1f");
    ImGui::SliderFloat("Speed Override", &m_speedOverride, 1.0f, 100.0f, "%.0f%%");
    ImGui::SliderFloat("Acceleration", &m_accelerationPct, 10.0f, 500.0f, "%.0f deg/s2");

    ImGui::Spacing();

    // Calibration
    ImGui::TextColored(ImVec4(0.6f, 0.9f, 0.6f, 1.0f), "Joint Calibration");
    ImGui::Separator();
    if (m_motion) {
        auto& calib = m_motion->GetCalibration();
        for (int i = 0; i < 6; i++) {
            char lbl[16]; snprintf(lbl, sizeof(lbl), "J%d Offset", i + 1);
            ImGui::SliderFloat(lbl, &calib.encoderOffsets[i], -1.0f, 1.0f, "%.4f rad");
        }
    }

    ImGui::End();
}
