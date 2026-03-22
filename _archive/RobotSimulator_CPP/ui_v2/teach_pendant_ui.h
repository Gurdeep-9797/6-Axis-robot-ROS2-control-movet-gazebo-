#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <imgui.h>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "program_model.h"

// Forward declarations
class DX12Core;
class MotionEngine;
class RobotModel;
class SceneRenderer;

// ─────────────────────────────────────────────────────────
// Teach Pendant UI — FR-HMI style industrial controller
// ─────────────────────────────────────────────────────────
class TeachPendantUI {
public:
    TeachPendantUI(HWND hwnd, DX12Core* dx12, MotionEngine* motion, RobotModel* robot, SceneRenderer* scene);
    ~TeachPendantUI();

    void BeginFrame();
    void Render();
    void EndFrame();

    // Access for main loop
    ProgramModel& GetProgram() { return m_program; }

private:
    DX12Core* m_dx12;
    MotionEngine* m_motion;
    RobotModel* m_robot;
    SceneRenderer* m_scene;
    ProgramModel m_program;

    // ── Theme ──
    void ApplyFRTheme();

    // ── UI Zones ──
    void RenderTopToolbar();          // Zone 1
    void RenderLeftSidebar();         // Zone 2
    void RenderCommandPalette();      // Zone 3
    void RenderProgramEditor();       // Zone 4
    void RenderOperationTabs();       // Zone 5
    void RenderJointPanel();          // Zone 6
    void Render3DViewport();          // Zone 7
    void RenderTCPDisplay();          // Zone 8
    void RenderPointManager();        // Zone 9
    void RenderDataReadouts();        // Zone 10
    void RenderBottomBar();           // Zone 11
    void RenderModeToggle();          // Zone 12

    // ── Helper renderers ──
    void RenderSettingsModal();
    void RenderToolData();

    // ── State ──
    int m_selectedSidebarItem = 1;     // 0=Init, 1=Teaching, 2=Status, 3=Auxiliary, 4=Settings
    int m_selectedTeachingTab = 0;     // 0=Program Teaching, 1=Graphical, 2=Node Graph, 3=Manage
    int m_selectedOperationTab = 0;    // 0=Joint, 1=Base, 2=Tool, 3=Wobj, 4=Move
    int m_selectedSubTab = 0;          // 0=Exaxis, 1=IO, 2=TPD, 3=FT, 4=RCM
    int m_jogMode = 0;                 // 0=Single, 1=Multi

    float m_speedOverride = 100.0f;    // 0-100%
    float m_accelerationPct = 180.0f;  // deg/s^2
    float m_threshold = 30.0f;

    bool m_showSettings = false;
    bool m_simulatorMode = true;       // true = Sim, false = Real

    // Point manager state
    char m_pointPrefix[64] = "";
    char m_pointName[64] = "";
    int m_selectedSensor = 0;

    // Jog step state
    float m_jogStep = 1.0f;            // degrees per click

    // Error tracking (sim vs real)
    std::vector<float> m_jointErrors = {0,0,0,0,0,0};
    float m_totalErrorPct = 0.0f;

    // Coordinate readout cache
    glm::vec3 m_tcpPos = glm::vec3(0);
    glm::vec3 m_tcpRot = glm::vec3(0);
};
