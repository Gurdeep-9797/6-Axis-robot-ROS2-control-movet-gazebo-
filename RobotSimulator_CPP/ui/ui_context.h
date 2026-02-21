#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <imgui.h>
#include <string>
#include <vector>
#include <glm/glm.hpp>

// Forward declarations
class DX12Core;
class PipelineView;
class CalibrationPanel;
class MotionEngine;
class RobotModel;
class SceneRenderer;

struct Obstacle {
    std::string name;
    glm::vec3 position;
    glm::vec3 size;
    bool active;
};

struct PlannerTask {
    std::string name;
    std::vector<float> targetJoints;
    float time;
};

class UIContext {
public:
    UIContext(HWND hwnd, DX12Core* dx12, MotionEngine* motion, RobotModel* robot, SceneRenderer* scene);
    ~UIContext();

    void BeginFrame();
    void Render();
    void EndFrame();

    std::vector<Obstacle> m_obstacles;
    std::vector<PlannerTask> m_taskQueue;

private:
    DX12Core* m_dx12;
    MotionEngine* m_motion;
    RobotModel* m_robot;
    SceneRenderer* m_scene;
    PipelineView* m_pipelineView;
    CalibrationPanel* m_calibrationPanel;
    
    void ApplyCNCTheme();
    void RenderCNCPanel();
    void RenderJointControl();
    void RenderPresetPanel();
    void RenderDiagnosticsPanel();
    void RenderRobotInfo();
    void RenderPlannerPanel();
    void RenderMenu();
};
