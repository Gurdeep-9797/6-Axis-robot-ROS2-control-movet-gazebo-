#pragma once

#include <vector>
#include <string>
#include <functional>
#include <glm/glm.hpp>
#include "../robot/robot_model.h"
#include "../engine/ros_client.h"

enum class MotionProfile {
    Linear,
    Joint,
    Circular
};

struct MotionPreset {
    std::string name;
    std::vector<std::vector<float>> waypoints; // sequence of joint configs
    float speed = 1.0f;
};

class MotionEngine {
public:
    MotionEngine(RobotModel* robot, RosClient* ros);
    ~MotionEngine();

    void Update(float deltaTime);
    
    // Planner
    void MoveTo(const std::vector<float>& targetJoints, float speed);
    void MoveLinear(const glm::mat4& targetPose, float speed);
    
    void SetProfile(MotionProfile profile) { m_profile = profile; }

    enum class ControlMode {
        Simulation,
        RealHardware
    };

    struct PipelineState {
        bool inputActive = false;
        bool plannerActive = false;
        bool outputActive = false;
        bool feedbackActive = false;
    };

    struct CalibrationData {
        std::vector<float> encoderOffsets = {0,0,0,0,0,0};
        std::vector<bool> invertMotor = {false,false,false,false,false,false};
        std::string connectionString = "COM3";
    };

    void SetMode(ControlMode mode);
    ControlMode GetMode() const { return m_currentMode; }
    
    // State Access
    std::vector<float> GetCurrentJoints() const;
    std::vector<float> GetReferenceJoints() const;
    std::vector<float> GetJointErrors() const;

    PipelineState GetPipelineState() const { return m_pipelineState; }
    CalibrationData& GetCalibration() { return m_calibration; }

    // Preset Movements
    void Home();
    void Ready();
    void WaveDemo();
    void PickPlaceDemo();
    void RunPreset(int index);
    const std::vector<MotionPreset>& GetPresets() const { return m_presets; }

    bool IsMoving() const { return m_isMoving; }
    std::string GetStatusText() const;

private:
    RobotModel* m_robot;
    RosClient* m_ros;
    MotionProfile m_profile;
    ControlMode m_currentMode = ControlMode::Simulation;
    PipelineState m_pipelineState;
    CalibrationData m_calibration;
    
    // Real Hardware State
    std::vector<float> m_realJoints = {0,0,0,0,0,0};
    
    // Interpolator state
    std::vector<float> m_startJoints;
    std::vector<float> m_targetJoints;
    float m_currentTime = 0.0f;
    float m_duration = 2.0f;
    bool m_isMoving = false;

    // Multi-waypoint sequence
    std::vector<std::vector<float>> m_waypointQueue;
    int m_currentWaypoint = 0;
    std::string m_statusText = "IDLE";

    // Built-in presets
    std::vector<MotionPreset> m_presets;
    void InitPresets();
};
