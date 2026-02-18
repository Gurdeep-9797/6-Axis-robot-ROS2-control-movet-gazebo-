#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "../robot/robot_model.h"
#include "../engine/ros_client.h"

enum class MotionProfile {
    Linear,
    Joint,
    Circular
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
        bool outputActive = false; // Sending to Sim/Real
        bool feedbackActive = false; // Receiving from Sim/Real
    };

    struct CalibrationData {
        std::vector<float> encoderOffsets = {0,0,0,0,0,0};
        std::vector<bool> invertMotor = {false,false,false,false,false,false};
        std::string connectionString = "COM3"; // or IP
    };

    void SetMode(ControlMode mode);
    ControlMode GetMode() const { return m_currentMode; }
    
    // Abstracted State Access
    std::vector<float> GetCurrentJoints() const;
    std::vector<float> GetReferenceJoints() const;
    std::vector<float> GetJointErrors() const;

    PipelineState GetPipelineState() const { return m_pipelineState; }
    CalibrationData& GetCalibration() { return m_calibration; }

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
    float m_currentTime;
    float m_duration;
    bool m_isMoving;
    
    // S-Curve logic...
};
