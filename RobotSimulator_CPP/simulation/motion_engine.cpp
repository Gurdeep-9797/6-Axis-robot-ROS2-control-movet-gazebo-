#include "motion_engine.h"
#include <algorithm>

MotionEngine::MotionEngine(RobotModel* robot, RosClient* ros) 
    : m_robot(robot), m_ros(ros), m_isMoving(false) 
{
    m_currentMode = ControlMode::Simulation;
}
MotionEngine::~MotionEngine() {}

void MotionEngine::SetMode(ControlMode mode) {
    m_currentMode = mode;
    // Reset state or reconnect hardware here
}

std::vector<float> MotionEngine::GetCurrentJoints() const {
    if (m_currentMode == ControlMode::RealHardware) {
        // Return encoder values modified by calibration
        std::vector<float> calibrated = m_realJoints;
        for(size_t i=0; i<calibrated.size(); i++) {
            if(i < m_calibration.encoderOffsets.size()) {
                calibrated[i] += m_calibration.encoderOffsets[i];
            }
        }
        return calibrated;
    }
    // In Sim mode, return the interpolated target or read from ROS
    if (m_currentMode == ControlMode::Simulation && m_ros && m_ros->IsConnected()) {
        return m_ros->GetLatestJointState();
    }

    if (m_isMoving) {
        // Linearly interpolated value (Simplified for now)
        std::vector<float> result = m_startJoints;
        float t = std::min(m_currentTime / m_duration, 1.0f);
        for(size_t i=0; i<result.size(); ++i) {
             result[i] += (m_targetJoints[i] - m_startJoints[i]) * t;
        }
        return result;
    }
    return m_targetJoints.empty() ? std::vector<float>(6, 0.0f) : m_targetJoints;
}

std::vector<float> MotionEngine::GetReferenceJoints() const {
    if (m_ros && m_ros->IsConnected()) {
        return m_ros->GetReferenceJointState();
    }
    return std::vector<float>(6, 0.0f);
}

std::vector<float> MotionEngine::GetJointErrors() const {
    std::vector<float> actual = GetCurrentJoints();
    std::vector<float> ref = GetReferenceJoints();
    std::vector<float> errors;
    
    for(size_t i=0; i<actual.size() && i<ref.size(); ++i) {
        errors.push_back(std::abs(actual[i] - ref[i]));
    }
    return errors;
}

void MotionEngine::MoveTo(const std::vector<float>& targetJoints, float speed) {
    // Setup linear joint interpolation
    m_targetJoints = targetJoints;
    m_isMoving = true;
    m_currentTime = 0.0f;
    m_duration = 2.0f; // Calculate based on speed/accel limits
}

void MotionEngine::Update(float deltaTime) {
    if (!m_isMoving) return;
    
    m_currentTime += deltaTime;
    float t = std::min(m_currentTime / m_duration, 1.0f);
    
    // Basic Linear Interpolation (placeholder for S-Curve)
    // In real implementation:
    // current = start + (target - start) * SCurve(t)
    
    if (t >= 1.0f) {
        m_isMoving = false;
    }
}
