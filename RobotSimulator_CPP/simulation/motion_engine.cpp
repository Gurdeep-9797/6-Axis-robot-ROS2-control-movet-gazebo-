#include "motion_engine.h"
#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>

MotionEngine::MotionEngine(RobotModel* robot, RosClient* ros) 
    : m_robot(robot), m_ros(ros), m_isMoving(false),
      m_profile(MotionProfile::Joint),
      m_currentTime(0.0f), m_duration(2.0f),
      m_currentWaypoint(0)
{
    m_currentMode = ControlMode::Simulation;
    m_startJoints = {0,0,0,0,0,0};
    m_targetJoints = {0,0,0,0,0,0};
    InitPresets();
}

MotionEngine::~MotionEngine() {}

void MotionEngine::InitPresets() {
    // Home: all zeros
    {
        MotionPreset p;
        p.name = "Home";
        p.waypoints.push_back({0, 0, 0, 0, 0, 0});
        p.speed = 1.0f;
        m_presets.push_back(p);
    }

    // Ready: safe position
    {
        MotionPreset p;
        p.name = "Ready";
        p.waypoints.push_back({0, -0.785f, 0.785f, 0, 0, 0}); // J2=-45deg, J3=+45deg
        p.speed = 1.0f;
        m_presets.push_back(p);
    }

    // Wave Demo: oscillate wrist joints
    {
        MotionPreset p;
        p.name = "Wave Demo";
        p.waypoints.push_back({0, -0.5f, 0.8f, 0, 0, 0});       // reach up
        p.waypoints.push_back({0, -0.5f, 0.8f, 0, 0.8f, 0});     // wave right
        p.waypoints.push_back({0, -0.5f, 0.8f, 0, -0.8f, 0});    // wave left
        p.waypoints.push_back({0, -0.5f, 0.8f, 0, 0.8f, 0});     // wave right
        p.waypoints.push_back({0, -0.5f, 0.8f, 0, -0.8f, 0});    // wave left
        p.waypoints.push_back({0, -0.5f, 0.8f, 0, 0, 0});        // center
        p.speed = 2.0f;
        m_presets.push_back(p);
    }

    // Pick & Place Demo
    {
        MotionPreset p;
        p.name = "Pick & Place";
        p.waypoints.push_back({0, -0.3f, 0.5f, 0, 0, 0});         // hover above pick
        p.waypoints.push_back({0, -0.8f, 1.2f, 0, 0.5f, 0});      // reach down
        p.waypoints.push_back({0, -0.3f, 0.5f, 0, 0.5f, 0});      // lift
        p.waypoints.push_back({1.57f, -0.3f, 0.5f, 0, 0.5f, 0});  // rotate base 90deg
        p.waypoints.push_back({1.57f, -0.8f, 1.2f, 0, 0.5f, 0});  // lower to place
        p.waypoints.push_back({1.57f, -0.3f, 0.5f, 0, 0, 0});     // retract
        p.waypoints.push_back({0, 0, 0, 0, 0, 0});                 // home
        p.speed = 1.5f;
        m_presets.push_back(p);
    }
}

void MotionEngine::SetMode(ControlMode mode) {
    m_currentMode = mode;
}

std::string MotionEngine::GetStatusText() const {
    return m_statusText;
}

std::vector<float> MotionEngine::GetCurrentJoints() const {
    if (m_currentMode == ControlMode::RealHardware) {
        std::vector<float> calibrated = m_realJoints;
        for(size_t i=0; i<calibrated.size(); i++) {
            if(i < m_calibration.encoderOffsets.size()) {
                calibrated[i] += m_calibration.encoderOffsets[i];
            }
        }
        return calibrated;
    }

    if (m_robot && m_robot->GetDOF() > 0) {
        return m_robot->GetJointAngles();
    }

    return std::vector<float>(6, 0.0f);
}

std::vector<float> MotionEngine::GetReferenceJoints() const {
    if (m_ros && m_ros->IsConnected()) {
        return m_ros->GetReferenceJointState();
    }
    return m_targetJoints.empty() ? std::vector<float>(6, 0.0f) : m_targetJoints;
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

// ─────────────────────────────────────────────────────────
// Motion Commands
// ─────────────────────────────────────────────────────────

void MotionEngine::MoveTo(const std::vector<float>& targetJoints, float speed) {
    m_startJoints = GetCurrentJoints();
    m_targetJoints = targetJoints;
    m_isMoving = true;
    m_currentTime = 0.0f;
    
    // Calculate duration based on max joint travel
    float maxDelta = 0.0f;
    for (size_t i = 0; i < m_startJoints.size() && i < targetJoints.size(); i++) {
        maxDelta = std::max(maxDelta, std::abs(targetJoints[i] - m_startJoints[i]));
    }
    m_duration = std::max(0.5f, maxDelta / speed);
    m_statusText = "MOVING";
    
    spdlog::info("MoveTo: duration={:.2f}s, speed={:.2f}", m_duration, speed);
}

void MotionEngine::Update(float deltaTime) {
    if (!m_isMoving) return;
    
    m_currentTime += deltaTime;
    
    // S-curve easing: smooth acceleration/deceleration
    float t = std::min(m_currentTime / m_duration, 1.0f);
    float s = t * t * (3.0f - 2.0f * t); // smoothstep
    
    // Interpolate joints
    std::vector<float> current(m_startJoints.size());
    for (size_t i = 0; i < current.size(); i++) {
        float target = (i < m_targetJoints.size()) ? m_targetJoints[i] : 0.0f;
        current[i] = m_startJoints[i] + (target - m_startJoints[i]) * s;
    }
    
    // Apply to robot
    if (m_robot) {
        m_robot->SetJointAngles(current);
    }
    
    if (t >= 1.0f) {
        // Segment complete
        if (!m_waypointQueue.empty() && m_currentWaypoint < (int)m_waypointQueue.size() - 1) {
            // Next waypoint
            m_currentWaypoint++;
            MoveTo(m_waypointQueue[m_currentWaypoint], m_presets.empty() ? 1.0f : 1.5f);
        } else {
            // All done
            m_isMoving = false;
            m_waypointQueue.clear();
            m_currentWaypoint = 0;
            m_statusText = "IDLE";
            spdlog::info("Motion complete");
        }
    }
}

void MotionEngine::MoveLinear(const glm::mat4& targetPose, float speed) {
    // TODO: Cartesian linear motion via IK
    (void)targetPose;
    (void)speed;
}

// ─────────────────────────────────────────────────────────
// Presets
// ─────────────────────────────────────────────────────────

void MotionEngine::Home() {
    m_waypointQueue.clear();
    m_currentWaypoint = 0;
    MoveTo({0, 0, 0, 0, 0, 0}, 1.5f);
    m_statusText = "HOMING";
}

void MotionEngine::Ready() {
    m_waypointQueue.clear();
    m_currentWaypoint = 0;
    MoveTo({0, -0.785f, 0.785f, 0, 0, 0}, 1.0f);
    m_statusText = "READY POS";
}

void MotionEngine::WaveDemo() {
    m_waypointQueue = m_presets[2].waypoints;
    m_currentWaypoint = 0;
    MoveTo(m_waypointQueue[0], m_presets[2].speed);
    m_statusText = "WAVE DEMO";
}

void MotionEngine::PickPlaceDemo() {
    m_waypointQueue = m_presets[3].waypoints;
    m_currentWaypoint = 0;
    MoveTo(m_waypointQueue[0], m_presets[3].speed);
    m_statusText = "PICK & PLACE";
}

void MotionEngine::RunPreset(int index) {
    if (index < 0 || index >= (int)m_presets.size()) return;
    
    switch (index) {
        case 0: Home(); break;
        case 1: Ready(); break;
        case 2: WaveDemo(); break;
        case 3: PickPlaceDemo(); break;
        default: break;
    }
}
