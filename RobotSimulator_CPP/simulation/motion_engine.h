#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "../robot/robot_model.h"

enum class MotionProfile {
    Linear,
    Joint,
    Circular
};

class MotionEngine {
public:
    MotionEngine(RobotModel* robot);
    ~MotionEngine();

    void Update(float deltaTime);
    
    // Planner
    void MoveTo(const std::vector<float>& targetJoints, float speed);
    void MoveLinear(const glm::mat4& targetPose, float speed);
    
    void SetProfile(MotionProfile profile) { m_profile = profile; }

private:
    RobotModel* m_robot;
    MotionProfile m_profile;
    
    // Interpolator state
    std::vector<float> m_startJoints;
    std::vector<float> m_targetJoints;
    float m_currentTime;
    float m_duration;
    bool m_isMoving;
    
    // S-Curve logic...
};
