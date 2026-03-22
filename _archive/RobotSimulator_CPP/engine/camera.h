#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
    Camera(float aspect);
    ~Camera() = default;

    void SetAspect(float aspect);
    void Update(float deltaTime);

    glm::mat4 GetViewMatrix() const;
    glm::mat4 GetProjectionMatrix() const;

    // Orbit controls
    void Orbit(float deltaX, float deltaY);
    void Pan(float deltaX, float deltaY);
    void Zoom(float delta);

private:
    float m_radius;
    float m_theta;
    float m_phi;
    glm::vec3 m_target;
    
    float m_aspect;
    float m_fov;
    float m_near;
    float m_far;
};
