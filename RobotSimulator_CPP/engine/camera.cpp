#include "camera.h"
#include <algorithm>

Camera::Camera(float aspect) 
    : m_radius(2.0f), m_theta(0.785f), m_phi(0.785f), 
      m_target(0.0f, 0.5f, 0.0f),
      m_aspect(aspect), m_fov(45.0f), m_near(0.01f), m_far(100.0f)
{
}

void Camera::SetAspect(float aspect) {
    m_aspect = aspect;
}

void Camera::Update(float /*deltaTime*/) {
    // Smooth interpolation could go here
}

glm::mat4 Camera::GetViewMatrix() const {
    // Spherical to Cartesian
    float x = m_radius * sinf(m_phi) * cosf(m_theta);
    float z = m_radius * sinf(m_phi) * sinf(m_theta);
    float y = m_radius * cosf(m_phi);

    glm::vec3 pos = m_target + glm::vec3(x, y, z);
    
    // Y is up
    return glm::lookAt(pos, m_target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 Camera::GetProjectionMatrix() const {
    return glm::perspective(glm::radians(m_fov), m_aspect, m_near, m_far);
}

void Camera::Orbit(float deltaX, float deltaY) {
    m_theta -= deltaX * 0.01f;
    m_phi   -= deltaY * 0.01f;

    // Clamp phi to prevent flipping
    m_phi = std::clamp(m_phi, 0.01f, 3.14159f - 0.01f);
}

void Camera::Pan(float deltaX, float deltaY) {
    glm::mat4 V = GetViewMatrix(); // get current view
    glm::vec3 right(V[0][0], V[1][0], V[2][0]);
    glm::vec3 up(V[0][1], V[1][1], V[2][1]);
    
    float panSpeed = m_radius * 0.001f;
    m_target += -right * (deltaX * panSpeed);
    m_target += up * (deltaY * panSpeed);
}

void Camera::Zoom(float delta) {
    m_radius -= delta * 0.05f;
    m_radius = std::clamp(m_radius, 0.1f, 10.0f);
}
