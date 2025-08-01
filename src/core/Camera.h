#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>

class Camera {
public:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;

    float yaw = -90.0f;   // Yaw starts pointing to -Z
    float pitch = 0.0f;   // Level pitch

    float speed;
    float sensitivity;

    Camera();

    glm::mat4 GetViewMatrix() const;
    void HandleInput(GLFWwindow* window, float deltaTime);
    void ProcessMouseMovement(float xoffset, float yoffset);
    void UpdateVectors();

};
