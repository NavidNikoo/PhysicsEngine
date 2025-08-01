#include "Camera.h"

Camera::Camera()
    : position(0.0f, 0.0f, 20.0f),
      yaw(-90.0f),
      pitch(0.0f),
      speed(10.0f),
      sensitivity(0.1f)
{
    UpdateVectors();
}

glm::mat4 Camera::GetViewMatrix() const {
    return glm::lookAt(position, position + front, up);
}

void Camera::HandleInput(GLFWwindow* window, float deltaTime) {
    float velocity = speed * deltaTime;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        position += front * velocity;

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        position -= front * velocity;

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        position -= right * velocity;

    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        position += right * velocity;

    // Move up and down in WORLD space, not camera space
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        position.y += velocity;

    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        position.y -= velocity;
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset) {
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    // Clamp pitch to avoid gimbal lock
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    UpdateVectors();
}

void Camera::UpdateVectors() {
    glm::vec3 newFront;
    newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    newFront.y = sin(glm::radians(pitch));
    newFront.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));

    front = glm::normalize(newFront);

    // World up is (0,1,0) to keep "up" fixed for Q/E movement
    right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
    up    = glm::normalize(glm::cross(right, front));
}
