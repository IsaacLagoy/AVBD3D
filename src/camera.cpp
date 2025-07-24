#include "engine.h"

Camera::Camera(const vec3& startPos) : position(startPos), rotation(quat()) {}

void Camera::processMouseMovement(float dx, float dy) {
    dx *= mouseSensitivity;
    dy *= mouseSensitivity;

    // Pitch (x-axis) and yaw (y-axis) as quaternions
    quat qPitch = angleAxis(-dy, getRight());     // Look up/down
    quat qYaw   = angleAxis(-dx, vec3(0, 1, 0)); // Look left/right around world up

    rotation = normalize(qYaw * qPitch * rotation);
}

void Camera::processKeyboardInput(GLFWwindow* window, float dt) {
    float velocity = moveSpeed * dt;

    glm::vec3 horizontal = glm::normalize(getHorizontal()); // normalize horizontal forward vector
    glm::vec3 right = glm::normalize(getRight());           // normalize right vector
    glm::vec3 up(0.0f, 1.0f, 0.0f);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) position += horizontal * velocity;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) position -= horizontal * velocity;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) position -= right * velocity;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) position += right * velocity;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) position += up * velocity;
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) position -= up * velocity;
}

glm::vec3 Camera::getForward() const {
    return glm::normalize(rotation * glm::vec3(0, 0, -1));
}

glm::vec3 Camera::getHorizontal() const {
    glm::vec3 forward = getForward();
    forward.y = 0.0f;                // flatten y component to 0 for horizontal movement
    return glm::normalize(forward); // normalize the flattened vector
}

glm::vec3 Camera::getRight() const {
    return glm::normalize(rotation * glm::vec3(1, 0, 0));
}

glm::vec3 Camera::getUp() const {
    return glm::normalize(rotation * glm::vec3(0, 1, 0));
}


mat4x4 Camera::getViewMatrix() const {
    return glm::lookAt(position, position + getForward(), vec3(0, 1, 0));
}

mat4x4 Camera::getProjectionMatrix(float aspectRatio) const {
    float fov = glm::radians(45.0f);  // 45 degree field of view, you can make this configurable
    float nearPlane = 0.1f;
    float farPlane = 100.0f;
    return glm::perspective(fov, aspectRatio, nearPlane, farPlane);
}