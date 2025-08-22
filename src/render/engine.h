#ifndef ENGINE_H
#define ENGINE_H

#include "shader.h"
#include "solver.h" // Include your PhysicsEngine and RigidBody definitions
#include "mesh.h"
#include "rigid.h"
#include <cstdio>

class Camera {
    float moveSpeed = 5.0f;
    float mouseSensitivity = 0.002f;

    void updateRotation();

    public:
    vec3 position;
    quat rotation;

    Camera(const vec3& startPos = vec3(0, 0, 5));

    void processMouseMovement(float dx, float dy);
    void processKeyboardInput(GLFWwindow* window, float dt);

    vec3 getForward() const;
    vec3 getRight() const;
    vec3 getHorizontal() const;
    vec3 getUp() const;
    mat4x4 getViewMatrix() const;
    mat4x4 getProjectionMatrix(float aspectRatio) const;
};

class Engine {
    GLFWwindow* window;
    Shader* shader;
    Rigid*& bodies;
    Force*& forces;
    Camera camera;
    unsigned int VAO, VBOPositions, VBONormals, EBO;

    float lastX = 0.0f, lastY = 0.0f;
    bool firstMouse = true;
    float lastFrame = 0.0f;

    void setupCallbacks();
    bool initOpenGL();

    public:
    Engine(int width,
        int height,
        const char* title,
        const char* vertexShaderPath,
        const char* fragmentShaderPath,
        Rigid*& bodies,
        Force*& forces
    );
    ~Engine();

    
    void render();
    void update();
    bool shouldClose();

    Camera& getCamera() { return camera; }

    void processMouseMovement(float xpos, float ypos);
};

#endif