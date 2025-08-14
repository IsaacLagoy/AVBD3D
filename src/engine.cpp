/*
 * engine.cpp – 3‑D renderer that draws a vector of `Rigid*` cuboids.
 * Updated to remove all `scaleof`/`glfwGetFramebufferscale` typos and
 * ensure GLM/GLAD/GLFW includes are present.  Also clarified comments and
 * added error‑checking where appropriate.
 */

#include "engine.h"
#include <cstdio>

// mouse callback commands
void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    Engine* engine = static_cast<Engine*>(glfwGetWindowUserPointer(window));

    // detect clicking into the window
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    if (engine) engine->processMouseMovement(static_cast<float>(xpos), static_cast<float>(ypos));
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        // Lock & hide the cursor when left mouse button is clicked
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // Optionally center cursor
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glfwSetCursorPos(window, width / 2.0, height / 2.0);
    }
}

Engine::Engine(int width,
               int height,
               const char* title,
               const char* vertexPath,
               const char* fragmentPath,
               Rigid*& bodies,
               Force*& forces
)
    : window(nullptr), shader(nullptr), bodies(bodies), forces(forces)
{
    if(!initOpenGL()) {
        std::cerr << "Failed to initialize OpenGL and GLFW" << std::endl;
        return;
    }
    shader = new Shader(vertexPath, fragmentPath);

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBOPositions);
    glGenBuffers(1, &VBONormals);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    // Positions (binding to GL_ARRAY_BUFFER)
    glBindBuffer(GL_ARRAY_BUFFER, VBOPositions);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Mesh::verts), Mesh::verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    // Normals (same binding point but new buffer)
    glBindBuffer(GL_ARRAY_BUFFER, VBONormals);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Mesh::norms), Mesh::norms, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);

    // Indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Mesh::inds), Mesh::inds, GL_STATIC_DRAW);

    glBindVertexArray(0);
}

Engine::~Engine()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBOPositions);
    glDeleteBuffers(1, &VBONormals);
    glDeleteBuffers(1, &EBO);

    delete shader; 
    shader=nullptr;

    glfwTerminate();
}

// Renders all rigid bodies from the associated PhysicsEngine
void Engine::render() {
    // Clear the color and depth buffers
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f); // Set background color
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use the shader program
    shader->use();

    // Create view and projection matrices (common for all objects)
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 projection = camera.getProjectionMatrix((float) width / height);

    // Pass view and projection matrices to the shader
    shader->setMat4("view", view);
    shader->setMat4("projection", projection);

    shader->setVec3("lightPos", glm::vec3(10.0f, 10.0f, 10.0f));
    shader->setVec3("viewPos", camera.position); // Your camera position
    shader->setVec3("lightColor", glm::vec3(1.0f)); // white light

    // Bind the VAO
    glBindVertexArray(VAO);
    glm::mat4 model;

    // render wire frame for visibility
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // Iterate through all rigid bodies in the physics engine and render them
    for (Rigid* rigid = bodies; rigid != 0; rigid = rigid->next) {
        // Calculate the model matrix for the current rigid body
        // This includes translation (position), rotation, and scaling
        model = buildModelMatrix(rigid);

        // Pass the model matrix to the shader
        shader->setMat4("model", model);
        shader->setVec3("objectColor", rigid->color);

        // Draw the cube
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    }


    // render full shapes
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    // render all forces
    for (Force* force = forces; force != 0; force = force->next) {
        Manifold* man = (Manifold*) force;

        vec3 rA = transform(man->contacts[0].rA, man->bodyA);
        vec3 rB = transform(man->contacts[0].rB, man->bodyB);

        // graph rA
        model = buildModelMatrix(rA, vec3(0.1f), quat(1, 0, 0, 0));
        shader->setMat4("model", model);
        shader->setVec3("objectColor", vec4(1, 0, 0, 1));
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        // graph rB
        model = buildModelMatrix(rB, vec3(0.1f), quat(1, 0, 0, 0));
        shader->setMat4("model", model);
        shader->setVec3("objectColor", vec4(0, 0, 1, 1));
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        // project all minkowski points
        for (int i = 0; i < 3; i++) {
            vec3 rA = transform(man->contacts[0].CA[i], man->bodyA);
            vec3 rB = transform(man->contacts[0].CB[i], man->bodyB);

            model = buildModelMatrix(rA, vec3(0.05f), quat(1, 0, 0, 0));
            shader->setMat4("model", model);
            shader->setVec3("objectColor", vec4(1, 0, 0, 1));
            glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

            model = buildModelMatrix(rB, vec3(0.05f), quat(1, 0, 0, 0));
            shader->setMat4("model", model);
            shader->setVec3("objectColor", vec4(0, 0, 1, 1));
            glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
        }

        // connect the two contact points
        vec3 dir = rB - rA;
        float dist = glm::length(dir);
        quat look = glm::quatLookAt(dist < 1e-9f ? vec3{ 0, 1, 0, } : glm::normalize(dir), vec3{0, 1, 0});
        vec3 center = (rA + rB) * 0.5f;

        model = buildModelMatrix(center, vec3(0.02, 0.02, glm::length(dir)), look);
        shader->setMat4("model", model);
        shader->setVec3("objectColor", vec4(1, 0, 1, 1));
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        // graph normal
        vec3 n = man->contacts[0].normal; // already world space
        vec3 up = glm::abs(n.y) > 0.99f ? vec3(1,0,0) : vec3(0,1,0);
        look = glm::quatLookAt(n, up);

        vec3 lineCenter = (man->bodyA->position + man->bodyB->position) / 2.0f;  // center of the line
        float length = 2.0f;                     // desired visual length
        model = buildModelMatrix(lineCenter, vec3(0.02f, 0.02f, length), look);
        shader->setMat4("model", model);
        shader->setVec3("objectColor", vec4(0,1,0,1));
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        // put normal tip
        vec3 tipCenter = (man->bodyA->position + man->bodyB->position) / 2.0f + n;
        model = buildModelMatrix(tipCenter, vec3(0.1f), quat(1, 0, 0, 0));
        shader->setMat4("model", model);
        shader->setVec3("objectColor", vec4(0,1,0,1));
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    }
}

void Engine::update()
{
    // update camera
    float currentFrame = glfwGetTime();
    float deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    camera.processKeyboardInput(window, deltaTime);

    // calculate instantaneous FPS
    float fps = 1.0f / deltaTime;
    fps = (int) (fps * 10) / 10;
    char title[100];
    snprintf(title, sizeof(title), "Physics Engine Renderer ~ FPS: %.1f", fps);
    glfwSetWindowTitle(window, title);

    // update window
    glfwSwapBuffers(window);
    glfwPollEvents();
}

bool Engine::shouldClose() { return glfwWindowShouldClose(window); }

// Initializes OpenGL and GLFW
bool Engine::initOpenGL() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // Configure GLFW window hints
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a GLFW window
    window = glfwCreateWindow(800, 600, "Physics Engine Renderer", NULL, NULL);

    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetWindowUserPointer(window, this);  // inside Engine constructor
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);

    glfwSwapInterval(0);

    // Load OpenGL function pointers using GLAD
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return false;
    }

    // Set the viewport dimensions
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    // Enable depth testing for 3D rendering
    glEnable(GL_DEPTH_TEST);

    return true;
}

void Engine::setupCallbacks() {
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
}

void Engine::processMouseMovement(float xpos, float ypos) {
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
        return;
    }

    float dx = xpos - lastX;
    float dy = ypos - lastY;

    lastX = xpos;
    lastY = ypos;

    camera.processMouseMovement(dx, dy);
}
