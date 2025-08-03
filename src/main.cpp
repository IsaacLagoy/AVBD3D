/*
 * main.cpp – minimal example rendering several 3D cuboids using AVBD Rigid structs
 */

#include "engine.h"
#include "solver.h"
#include <vector>
#include <cstdlib>
#include <chrono>

int main() {
    // 1. Create a few Rigid cuboids
    Solver solver;
    solver.gravity = -9.8f;
    solver.dt = 1.0f / 60.0f;
    solver.iterations = 10;

    // // Create ground plane (large flat box)
    new Rigid(&solver, {5, 0.25f, 5}, 1.0f, 0.5f, {0, -0.25f, 0});

    for (int i = 0; i < 2; ++i) {
        new Rigid(&solver, vec3(0.5f), 1.0f, 0.4f, vec3(uniform(-0.2f, 0.2f), uniform(1.5f, 2.0f), uniform(-0.2f, 0.2f)), vec6(), vec4(0, 0, 1, 1));
    }

    // 2. Create rendering engine and pass bodies
    Engine engine(800, 600, "AVBD Cuboids", "shaders/vertex.glsl", "shaders/fragment.glsl", solver.bodies);

    // track time
    std::__1::chrono::steady_clock::time_point lastFrameTime = std::chrono::steady_clock::now();

    // 3. Main loop
    while (!engine.shouldClose()) {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt = currentTime - lastFrameTime;

        solver.step(dt.count());
        engine.render();
        engine.update();

        lastFrameTime = currentTime;
    }
}
