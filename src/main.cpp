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
    solver.gravity = vec3(0, -9.8f, 0);
    solver.iterations = 10;

    vec3 offset = vec3(uniform(-2, 2), uniform(-2, 2),uniform(-2, 2));
    float diff = 0.2f;

    // Create ground plane (large flat box)
    new Rigid(&solver, {5, 0.25f, 5}, -1.0f, 0.5f, {0, -1.0f, 0});

    for (int i = 0; i < 1; ++i) {
        new Rigid(&solver, vec3(0.5f), 1.0f, 0.4f, vec3(uniform(-diff, diff), uniform(1.0f, diff), uniform(-diff, diff)), vec6());
    }

    // 2. Create rendering engine and pass bodies
    Engine engine(800, 600, "AVBD Cuboids", "shaders/vertex.glsl", "shaders/fragment.glsl", solver.bodies);

    // track time
    std::__1::chrono::steady_clock::time_point lastFrameTime = std::chrono::steady_clock::now();

    // 3. Main loop
    while (!engine.shouldClose()) {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> dt = currentTime - lastFrameTime;

        solver.step(dt.count() / 10);
        engine.render();
        engine.update();

        lastFrameTime = currentTime;
    }
}
