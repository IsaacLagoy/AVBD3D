#include "solver.h"
#include <iostream>

Solver::Solver() : bodies(nullptr), forces(nullptr) {
    defaultParams();
}

Solver::~Solver() {
    clear();
}

void Solver::clear() {
    // delete[] forces;
    delete[] bodies;
}  

void Solver::defaultParams()
{
    dt = 1.0f / 60.0f;
    gravity = -10;
    iterations = 10;

    // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
    // depend on the length, mass, and constraint function scales (ie units) of your simulation,
    // along with your strategy for incrementing the penalty parameters.
    // If the value is not in the right range, you may see slower convergance for complex scenes.
    beta = 100000.0f;

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic. Tune this depending
    // on your desired constraint error response.
    alpha = 0.99f;

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    // This should always be < 1 so that the penalty values can decrease (unless you use a different
    // penalty parameter strategy which does not require decay).
    gamma = 0.99f;
}

void Solver::step(float dt) {
    // broadphase collision, simple spherical distance checks
    for (Rigid* bodyA = bodies; bodyA; bodyA = bodyA->next) 
        for (Rigid* bodyB = bodyA->next; bodyB; bodyB = bodyB->next) {
            vec3 dp = bodyA->position - bodyB->position;
            float r = bodyA->radius + bodyB->radius;
            if (glm::dot(dp, dp) <= r * r && !bodyA->constrainedTo(bodyB))
                new Manifold(this, bodyA, bodyB); // handles narrowphase collision internally
        }

        // initialize and warmstart forces
        for (Force* force = forces; force != nullptr;) {
            // initialization can include caching anything that is constant over the step
            if (!force->initialize()) {
                // force has returned false meaning it is inactive, so remove it from the solver
                Force* next = force->next;
                delete force;
                force = next;
            } else {
                for (int i = 0; i < force->rows(); i++) {
                    // warmstart the dual variables and penalty parameters (Eq. 19)
                    // penalty is safely clamped to a minimum and maximum value
                    force->lambda[i] = force->lambda[i] * alpha * gamma;
                    force->penalty[i] = glm::clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);

                    // if it's not a hard constraint, we don't let the penalty exceed material stiffness
                    force->penalty[i] = glm::min(force->penalty[i], force->stiffness[i]);
                }

                force = force->next;
            }
        }

    // initialize and warmstart bodies (i.e. primal variables)
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        // don't let bodies rotate too fast !!! Fishy 2d code
        // body->velocity.z = clamp(body->velocity.z, -10.0f, 10.0f);

        // compute inertial position (Eq 2)
        body->inertial = body->position + body->velocity * dt;
        if (body->mass > 0) 
            body->inertial += vec3(0, gravity, 0) * (dt * dt);

        // adaptive warmstart (See original VBD paper)
        vec3 accel = (body->velocity - body->prevVelocity) / dt;
        float accelExt = accel.y * glm::sign(gravity);
        float accelWeight = glm::clamp(accelExt / abs(gravity), 0.0f, 1.0f);
        if (!isfinite(accelWeight)) accelWeight = 0.0f;

        // Save initial position (x-) and compute warmstarted position (See original VBD paper)
        body->initial = body->position;
        body->position = body->position + body->velocity * dt + vec3(0, gravity, 0) * (accelWeight * dt * dt);
    }

    // compute velocities (BDF1)
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        body->prevVelocity = body->velocity;
        if (body->mass > 0)
            body->velocity = (body->position - body->initial) / dt;
    }
}