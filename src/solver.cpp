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

    if (dt < 1e-5f) dt = 1e-5f;

    if (DEBUG_PRINT) print("Starting Solver Step");

    // broadphase collision, simple spherical distance checks
    for (Rigid* bodyA = bodies; bodyA; bodyA = bodyA->next) 
        for (Rigid* bodyB = bodyA->next; bodyB; bodyB = bodyB->next) {
            vec3 dp = bodyA->position - bodyB->position;
            float r = bodyA->radius + bodyB->radius;
            if (glm::dot(dp, dp) <= r * r && !bodyA->constrainedTo(bodyB))
                new Manifold(this, bodyA, bodyB); // handles narrowphase collision internally
        }

    if (DEBUG_PRINT) print("Warmstart Forces");

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
                if (DEBUG_PRINT) print("Setting force lambda");
                force->lambda[i] = force->lambda[i] * alpha * gamma;
                if (DEBUG_PRINT) print("Setting force penalty");
                force->penalty[i] = glm::clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);

                // if it's not a hard constraint, we don't let the penalty exceed material stiffness
                if (DEBUG_PRINT) print("Something about hard constraints");
                force->penalty[i] = glm::min(force->penalty[i], force->stiffness[i]);
            }

            force = force->next;
        }
    }

    if (DEBUG_PRINT) print("Warmstart Bodies");

    // initialize and warmstart bodies (i.e. primal variables)
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        // don't let bodies rotate too fast !!! Fishy 2d code
        // body->velocity.z = clamp(body->velocity.z, -10.0f, 10.0f);

        // compute inertial position (Eq 2)
        body->inertial = body->getConfiguration() + body->velocity * dt;
        if (body->mass > 0) 
            body->inertial.linear += vec3(0, gravity, 0) * (dt * dt);

        // adaptive warmstart (See original VBD paper)
        vec6 accel = (body->velocity - body->prevVelocity) / dt;
        float accelExt = accel.linear.y * glm::sign(gravity);
        float accelWeight = glm::clamp(accelExt / abs(gravity), 0.0f, 1.0f);
        if (!isfinite(accelWeight)) accelWeight = 0.0f;

        // Save initial position (x-) and compute warmstarted position (See original VBD paper)
        body->initial = body->getConfiguration();
        body->setConfiguration(body->getConfiguration() + body->velocity * dt + vec6(0, gravity, 0, 0, 0, 0) * (accelWeight * dt * dt));
    }

    if (DEBUG_PRINT) print("Main solver loop");

    // main solver loop
    for (int i = 0; i < iterations; i++) {
        // primal update
        for (Rigid* body = bodies; body != nullptr; body = body->next) {
            // skip static bodies
            if (body->mass <= 0) continue;

            // initialize left and right hand sides of the linear system (Eqs. 5, 6)
            mat6x6 M = body->getMassMatrix();
            mat6x6 lhs = M / (dt * dt);
            vec6 rhs = lhs * (body->getConfiguration() - body->inertial);

            // iterate over all acting on the body
            for (Force* force = body->forces; force != nullptr; force = (force->bodyA == body) ? force->nextA : force->nextB) {
                // compute constraint and its derivatives
                force->computeConstraint(alpha);
                force->computeDerivatives(body);

                for (int i = 0; i < force->rows(); i++) {
                    // use lambda as 0 if it's not a hard constraint
                    float lambda = isinf(force->stiffness[i]) ? force->lambda[i] : 0.0f;

                    // compute the clamped force magnitude (sec 3.2)
                    float f = glm::clamp(force->penalty[i] * force->C[i] + lambda + force->motor[i], force->fmin[i], force->fmax[i]);

                    // compute the diagonally lumped geometric stiffness term (sec 3.5)
                    mat6x6 G;

                    // accumulate force (eq. 13) and hessian (eq. 17)
                    rhs += force->J[i] * f;
                    // TODO do lhs stuff
                }
            }
        }
    }

    if (DEBUG_PRINT) print("Compute Velocities");

    // compute velocities (BDF1)
    for (Rigid* body = bodies; body != nullptr; body = body->next) {
        body->prevVelocity = body->velocity;
        if (body->mass > 0) body->velocity = (body->getConfiguration() - body->initial) / dt;
    }
}