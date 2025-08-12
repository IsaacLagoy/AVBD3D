#include "solver.h"

Force::Force(Solver* solver, Rigid* bodyA, Rigid* bodyB) : solver(solver), bodyA(bodyA), bodyB(bodyB), nextA(nullptr), nextB(nullptr) {
    // add force to linked list
    next = solver->forces;
    solver->forces = this;

    if (bodyA) {
        nextA = bodyA->forces;
        bodyA->forces = this;
    }

    if (bodyB) {
        nextB = bodyB->forces;
        bodyB->forces = this;
    }

    // allocate memory for vectors
    J.resize(MAX_ROWS);
    H.resize(MAX_ROWS);

    // set reasonable defaults
    for (int i = 0; i < MAX_ROWS; i++) {
        J[i] = vec6(0); // 6 DOF
        H[i] = mat6x6(); // error here

        C[i] = 0.0f;
        motor[i] = 0.0f;

        stiffness[i] = INFINITY;
        fmax[i] = INFINITY;
        fmin[i] = -INFINITY;
        fracture[i] = INFINITY;

        penalty[i] = 0.0f;
        lambda[i] = 0.0f;
    }
}

Force::~Force() {
    // remove from all linked lists
    Force** p = &solver->forces;
    while (*p != this) p = &(*p)->next;
    *p = next;

    if (bodyA)
    {
        p = &bodyA->forces;
        while (*p != this)
            p = (*p)->bodyA == bodyA ? &(*p)->nextA : &(*p)->nextB;
        *p = nextA;
    }

    if (bodyB)
    {
        p = &bodyB->forces;
        while (*p != this)
            p = (*p)->bodyA == bodyB ? &(*p)->nextA : &(*p)->nextB;
        *p = nextB;
    }
}

void Force::disable() {
    for (int i = 0; i < MAX_ROWS; i++) {
        stiffness[i] = 0.0f;
        penalty[i] = 0.0f;
        lambda[i] = 0.0f;
    }
}