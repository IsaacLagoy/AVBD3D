#include "solver.h"

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB) : Force(solver, bodyA, bodyB), numContacts(0) {
    fmax[0] = fmax[2] = 0.0f;
    fmin[0] = fmin[2] = -INFINITY;
}

bool Manifold::initialize() {
    // compute friction
    friction = sqrtf(bodyA->friction * bodyB->friction);

    // store previous contact state
    Contact oldContacts[2] = { contacts[0], contacts[1] };
    float oldPenalty[4] = { penalty[0], penalty[1], penalty[2], penalty[3] };
    float oldLambda[4] = { lambda[0], lambda[1], lambda[2], lambda[3] };
    bool oldStick[2] = { contacts[0].stick, contacts[1].stick };
    int oldNumContacts = numContacts;

    // Compute new contacts
    numContacts = collide(bodyA, bodyB, contacts);

    // Merge old contact data with new contacts
    // for (int i = 0; i < numContacts; i++) {
    //     penalty[i * 2 + 0] = penalty[i * 2 + 1] = 0.0f;
    //     lambda[i * 2 + 0] = lambda[i * 2 + 0] = 0.0f;

    // }

    return numContacts > 0;
}

void Manifold::computeConstraint(float alpha) {

}

void Manifold::computeDerivatives(Rigid* body) {

}

void Manifold::draw() const {}