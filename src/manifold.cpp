#include "solver.h"

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB) 
    : Force(solver, bodyA, bodyB), numContacts(0) 
{   
    // set all normal f limits
    for (int i = 0; i < 4; i++) {
        fmax[i * 3] = 0.0f;
        fmin[i * 3] = -INFINITY;
    }
}

bool Manifold::initialize() {

    // compute friction
    friction = sqrtf(bodyA->friction * bodyB->friction);

    // TODO store previous contact state

    // Compute new contacts
    numContacts = collide(bodyA, bodyB, contacts);
    if (numContacts == 0) return false;

    // Set stiffness to FLT_MAX for hard constraints (normal and friction)
    for (int i = 0; i < rows(); ++i) {
        stiffness[i] = INFINITY;
    }

    // TODO Merge old contact data with new contacts

    // initialize contact data
    for (int i = 0; i < numContacts; i++) {
        Contact& contact = contacts[i];

        vec3 wA = transform(contact.rA, bodyA);
        vec3 wB = transform(contact.rB, bodyB);

        // compute tangent data
        vec3 linIndep = fabs(glm::dot(contact.normal, vec3(0, 1, 0))) > 0.95 ? vec3(1, 0, 0) : vec3(0, 1, 0);
        contact.t1 = linIndep - glm::dot(linIndep, contact.normal) * contact.normal;
        contact.t1 = glm::normalize(contact.t1);
        contact.t2 = glm::normalize(glm::cross(contact.t1, contact.normal)); // guarunteed to be normal

        // compute derivatives using taylor series to the first degree
        contact.JAn  = vec6(contact.normal, glm::cross(wA, contact.normal));
        contact.JAt1 = vec6(contact.t1    , glm::cross(wA, contact.t1)    );
        contact.JAt2 = vec6(contact.t2    , glm::cross(wA, contact.t2)    );

        contact.JBn  = vec6(-1.0f * contact.normal, -1.0f * glm::cross(wB, contact.normal));
        contact.JBt1 = vec6(-1.0f * contact.t1    , -1.0f * glm::cross(wB, contact.t1)    );
        contact.JBt2 = vec6(-1.0f * contact.t2    , -1.0f * glm::cross(wB, contact.t2)    );

        mat3x3 orthonormalBasis = glm::transpose(mat3x3(contact.t1, contact.t2, contact.normal));

        contact.C0 = orthonormalBasis * (bodyA->position + wA - bodyB->position - wB); // TODO add collision margin
    }

    return true;
}

void Manifold::computeConstraint(float alpha) {
    // compute positional changes
    // vec6 dA = bodyA->getConfiguration() - bodyA->initial;
    // vec6 dB = bodyB->getConfiguration() - bodyB->initial;

    for (int i = 0; i < numContacts; i++) {
        // --- Simple, Direct Constraint Calculation ---
        // Goal: C = 0 when objects are just touching, C < 0 when penetrating
        Contact& contact = contacts[i];

        vec6 dpA = { bodyA->position - bodyA->initialPosition, bodyA->deltaWInitial() };
        vec6 dpB = { bodyB->position - bodyB->initialPosition, bodyB->deltaWInitial() };

        // When C < 0, objects are too close (violating constraint)
        // When C >= 0, objects are properly separated (satisfying constraint)
        C[i * 3 + 0] = contact.C0[0] * (1 - alpha) + dot(contact.JAn,  dpA) + dot(contact.JBn,  dpB);
        C[i * 3 + 1] = contact.C0[1] * (1 - alpha) + dot(contact.JAt1, dpA) + dot(contact.JBt1, dpB);
        C[i * 3 + 2] = contact.C0[2] * (1 - alpha) + dot(contact.JAt2, dpA) + dot(contact.JBt2, dpB);

        // --- Update Force Limits for Friction Cone ---
        float frictionBound = abs(lambda[i * 3 + 0]) * friction;
        fmax[i * 3 + 1] = frictionBound;
        fmin[i * 3 + 1] = -frictionBound;
        fmax[i * 3 + 2] = frictionBound;
        fmin[i * 3 + 2] = -frictionBound;
        
        // --- Sticking Logic ---
        contact.stick = abs(lambda[i * 3 + 1]) < frictionBound && abs(contact.C0.z) < STICK_THRESH; // TODO check this convertsion to 3d
    }
}

void Manifold::computeDerivatives(Rigid* body) {
    // Just store precomputed derivatives in J for the desired body
    for (int i = 0; i < numContacts; i++)
    {
        Contact& contact = contacts[i];
        
        bool isA = body == bodyA;

        J[i * 3 + 0] = isA ? contact.JAn  : contact.JBn;
        J[i * 3 + 1] = isA ? contact.JAt1 : contact.JBt1;
        J[i * 3 + 2] = isA ? contact.JAt2 : contact.JBt2;
    }
}

void Manifold::draw() const {}