#include "solver.h"

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB) 
    : Force(solver, bodyA, bodyB), numContacts(0) 
{   
    // fmax[0] = fmax[2] = 0.0f;
    // fmin[0] = fmin[2] = -INFINITY;
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

        vec3 wA = rotate(contact.rA, bodyA);
        vec3 wB = rotate(contact.rB, bodyB);

        vec3 pA = bodyA->position + wA;
        vec3 pB = bodyB->position + wB;

        vec3 vrel = (bodyA->velocity.linear + glm::cross(bodyA->velocity.angular, wA)) -
                    (bodyB->velocity.linear + glm::cross(bodyB->velocity.angular, wB));

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

        vec3 wA = rotate(contact.rA, bodyA);
        vec3 wB = rotate(contact.rB, bodyB);

        vec3 pA = bodyA->position + wA;
        vec3 pB = bodyB->position + wB;

        vec6 dpA = bodyA->getConfiguration() - bodyA->initial;
        vec6 dpB = bodyB->getConfiguration() - bodyB->initial;

        // Compute separation distance along normal
        // If normal points from B to A, then dot(pA - pB, normal) is positive when separated
        float separation = glm::dot(pA - pB, contact.normal);

        // When C < 0, objects are too close (violating constraint)
        // When C >= 0, objects are properly separated (satisfying constraint)
        C[i * 3 + 0] = contact.C0[0] * (1 - alpha) + dot(contacts[i].JAn, dpA) + dot(contacts[i].JBn, dpB);
        
        // Disable friction in position solver
        C[i * 3 + 1] = contact.C0[1] * (1 - alpha) + dot(contacts[i].JAt1, dpA) + dot(contacts[i].JBt1, dpB);
        C[i * 3 + 2] = contact.C0[2] * (1 - alpha) + dot(contacts[i].JAt2, dpA) + dot(contacts[i].JBt2, dpB);

        // --- Update Force Limits for Friction Cone ---
        float friction_limit = friction * abs(lambda[i * 3 + 0]);
        fmin[i * 3 + 1] = -friction_limit;
        fmax[i * 3 + 1] =  friction_limit;
        fmin[i * 3 + 2] = -friction_limit;
        fmax[i * 3 + 2] =  friction_limit;
        fmin[i * 3 + 0] = -INFINITY; // Allow negative forces (pushing apart)
        fmax[i * 3 + 0] = 0;       // Prevent positive forces (pulling together)
        
        // --- Sticking Logic ---
        float tangent_lambda = sqrtf(lambda[i*3+1]*lambda[i*3+1] + lambda[i*3+2]*lambda[i*3+2]);
        contacts[i].stick = tangent_lambda < friction_limit ; //&& length(contacts[i].C0[1]) < STICK_THRESH;
    }
}

void Manifold::computeDerivatives(Rigid* body) {
    // Just store precomputed derivatives in J for the desired body
    for (int i = 0; i < numContacts; i++)
    {
        Contact& contact = contacts[i];

        bool isA = body == bodyA;

        J[i * 3 + 0] = isA ? contact.JAn : contact.JBn;
        J[i * 3 + 1] = isA ? contact.JAt1 : contact.JBt1;
        J[i * 3 + 2] = isA ? contact.JAt2 : contact.JBt2;
    }
}

void Manifold::draw() const {}