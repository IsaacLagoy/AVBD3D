#include "solver.h"

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB) 
    : Force(solver, bodyA, bodyB), numContacts(0) 
{
    // fmax[0] = fmax[2] = 0.0f;
    // fmin[0] = fmin[2] = -INFINITY;

    

    // The constructor is minimal, all work is done in initialize
    for (int i = 0; i < 4; ++i) {
        contacts[i].C0_n = 0.0f;
        contacts[i].C0_t = vec3{0, 0, 0};
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

        vec3 pA = bodyA->position + contact.rA;
        vec3 pB = bodyB->position + contact.rB;

        // Precompute C0 - the constraint violation at the beginning of this frame
        // Following 2D: C0 = basis * (pA - pB) + collision_margin
        // Use the stored penetration depth from collision detection
        contacts[i].C0_n = -contacts[i].depth; // Negative because penetration is positive when objects overlap

        vec3 vrel = (bodyA->velocity.linear + glm::cross(bodyA->velocity.angular, contact.rA)) -
                    (bodyB->velocity.linear + glm::cross(bodyB->velocity.angular, contact.rB));

        // compute tangent data
        vec3 linIndep = fabs(glm::dot(contact.normal, vec3(0, 1, 0))) > 0.95 ? vec3(1, 0, 0) : vec3(0, 1, 0);
        contact.t1 = linIndep - glm::dot(linIndep, contact.normal) * contact.normal;
        contact.t1 = glm::normalize(contact.t1);
        contact.t2 = glm::cross(contact.t1, contact.normal); // guarunteed to be normal

        // // compute Jacobians
        // contact.JAn = vec6(contact.normal, glm::cross(contact.rA, contact.normal));
        // contact.JBn = vec6(-contact.normal, glm::cross(contact.rB, -contact.normal));

        // contact.JAt1 = vec6(contact.t1, glm::cross(contact.rA, contact.t1));
        // contact.JBt1 = vec6(-contact.t1, glm::cross(contact.rB, -contact.t1));
        // contact.JAt2 = vec6(contact.t2, glm::cross(contact.rA, contact.t2));
        // contact.JBt2 = vec6(-contact.t2, glm::cross(contact.rB, -contact.t2));

        // contact.stick = true;

        
        // vec3 displacement = pB - pA; // TODO ensure that this subtraction is in the correct direction

        // contact.C0 = vec3(
        //     glm::dot(displacement, contact.normal) + COLLISION_MARGIN,
        //     glm::dot(displacement, contact.t1),
        //     glm::dot(displacement, contact.t2)
        // );

        contacts[i].C0_t.x = glm::dot(vrel, contact.t1);
        contacts[i].C0_t.y = glm::dot(vrel, contact.t2);
        contacts[i].C0_t.z = 0;

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

        vec3 pA = bodyA->position + contact.rA;
        vec3 pB = bodyB->position + contact.rB;

        // Compute separation distance along normal
        // If normal points from B to A, then dot(pA - pB, normal) is positive when separated
        float separation = glm::dot(pA - pB, contact.normal);

        // When C < 0, objects are too close (violating constraint)
        // When C >= 0, objects are properly separated (satisfying constraint)
        C[i * 3 + 0] = separation;
        
        // Disable friction in position solver
        C[i * 3 + 1] = 0.0f;
        C[i * 3 + 2] = 0.0f;

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
        contacts[i].stick = tangent_lambda < friction_limit && length(contacts[i].C0_t) < STICK_THRESH;

        // // Taylor Approximation of C(x)
        // vec3 newC = contact.C0 * (1.0f - alpha) + vec3(
        //     dot(contact.JAn, dA) + dot(contact.JBn, dB), // Cn
        //     dot(contact.JAt1, dA) + dot(contact.JBt1, dB), // Ct1
        //     dot(contact.JAt2, dA) + dot(contact.JBt2, dB) // Ct2
        // );
        
        // C[i * 3 + 0] = newC.x;
        // C[i * 3 + 1] = newC.y;
        // C[i * 3 + 2] = newC.z;

        // // Update friction bounds
        // float frictionBound = abs(lambda[i * 3 + 0]) * friction;
        // vec3 newfmax = vec3(0.0f, frictionBound, frictionBound); // n t1 t2
        // vec3 newfmin = vec3(0.0f, -frictionBound, -frictionBound);

        // fmax[i * 3 + 0] = newfmax.x;
        // fmax[i * 3 + 1] = newfmax.y;
        // fmax[i * 3 + 2] = newfmax.z;
        // fmin[i * 3 + 0] = newfmin.x;
        // fmin[i * 3 + 1] = newfmin.y;
        // fmin[i * 3 + 2] = newfmin.z;

        // contact.stick = (abs(lambda[i * 3 + 1]) < frictionBound && abs(lambda[i * 3 + 2]) < frictionBound && abs(contact.C0.y) < STICK_THRESH && abs(contact.C0.z) < STICK_THRESH);
    }
}

void Manifold::computeDerivatives(Rigid* body) {
    // Just store precomputed derivatives in J for the desired body
    for (int i = 0; i < numContacts; i++)
    {
        Contact& contact = contacts[i];

        bool isA = body == bodyA;
        float sign = isA ? 1.0f : -1.0f;
        const vec3& r = isA ? contact.rA : contact.rB;

        J[i * 3 + 0] = vec6(contact.normal * sign, glm::cross(r, contact.normal) * sign);
        J[i * 3 + 1] = vec6(contact.t1 * sign, glm::cross(r, contact.t1) * sign);
        J[i * 3 + 2] = vec6(contact.t2 * sign, glm::cross(r, contact.t2) * sign);

        // // compute constraint jacobian matrix
        // mat3x6 JA(contact.JAt1, contact.JAt2, contact.JAn);
        // mat3x6 JB(contact.JBt1, contact.JBt2, contact.JBn);

        // // hessian term: k * J^T * J
        // // float k = contact.k;
        // // mat6x6 HA = transpose(JA) * JA * k;
        // // mat6x6 HB = transpose(JB) * JB * k;

        // if (body == bodyA)
        // {
        //     J[i * 3 + 0] = contacts[i].JAn;
        //     J[i * 3 + 1] = contacts[i].JAt1;
        //     J[i * 3 + 2] = contacts[i].JAt2;
            
        // }
        // else
        // {
        //     J[i * 3 + 0] = contacts[i].JBn;
        //     J[i * 3 + 1] = contacts[i].JBt1;
        //     J[i * 3 + 2] = contacts[i].JBt2;
        // }
    }
}

void Manifold::draw() const {}