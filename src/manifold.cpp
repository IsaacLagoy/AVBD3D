#include "solver.h"

Manifold::Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB) : Force(solver, bodyA, bodyB), numContacts(0) {
    fmax[0] = fmax[2] = 0.0f;
    fmin[0] = fmin[2] = -INFINITY;
}

bool Manifold::initialize() {
    // compute friction
    friction = sqrtf(bodyA->friction * bodyB->friction);

    // TODO store previous contact state

    // Compute new contacts
    numContacts = collide(bodyA, bodyB, contacts);

    print(numContacts);

    // TODO Merge old contact data with new contacts

    // initialize contact data
    for (int i = 0; i < numContacts; i++) {
        Contact& contact = contacts[i];

        // compute tangent data
        vec3 linIndep = fabs(glm::dot(contact.normal, vec3(0, 1, 0))) > 0.95 ? vec3(1, 0, 0) : vec3(0, 1, 0);
        contact.t1 = linIndep - glm::dot(linIndep, contact.normal) * contact.normal;
        contact.t1 = glm::normalize(contact.t1);
        contact.t2 = glm::cross(contact.t1, contact.normal); // guarunteed to be normal

        // stiffness and lagrange
        contact.k = 1000.0f;
        contact.lambda = vec3(0.0f);


        // print("computing Jacobians");

        // compute Jacobians
        contact.JAn = vec6(contact.normal, glm::cross(contact.rA, contact.normal));
        contact.JBn = vec6(-contact.normal, glm::cross(contact.rB, -contact.normal));

        // print("normals done");

        contact.JAt1 = vec6(contact.t1, glm::cross(contact.rA, contact.t1));
        contact.JBt1 = vec6(-contact.t1, glm::cross(contact.rB, -contact.t1));
        contact.JAt2 = vec6(contact.t2, glm::cross(contact.rA, contact.t2));
        contact.JBt2 = vec6(-contact.t2, glm::cross(contact.rB, -contact.t2));

        // print("jacobians computed");

        // compute error
        contact.C0 = vec3(-contact.depth, 0, 0);

        // friction variables
        contact.stick = true;
    }

    return numContacts > 0;
}

void Manifold::computeConstraint(float alpha) {
    // compute positional changes
    vec6 dA = bodyA->getConfiguration() - bodyA->initial;
    vec6 dB = bodyB->getConfiguration() - bodyB->initial;

    for (int i = 0; i < numContacts; i++) {
        Contact& contact = contacts[i];

        // print("Computing constraint");

        // Taylor Approximation of C(x)
        vec3 newC = contact.C0 * (1.0f - alpha) + vec3(
            dot(contact.JAn, dA) + dot(contact.JBn, dB), // Cn
            dot(contact.JAt1, dA) + dot(contact.JBt1, dB), // Ct1
            dot(contact.JAt2, dA) + dot(contact.JBt2, dB) // Ct2
        );
        
        C[i * 3 + 0] = newC.x;
        C[i * 3 + 1] = newC.y;
        C[i * 3 + 2] = newC.z;

        // print(newC);

        // Update friction bounds
        float frictionBound = abs(contact.lambda.x) * friction;
        vec3 newfmax = vec3(0.0f, frictionBound, frictionBound);
        vec3 newfmin = vec3(0.0f, -frictionBound, -frictionBound);

        fmax[i * 3 + 0] = newfmax.x;
        fmax[i * 3 + 1] = newfmax.y;
        fmax[i * 3 + 2] = newfmax.z;
        fmin[i * 3 + 0] = newfmin.x;
        fmin[i * 3 + 1] = newfmin.y;
        fmin[i * 3 + 2] = newfmin.z;

        contact.stick = (abs(contact.lambda.y) < frictionBound && abs(contact.lambda.z) < frictionBound && abs(contact.C0.y) < STICK_THRESH && abs(contact.C0.z) < STICK_THRESH);
    }
}

void Manifold::computeDerivatives(Rigid* body) {
    // Just store precomputed derivatives in J for the desired body
    for (int i = 0; i < numContacts; i++)
    {
        Contact& contact = contacts[i];

        // compute constraint jacobian matrix
        mat3x6 JA(contact.JAt1, contact.JAt2, contact.JAn);
        mat3x6 JB(contact.JBt1, contact.JBt2, contact.JBn);

        // hessian term: k * J^T * J
        float k = contact.k;
        mat6x6 HA = transpose(JA) * JA * k;
        mat6x6 HB = transpose(JB) * JB * k;

        if (body == bodyA)
        {
            J[i * 3 + 0] = contacts[i].JAn;
            J[i * 3 + 1] = contacts[i].JAt1;
            J[i * 3 + 2] = contacts[i].JAt2;
            
        }
        else
        {
            J[i * 3 + 0] = contacts[i].JBn;
            J[i * 3 + 1] = contacts[i].JBt1;
            J[i * 3 + 2] = contacts[i].JBt2;
        }
    }

    // print("Finished computing derivatives");
}


void Manifold::draw() const {}