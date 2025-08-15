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

    // store previous contact state
    Contact oldContacts[4]; for (int i = 0; i < 4; i++) oldContacts[i] = contacts[i];
    float oldPenalty[MAX_ROWS]; for (int i = 0; i < MAX_ROWS; i++) oldPenalty[i] = penalty[i];
    float oldLambda[MAX_ROWS];  for (int i = 0; i < MAX_ROWS; i++) oldLambda[i] = lambda[i];
    bool oldStick[4]; for (int i = 0; i < 4; i++) oldStick[i] = contacts[i].stick;
    int oldNumContacts = numContacts;

    // Compute new contacts
    // print("before collision");
    numContacts = collide(bodyA, bodyB, contacts);
    if (numContacts == 0) return false;
    // print("after collision");

    // Merge old contact data with new contacts
    for (int i = 0; i < numContacts; i++) {
        penalty[i * 3 + 0] = penalty[i * 3 + 1] = penalty[i * 3 + 2] = 0.0f;
        lambda[i * 3 + 0] = lambda[i * 3 + 1] = lambda[i * 3 + 2] = 0.0f;

        for (int j = 0; j < oldNumContacts; j++) {
            if (contacts[i] == oldContacts[j]) {
                for (int k = 0; k <  3; k++) penalty[i * 3 + k] = oldPenalty[j * 3 + k];
                for (int k = 0; k <  3; k++) lambda[i * 3 + k] = oldLambda[j * 3 + k];
                contacts[i].stick = oldStick[j];

                // If static friction in last frame, use the old contact points
                // TODO I'm not sure if this does anything 
                if (oldStick[j]) {
                    contacts[i].rA = oldContacts[j].rA;
                    contacts[i].rB = oldContacts[j].rB;
                }
            }
        }
    }

    // initialize contact data
    for (int i = 0; i < numContacts; i++) {
        Contact& contact = contacts[i];

        // TODO check if scale needs to be added here
        vec3 wA = rotateNScale(contact.rA, bodyA);
        vec3 wB = rotateNScale(contact.rB, bodyB);

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

        vec3 drX = bodyA->position + wA - bodyB->position - wB;
        contact.C0.x = glm::dot(contact.normal, drX) + COLLISION_MARGIN;
        contact.C0.y = glm::dot(contact.t1,     drX);
        contact.C0.z = glm::dot(contact.t2,     drX);
    }

    // print("end initialize");
    return true;
}

void Manifold::computeConstraint(float alpha) {
    // print("compute constraint");
    // compute positional changes
    for (int i = 0; i < numContacts; i++) {
        // --- Simple, Direct Constraint Calculation ---
        // Goal: C = 0 when objects are just touching, C < 0 when penetrating
        Contact& contact = contacts[i];

        // print(contact.C0);
        // print(contact.JAn);
        // print(contact.JAt1);
        // print(contact.JAt2);
        // print(contact.JBn);
        // print(contact.JBt1);
        // print(contact.JBt2);

        // print(bodyA->position - bodyA->initialPosition);
        // print(bodyA->deltaWInitial());
        // print(bodyB->position - bodyB->initialPosition);
        // print(bodyB->deltaWInitial());

        vec6 dpA = { bodyA->position - bodyA->initialPosition, bodyA->deltaWInitial() };
        vec6 dpB = { bodyB->position - bodyB->initialPosition, bodyB->deltaWInitial() };

        // When C < 0, objects are too close (violating constraint)
        // When C >= 0, objects are properly separated (satisfying constraint)
        C[i * 3 + 0] = contact.C0.x * (1 - alpha) + dot(contact.JAn,  dpA) + dot(contact.JBn,  dpB);
        C[i * 3 + 1] = contact.C0.y * (1 - alpha) + dot(contact.JAt1, dpA) + dot(contact.JBt1, dpB);
        C[i * 3 + 2] = contact.C0.z * (1 - alpha) + dot(contact.JAt2, dpA) + dot(contact.JBt2, dpB);

        // --- Update Force Limits for Friction Cone ---
        float frictionBound = abs(lambda[i * 3 + 0]) * friction;
        fmax[i * 3 + 1] = frictionBound;
        fmin[i * 3 + 1] = -frictionBound;
        fmax[i * 3 + 2] = frictionBound;
        fmin[i * 3 + 2] = -frictionBound;
        
        // --- Sticking Logic ---
        contact.stick = abs(lambda[i * 3 + 1]) < frictionBound && abs(contact.C0.z) < STICK_THRESH; // TODO check this convertsion to 3d
    }

    // print("end compute constraint");
}

void Manifold::computeDerivatives(Rigid* body) {
    // print("begin compute derivatives");
    // Just store precomputed derivatives in J for the desired body
    for (int i = 0; i < numContacts; i++)
    {
        Contact& contact = contacts[i];
        
        bool isA = body == bodyA;

        J[i * 3 + 0] = isA ? contact.JAn  : contact.JBn;
        J[i * 3 + 1] = isA ? contact.JAt1 : contact.JBt1;
        J[i * 3 + 2] = isA ? contact.JAt2 : contact.JBt2;
    }
    // print("end compute derivaties");
}