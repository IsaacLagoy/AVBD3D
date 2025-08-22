#include "rigid.h"

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
    Contact oldContacts[4]; for (int i = 0; i < numContacts; i++) oldContacts[i] = contacts[i];
    float oldPenalty[MAX_ROWS]; for (int i = 0; i < MAX_ROWS; i++) oldPenalty[i] = penalty[i];
    float oldLambda[MAX_ROWS];  for (int i = 0; i < MAX_ROWS; i++) oldLambda[i] = lambda[i];
    bool oldStick[4]; for (int i = 0; i < numContacts; i++) oldStick[i] = contacts[i].stick;
    int oldType[4]; for (int i = 0; i < numContacts; i++) oldType[i] = contacts[i].type;
    bool oldContactUsed[4]; for (int i = 0; i < numContacts; i++) oldContactUsed[i] = false;

    int oldNumContacts = numContacts;
    for (int i = 0; i < oldNumContacts; i++) {
        oldContacts[i] = contacts[i];
        for (int k = 0; k < 3; k++) {
            oldPenalty[i * 3 + k] = penalty[i * 3 + k];
            oldLambda[i * 3 + k]  = lambda[i * 3 + k];
        }
    }

    // compute new contacts
    numContacts = collide(bodyA, bodyB, contacts);
    if (numContacts == 0) return false;

    // calculate 
    bool canBeUsed[4]; 
    int sumContacts = 0;
    for (int i = 0; i < oldNumContacts; i++) {
        canBeUsed[i] = !oldContactUsed[i];
        if (canBeUsed[i]) sumContacts++;
    }

    // check if old contacts should still be used
    if (numContacts < 4 && sumContacts > 0) {

        // check if contact is still in the same place
        for (int i = 0; i < oldNumContacts; i++) {
            if (!canBeUsed[i]) continue;
            const Contact& contact = oldContacts[i];

            // ensure all minkowski difference support points are still in location
            vec3 pOnA = transform(contact.rA, bodyA);
            vec3 pOnB = transform(contact.rB, bodyB);
            vec3 sep = pOnA - pOnB;
            
            // determine if seperation is in direction of the current normal
            if (glm::dot(contacts[0].normal, sep) > COLLISION_MARGIN ) {
                canBeUsed[i] = false;
                sumContacts--;
                continue;
            }

            // check if minkowski points have drifted too much
            for (int j = 0; j < 3; j++) {
                vec3 curMink = transform(contact.face.sps[j].indexB, bodyB) - transform(contact.face.sps[j].indexA, bodyA);
                if (glm::length2(curMink - contact.face.sps[j].mink) > COLLISION_MARGIN) {
                    canBeUsed[i] = false;
                    sumContacts--;
                    break;
                }
            }
        }

        if (sumContacts > 0) {
            // pick best old contact points to update
            // TODO find better selection algorithm
            vec3 tot = vec3();
            for (int i = 0; i < numContacts; i++) tot += (contacts[i].rA + contacts[i].rB) / 2.0f;

            vec3 avgs[4];
            for (int i = 0; i < oldNumContacts; i++) avgs[i] = (oldContacts[i].rA + oldContacts[i].rB) / 2.0f;

            while (numContacts < 4 && sumContacts > 0) {
                vec3 center = tot / (float) numContacts;
                float bestScore = -1;
                int oldIndex = -1;

                // find furthest point from center
                for (int i = 0; i < oldNumContacts; i++) {
                    if (!canBeUsed[i]) continue;
                    float score = glm::length2(center - avgs[i]);
                    if (bestScore == -1 || bestScore < score) {
                        bestScore = score;
                        oldIndex = i;
                    }
                }

                // add best old point to new point
                canBeUsed[oldIndex] = false;
                tot += avgs[oldIndex];

                contacts[numContacts] = oldContacts[oldIndex];
                for (int k = 0; k < 3; k++) penalty[numContacts * 3 + k] = oldPenalty[oldIndex * 3 + k];
                for (int k = 0; k < 3; k++) lambda[numContacts * 3 + k] = oldLambda[oldIndex * 3 + k];
                contacts[numContacts].stick = oldStick[oldIndex];
                contacts[numContacts].type = oldType[oldIndex];

                // If static friction in last frame, use the old contact points
                if (oldStick[oldIndex]) {
                    contacts[numContacts].rA = oldContacts[oldIndex].rA;
                    contacts[numContacts].rB = oldContacts[oldIndex].rB;
                }

                sumContacts--;
                numContacts++;
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
        contact.C0.x = glm::dot(contact.normal, drX); // + COLLISION_MARGIN;
        contact.C0.y = glm::dot(contact.t1,     drX);
        contact.C0.z = glm::dot(contact.t2,     drX);
    }

    return true;
}

void Manifold::computeConstraint(float alpha) {
    // compute positional changes
    for (int i = 0; i < numContacts; i++) {
        // --- Simple, Direct Constraint Calculation ---
        // Goal: C = 0 when objects are just touching, C < 0 when penetrating
        Contact& contact = contacts[i];

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
}

void Manifold::computeDerivatives(Rigid* body) {
    // Just store precomputed derivatives in J for the desired body
    for (int i = 0; i < numContacts; i++)
    {
        Contact& contact = contacts[i];
        
        bool isA = body == bodyA;

        // compute Jacobians
        J[i * 3 + 0] = isA ? contact.JAn  : contact.JBn;
        J[i * 3 + 1] = isA ? contact.JAt1 : contact.JBt1;
        J[i * 3 + 2] = isA ? contact.JAt2 : contact.JBt2;

        // compute Hessians
        for (int j = 0; j < 3; j++) {
            vec3 dir = J[i * 3 + j].linear;
            vec3 s = isA ? rotateNScale(contact.rA, bodyA) : rotateNScale(contact.rB, bodyB);
            H[i * 3 + j] = mat6x6();
            H[i * 3 + j].addBottomRight(lambda[i] * (0.5f * (outer(dir, s) + outer(s, dir) - glm::dot(dir, s) * glm::diagonal3x3(vec3(1.0f)))));

            // H[i * 3 + j] = mat6x6();
            // mat3x3 inertia = isA ? bodyA->getInertiaTensor() : bodyB->getInertiaTensor();
            // H[i * 3 + j].addBottomRight(glm::diagonal3x3(glm::abs(glm::cross(J[i * 3 + j].angular, inertia * J[i * 3 + j].angular))));
        }
    }
}

bool Manifold::isContactStillValid(const Contact& c, Rigid* A, Rigid* B)
{
    // Tolerances
    const float margin = COLLISION_MARGIN;
    const float keepSlack = 2.0f * margin;      // allow a little separation
    const float maxNormalAngleCos = 0.8f;       // ~36°; relax/tighten as needed
    const float maxTangentialSlip = 2.0f * margin;

    // --- 1) Quick SAT along the OLD normal ---
    // If the separation along the old normal is small or negative (penetrating), keep it.
    // supportX(dir) should return WORLD-SPACE support point on shape X in direction 'dir'.
    const SupportPoint s = getSupportPoint(A, B, c.normal);      // world
    const vec3 sA = transform(s.indexA, A);
    const vec3 sB = transform(s.indexB, B);
    const float sep = glm::dot(sB - sA, c.normal);
    if (sep > keepSlack) return false;

    // --- 2) Optional: reject if the best direction drifted too far ---
    // (Re-estimate a direction from those supports; cheap & stable.)
    const vec3 span = sB - sA;
    if (glm::length2(span) > 1e-12f) {
        const vec3 nNew = glm::normalize(span);
        if (glm::dot(nNew, c.normal) < maxNormalAngleCos) {
            return false;
        }
    }

    // --- 3) Optional: if we "stuck" the anchors, limit tangential slip ---
    // IMPORTANT: this assumes rA/rB are LOCAL anchors. If you sometimes store world-space,
    // either always store local or branch here to skip the transform for world-space anchors.
    if (c.stick) {
        const vec3 pA = transform(c.rA, A); // xA + RA * rA_local
        const vec3 pB = transform(c.rB, B); // xB + RB * rB_local

        const vec3 d   = pB - pA;
        const float dn = glm::dot(d, c.normal);
        const vec3 dt  = d - dn * c.normal;    // lateral drift

        if (glm::length2(dt) > maxTangentialSlip * maxTangentialSlip) {
            return false;
        }
        // (Optionally also bound |dn| against keepSlack if you want tighter control.)
    }

    return true;
}
