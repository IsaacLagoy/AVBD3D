#include "rigid.h"

Rigid::Rigid(Solver* solver, vec3 size, float density, float friction,
             vec3 position, quat rotation, vec6 velocity, vec4 color)
    :   solver(solver),
        forces(nullptr), 
        next(nullptr),
        position(position), 
        rotation(glm::normalize(rotation)),
        velocity(velocity), 
        prevVelocity(velocity),
        initialPosition(),
        inertialPosition(),
        initialRotation(),
        inertialRotation(),
        scale(size), 
        friction(friction), 
        color(color)
{
    // Add to linked list
    next = solver->bodies;
    solver->bodies = this;

    mass = scale.x * scale.y * scale.z * density;
    float invMass = 1.0f / mass;

    if (mass > 0) {
        float Ixx = (1.0f / 12.0f) * mass * (size.y * size.y + size.z * size.z);
        float Iyy = (1.0f / 12.0f) * mass * (size.x * size.x + size.z * size.z);
        float Izz = (1.0f / 12.0f) * mass * (size.x * size.x + size.y * size.y);

        inertiaTensor = mat3x3(
            {Ixx, 0, 0},
            {0, Iyy, 0},
            {0, 0, Izz}
        );
    } else {
        inertiaTensor = mat3x3({0,0,0}, {0,0,0}, {0,0,0});
    }
    radius = glm::length(scale); // max half extent magnitude
}

Rigid::~Rigid() {
    // Remove from linked list
    Rigid** p = &solver->bodies;
    while (*p != this)
        p = &(*p)->next;
    *p = next;
}

bool Rigid::constrainedTo(Rigid* other) const {
    // check if this body is constrained to the other body
    for (Force* f = forces; f != nullptr; f = f->next)
        if ((f->bodyA == this && f->bodyB == other) || (f->bodyA == other && f->bodyB == this)) 
            return true;
    return false;
}

mat6x6 Rigid::getMassMatrix() const {
    mat3x3 topLeft = mass * glm::mat3x3(1.0f);
    mat3x3 bottomRight = getInertiaTensor();

    return { topLeft, mat3x3(), mat3x3(), bottomRight };
}

mat3x3 Rigid::getInertiaTensor() const {
    mat3x3 R(rotation);
    return R * inertiaTensor * glm::transpose(R);
}

vec3 Rigid::deltaWInitial() const {
    quat rel = 2.0f * (rotation * glm::inverse(initialRotation));
    return {rel.x, rel.y, rel.z};
}

vec3 Rigid::deltaWInertial() const {
    quat rel = 2.0f * (rotation * glm::inverse(inertialRotation));
    return {rel.x, rel.y, rel.z};
}

// helper functions
mat4x4 buildModelMatrix(const Rigid* b) {
    mat4x4 translation = glm::translate(mat4x4(1), b->position);
    mat4x4 scaling = glm::scale(mat4x4(1), b->scale);
    mat4x4 rotate = mat4x4(b->rotation);

    return translation * rotate * scaling;
}

mat4x4 buildModelMatrix(const vec3& pos, const vec3& sca, const quat& rot) {
    mat4x4 translation = glm::translate(mat4x4(1), pos);
    mat4x4 scaling = glm::scale(mat4x4(1), sca);
    mat4x4 rotate = mat4x4(rot);

    return translation * rotate * scaling;
}

glm::mat4 buildInverseModelMatrix(const Rigid* b) {
    glm::mat4 invTranslation = glm::translate(glm::mat4(1), -b->position);
    glm::mat4 invRotation = glm::mat4(glm::conjugate(b->rotation));
    glm::mat4 invScaling = glm::scale(glm::mat4(1), 1.0f / b->scale);
    
    return invRotation * invScaling * invTranslation;
}

glm::vec3 inverseTransform(const glm::vec3& worldPoint, Rigid* body) {
    // Undo translation
    glm::vec3 p = worldPoint - body->position;

    // Undo rotation
    glm::quat invRot = glm::conjugate(glm::normalize(body->rotation));
    p = invRot * p;

    // Undo non-uniform scale
    p /= body->scale; // component-wise division

    return p;
}