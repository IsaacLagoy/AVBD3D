#include "solver.h"

Rigid::Rigid(Solver* solver, vec3 size, float density, float friction,
             vec3 position, vec6 velocity, vec4 color)
    :   solver(solver), 
        forces(nullptr), 
        next(nullptr),
        position(position), 
        rotation(glm::quat(1, 0, 0, 0)),
        velocity(velocity), 
        prevVelocity(velocity),
        initial(),
        inertial(),
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

        invInertiaTensor = mat3x3(
            {1.0f / Ixx, 0, 0},
            {0, 1.0f / Iyy, 0},
            {0, 0, 1.0f / Izz}
        );
    } else {
        invInertiaTensor = mat3x3({0,0,0}, {0,0,0}, {0,0,0});
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

vec6 Rigid::getConfiguration() const {
    return vec6(position, logMapSO3(rotation));
}

void Rigid::setConfiguration(const vec6& config) {
    if (hasNaN(config.linear)) throw std::runtime_error("setConfiguration has NaN linear component");
    position = config.linear;
    rotation = glm::normalize(expMapSO3(config.angular));
}

void Rigid::draw() {}

mat6x6 Rigid::getMassMatrix() const {
    mat3x3 topLeft = mass * glm::mat3x3(1.0f);
    mat3x3 bottomRight = glm::transpose(getInvInertiaTensor());

    return { topLeft, mat3x3(), mat3x3(), bottomRight };
}

mat3x3 Rigid::getInvInertiaTensor() const {
    mat3x3 R(rotation);
    return R * invInertiaTensor * glm::transpose(R);
}
