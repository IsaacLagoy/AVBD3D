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
    if (mass <= 0) throw std::runtime_error("Rigid body mass less than 0");
    moment = mass / 12.0f * mat3x3(
        scale.y * scale.y + scale.z * scale.z, 0, 0,
        0, scale.x * scale.x + scale.z * scale.z, 0,
        0, 0, scale.x * scale.x + scale.z * scale.z
    ); 
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
    rotation = expMapSO3(config.angular); // TODO ensure this line is correct.
}

void Rigid::draw() {}

mat6x6 Rigid::getMassMatrix() const {
    mat3x3 comSkew = skewSymmetricCrossProductMatrix(position);
    mat3x3 topLeft = mass * glm::mat3x3(1.0f);
    mat3x3 topRight = -mass * comSkew;
    mat3x3 bottomLeft = mass * comSkew;
    mat3x3 bottomRight = moment - mass * comSkew * comSkew;

    return { topLeft, topRight, bottomLeft, bottomRight };
}