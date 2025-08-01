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
        scale(size), 
        friction(friction), 
        color(color)
{
    // Add to linked list
    next = solver->bodies;
    solver->bodies = this;

    mass = scale.x * scale.y * scale.z * density;
    moment = mat3x3(1); // TODO: replace with actual box inertia
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
    position = config.linear;
    rotation = expMapSO3(config.angular); // TODO ensure this line is correct.
}

void Rigid::draw() {}

mat6x6 Rigid::getMassMatrix() const {
    mat3x3 comSkew = skewSymmetricCrossProductMatrix(position);
    mat3x3 comSkewT = -comSkew;

    mat3x3 topLeft = moment + mass * comSkew * comSkewT;
    mat3x3 bottomRight = mat3x3(mass, 0, 0, 0, mass, 0, 0, 0, mass);
    mat3x3 topRight = mass * comSkewT;
    mat3x3 bottomLeft = mass * comSkew;

    return { topLeft, topRight, bottomLeft, bottomRight };
}