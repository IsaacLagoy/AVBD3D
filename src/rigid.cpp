#include "solver.h"

Rigid::Rigid(Solver* solver, vec3 size, float density, float friction,
             vec3 position, vec3 velocity, vec3 angularVelocity, vec4 color)
    : solver(solver), forces(nullptr), next(nullptr),
      position(position), rotation(glm::quat(1, 0, 0, 0)),
      velocity(velocity), prevVelocity(velocity), angularVelocity(angularVelocity),
      scale(size), friction(friction), color(color)
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

void Rigid::draw() {}