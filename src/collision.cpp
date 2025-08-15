#include "collision.h"
#include <cmath>

// helper functions
vec3 transform(const vec3& vertex, Rigid* body) {
    vec4 four = vec4(vertex, 1.0f);
    return vec3(buildModelMatrix(body) * four);
}

vec3 transform(int index, Rigid* body) {
    vec3 vertex = Mesh::uniqueVerts[index];
    return transform(vertex, body);
}

vec3 rotateNScale(const vec3& vertex, Rigid* body) {
    return body->rotation * (vertex * body->scale);
}

vec3 rotateNScale(int index, Rigid* body) {
    vec3 vertex = Mesh::uniqueVerts[index];
    return rotateNScale(vertex, body);
}

int bestDot(Rigid* body, const vec3& dir) {
    // transform dir to model space
    vec3 inv = glm::inverse(body->rotation) * dir;
    return Mesh::bestDot(inv);
}

SupportPoint getSupportPoint(Rigid* bodyA, Rigid* bodyB, const vec3& dir) {
    int indexA = bestDot(bodyA, -dir);
    int indexB = bestDot(bodyB, dir);
    return { indexA, indexB, transform(indexB, bodyB) - transform(indexA, bodyA) };
}

// Main
int Manifold::collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts) {
    // run collision detection
    Simplex simplex = Simplex(); // can prolly go on the stack idk, there's only one rn
    bool collided = gjk(bodyA, bodyB, simplex);

    if (!collided) return 0;

    // run collision resolution
    Polytope* polytope = new Polytope(simplex);
    epa(bodyA, bodyB, polytope);

    if (hasNaN(polytope->front().normal)) std::runtime_error("normal has nan");

    std::vector<vec3> rAs, rBs;
    getContact(rAs, rBs, polytope, bodyA, bodyB);

    // compute contact information
    contacts[0].normal = polytope->front().normal;
    contacts[0].depth = polytope->front().distance;
    contacts[0].face = polytope->front();

    contacts[0].rA = inverseTransform(rAs[0], bodyA);
    contacts[0].rB = inverseTransform(rBs[0], bodyB);

    // ensure normal is facing the correct direction
    if (glm::dot(contacts[0].normal, bodyA->position - bodyB->position) < 0) contacts[0].normal *= -1;

    delete polytope;
    return 1;
}