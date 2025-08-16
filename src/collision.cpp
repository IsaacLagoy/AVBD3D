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

    if (rAs.size() != rBs.size()) throw std::runtime_error("Contact point size missmatch");

    int size = glm::clamp((int) rAs.size(), 0, 4);

    for (int i = 0; i < size; i++) {
        // compute contact information
        contacts[i].normal = polytope->front().normal;
        contacts[i].depth = polytope->front().distance * 10;
        contacts[i].face = polytope->front();
        contacts[i].rA = inverseTransform(rAs[i], bodyA);
        contacts[i].rB = inverseTransform(rBs[i], bodyB);

        if (hasNaN(rAs[i])) throw std::runtime_error("Contact point from rA has Nan");
        if (hasNaN(rBs[i])) throw std::runtime_error("Contact point from rB has Nan");
    }

    // ensure normal is facing the correct direction
    for (int i = 0; i < size; i++) if (glm::dot(contacts[i].normal, bodyA->position - bodyB->position) < 0)  contacts[i].normal *= -1;

    delete polytope;
    return size;
}