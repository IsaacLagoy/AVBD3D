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

    if (DEBUG_PRINT_GJK) print("position");
    if (DEBUG_PRINT_GJK) print(bodyA->position);
    if (DEBUG_PRINT_GJK) print(bodyB->position);

    if (!collided) return 0;

    if (DEBUG_PRINT_GJK) print("collided");
    if (DEBUG_PRINT_GJK) print((int) simplex.size());
    for (int i = 0; i < 4; i++) if (DEBUG_PRINT_GJK) print(simplex[i].mink);

    // run collision resolution
    Polytope* polytope = new Polytope(simplex);

    if (DEBUG_PRINT_GJK) print("created polytope");

    epa(bodyA, bodyB, polytope);

    if (DEBUG_PRINT_GJK) print("epa");

    if (hasNaN(polytope->front().normal)) std::runtime_error("normal has nan");

    if (DEBUG_PRINT_GJK) print("stats");
    if (DEBUG_PRINT_GJK) print(polytope->front().normal);
    if (DEBUG_PRINT_GJK) for (int i = 0; i < 3; i++) print(polytope->front().sps[i]->mink);

    // compute contact information
    contacts[0].normal = polytope->front().normal;
    contacts[0].depth = polytope->front().distance;

    // add mink points to contact
    for (int i = 0; i < 3; i++) {
        contacts[0].CA[i] = Mesh::uniqueVerts[polytope->front().sps[i]->indexA];
        contacts[0].CB[i] = Mesh::uniqueVerts[polytope->front().sps[i]->indexB];
    }

    std::pair<vec3, vec3> rs = getContact(polytope, bodyA, bodyB);

    contacts[0].rA = inverseTransform(rs.first, bodyA);
    contacts[0].rB = inverseTransform(rs.second, bodyB);

    bodyA->color = vec4(1, 0, 0, 1);
    bodyB->color = vec4(0, 0, 1, 1);

    // print("model space contacts");
    // print(rs.first);
    // print(rs.second);

    // ensure normal is facing the correct direction
    if (glm::dot(contacts[0].normal, bodyA->position - bodyB->position) < 0) {
        contacts[0].normal *= -1;
        print("flipping normal");
    }

    delete polytope;
    return 1;
}