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

vec3 rotate(const vec3& vertex, Rigid* body) {
    return body->rotation * vertex;
}

vec3 rotate(int index, Rigid* body) {
    vec3 vertex = Mesh::uniqueVerts[index];
    return rotate(vertex, body);
}

int bestDot(Rigid* body, const vec3& dir) {
    // transform dir to model space
    if (DEBUG_PRINT_GJK) print("dir best Dot");
    if (DEBUG_PRINT_GJK) print(dir);
    vec3 inv = glm::normalize(glm::inverse(body->rotation)) * dir;
    if (DEBUG_PRINT_GJK) print("inv");
    if (DEBUG_PRINT_GJK) print(body->rotation);
    if (DEBUG_PRINT_GJK) print(inv);
    return Mesh::bestDot(inv);
}

SupportPoint getSupportPoint(Rigid* bodyA, Rigid* bodyB, const vec3& dir) {

    if (DEBUG_PRINT_GJK) print("BestDot A");
    int indexA = bestDot(bodyA, dir);
    if (DEBUG_PRINT_GJK) print("Best dot B");
    int indexB = bestDot(bodyB, -dir);
    return { indexA, indexB, transform(indexA, bodyA) - transform(indexB, bodyB) };
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

    std::pair<vec3, vec3> rs = barycentric(polytope, bodyA, bodyB);

    contacts[0].rA = rs.first;
    contacts[0].rB = rs.second;

    print("contacts");
    print(contacts[0].rA);
    print(contacts[0].rB);
    print(contacts[0].normal);

    delete polytope;
    return 1;
}