#include "collision.h"

bool gjk(Rigid* bodyA, Rigid* bodyB, Simplex& simplex) {
    vec3 dir;
    for (unsigned short i = 0; i < 20; i++) {

        // defines direction and determins if collision has happened
        bool detected = handleSimplex(simplex, bodyA, bodyB, dir); 

        // return early if collision is found
        if (detected) return true;
        // add a new point to simplex
        simplex.add(getSupportPoint(bodyA, bodyB, dir));
        
        // check if that point was discovered past the origin
        if (glm::dot(simplex[simplex.size() - 1].mink, dir) < 0) return false;
    }

    return false;
}

bool handleSimplex(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    switch(simplex.size()) {
        case 0: return simplex0(simplex, bodyA, bodyB, dir);
        case 1: return simplex1(simplex, bodyA, bodyB, dir);
        case 2: return simplex2(simplex, bodyA, bodyB, dir);
        case 3: return simplex3(simplex, bodyA, bodyB, dir);
        case 4: return simplex4(simplex, bodyA, bodyB, dir);
        default: throw std::runtime_error("Simplex has an unrecognized number of vertices during GJK.");
    }
}

bool simplex0(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    dir = bodyA->position - bodyB->position;
    if (glm::length2(dir) < 1e-6f) dir = vec3(0, 1, 0);
    return false;
}

bool simplex1(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    dir *= -1;
    return false;
}

bool simplex2(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    vec3 vecBA = simplex[B].mink - simplex[A].mink;
    vec3 vecBo = -simplex[A].mink;
    dir = glm::cross(glm::cross(vecBA, vecBo), vecBA);

    // if all points are co-linear, fallback
    if (glm::length2(dir) < 1e-6f) {
        vec3 fallback = glm::abs(vecBA.x) < glm::abs(vecBA.z) ? vec3(1, 0, 0) : vec3(0, 0, 1);
        dir = glm::cross(vecBA, fallback);
    }

    return false;
}

bool simplex3(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    dir = glm::cross(simplex[B].mink - simplex[C].mink, simplex[A].mink - simplex[C].mink);

    // ensure that it is pointing towards the origin
    vec3 vecCo = -simplex[C].mink;
    if (glm::dot(dir, vecCo) < 0) dir *= -1;
    return false;
}

bool simplex4check(const vec3& normal, const vec3& Do, Index index, Simplex& simplex, vec3& dir) {
    if (glm::dot(normal, Do) > 0) {
        vec3 io = -simplex[index].mink;
        if (glm::length2(io) < 1e-7) return true;
        dir = (glm::dot(normal, io) > 0) ? normal : -normal;
        simplex.remove(index);
        return true;
    }
    return false;
}

// check the simplex to see if it contains the origin
bool simplex4(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    vec3 DC = simplex[C].mink - simplex[D].mink;
    vec3 DB = simplex[B].mink - simplex[D].mink;
    vec3 DA = simplex[A].mink - simplex[D].mink;
    vec3 Do = -simplex[D].mink;

    vec3 DCB = cross(DB, DC);
    vec3 DBA = cross(DA, DB);
    vec3 DAC = cross(DC, DA);

    // Triangle faces: ABD, DCA, DBC
    if (simplex4check(DCB, Do, A, simplex, dir)) return false;
    if (simplex4check(DBA, Do, C, simplex, dir)) return false;
    if (simplex4check(DAC, Do, B, simplex, dir)) return false;

    return true;
}