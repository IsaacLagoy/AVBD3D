#include "collision.h"

// helper function
vec3 projectPointToPlane(const vec3& point, const vec3& normal, const vec3& planePoint) {
    vec3 diff = point - planePoint;
    float d = glm::dot(normal, diff);
    vec3 proj = d * normal;
    return point - proj;
}

std::pair<vec3, vec3> barycentric(Polytope* polytope, Rigid* bodyA, Rigid* bodyB) {
    // project origin onto triangle
    const Face& face = polytope->front();

    const SupportPoint& sp0 = *face.sps[0];
    const SupportPoint& sp1 = *face.sps[1];
    const SupportPoint& sp2 = *face.sps[2];

    vec3 proj = projectPointToPlane(vec3(), face.normal, sp0.mink);

    vec3 v0 = sp1.mink - sp0.mink;
    vec3 v1 = sp2.mink - sp0.mink;
    vec3 v2 = proj     - sp0.mink;

    // compute barycentric coordinates for projection
    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;

    float alpha, beta, gamma;

    // will happen if triangle is nearly colinear
    if (fabs(denom) == 0) {
        // fallback, treat triangle as line by using its longest edge
        vec3 v3 = sp2.mink - sp1.mink;
        
        float l10 = glm::length2(v0);
        float l20 = glm::length2(v1);
        float l21 = glm::length2(v2);

        vec3 a, b;

        if      (l10 > l20 && l10 > l21) { a = sp0.mink; b = sp1.mink; }
        else if (l20 > l21)              { a = sp0.mink; b = sp2.mink; } 
        else                             { a = sp1.mink; b = sp2.mink; } 

        vec3 e = b - a;
        float t = glm::dot(proj - a, e) / glm::dot(e, e);
        t = glm::clamp(t, 0.0f, 1.0f);

        alpha = 1.0f - t; 
        beta = t; 
        gamma = 0.0f;
    } else {
        beta = (d11 * d20 - d01 * d21) / denom;
        gamma = (d00 * d21 - d01 * d20) / denom;
        alpha = 1.0f - beta - gamma;
    }

    

    // interpolate points and bodyA and bodyB
    vec3 PA = alpha * Mesh::uniqueVerts[sp0.indexA] + beta * Mesh::uniqueVerts[sp1.indexA] + gamma * Mesh::uniqueVerts[sp2.indexA];
    vec3 PB = alpha * Mesh::uniqueVerts[sp0.indexB] + beta * Mesh::uniqueVerts[sp1.indexB] + gamma *Mesh::uniqueVerts[sp2.indexB];

    return { PA, PB };
}