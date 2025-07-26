#include "collision.h"

std::pair<vec3, vec3> barycentric(Polytope* polytope, Rigid* bodyA, Rigid* bodyB) {
    // project origin onto triangle
    const Face& face = polytope->front();

    const SupportPoint& sp0 = *face.sps[0];
    const SupportPoint& sp1 = *face.sps[1];
    const SupportPoint& sp2 = *face.sps[2];

    vec3 v0 = sp1.mink - sp0.mink;
    vec3 v1 = sp2.mink - sp0.mink;
    vec3 v2 = -sp0.mink;

    // compute barycentric coordinates for projection
    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1 - v - w;

    vec3 barycentric = glm::clamp(vec3(u, v, w), 0.0f, 1.0f);

    // interpolate points and bodyA and bodyB
    vec3 PA = u * transform(sp0.indexA, bodyA) + v * transform(sp1.indexA, bodyA) + w * transform(sp2.indexA, bodyA);
    vec3 PB = u * transform(sp0.indexB, bodyB) + v * transform(sp1.indexB, bodyB) + w * transform(sp2.indexB, bodyB);

    // find midpoint
    return { PA, PB };
}