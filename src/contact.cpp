#include "collision.h"

// helper function
vec3 projectPointToPlane(const vec3& point, const vec3& normal, const vec3& planePoint) {
    vec3 diff = point - planePoint;
    float d = glm::dot(normal, diff);
    vec3 proj = d * normal;
    return point - proj;
}

std::pair<vec3, vec3> projectbcs(const SupportPoint& sp0, const SupportPoint& sp1, const SupportPoint& sp2, const vec3& bcs) {
    // interpolate points and bodyA and bodyB in model space
    vec3 PA = bcs[0] * Mesh::uniqueVerts[sp0.indexA] + bcs[1] * Mesh::uniqueVerts[sp1.indexA] + bcs[2] * Mesh::uniqueVerts[sp2.indexA];
    vec3 PB = bcs[0] * Mesh::uniqueVerts[sp0.indexB] + bcs[1] * Mesh::uniqueVerts[sp1.indexB] + bcs[2] * Mesh::uniqueVerts[sp2.indexB];

    return { PA, PB };
}

std::pair<vec3, vec3> barycentric(Polytope* polytope, Rigid* bodyA, Rigid* bodyB) {
    // project origin onto triangle
    const Face& face = polytope->front();

    // rename data
    const SupportPoint& sp0 = *face.sps[0];
    const SupportPoint& sp1 = *face.sps[1];
    const SupportPoint& sp2 = *face.sps[2];

    const vec3& a = sp0.mink;
    const vec3& b = sp1.mink;
    const vec3& c = sp2.mink;
    const vec3 p(0.0f);

// We'll use an epsilon for robustness
const float eps = 1e-8f;

// Edges and point vectors
vec3 ab = b - a;
vec3 ac = c - a;
vec3 bc = c - b;

vec3 ap = p - a;
vec3 bp = p - b;
vec3 cp = p - c;

// Dot products for region checks
float d1 = glm::dot(ab, ap);
float d2 = glm::dot(ac, ap);
float d3 = glm::dot(ab, bp);
float d4 = glm::dot(ac, bp);
float d5 = glm::dot(ab, cp);
float d6 = glm::dot(ac, cp);

// --- Vertex regions ---
if (d1 <= eps && d2 <= eps) {
    return projectbcs(sp0, sp1, sp2, {1.0f, 0.0f, 0.0f});
}
if (d3 >= -eps && d4 <= eps) {
    return projectbcs(sp0, sp1, sp2, {0.0f, 1.0f, 0.0f});
}
if (d6 >= -eps && d5 <= eps) {
    return projectbcs(sp0, sp1, sp2, {0.0f, 0.0f, 1.0f});
}

// --- Edge AB region ---
float vc = d1 * d4 - d3 * d2;
if (vc <= eps && d1 >= -eps && d3 <= eps) {
    float denom = d1 - d3;
    float t = (fabs(denom) > eps) ? d1 / denom : 0.0f;
    t = glm::clamp(t, 0.0f, 1.0f);
    return projectbcs(sp0, sp1, sp2, {1.0f - t, t, 0.0f});
}

// --- Edge AC region ---
float vb = d5 * d2 - d1 * d6;
if (vb <= eps && d2 >= -eps && d6 <= eps) {
    float denom = d2 - d6;
    float t = (fabs(denom) > eps) ? d2 / denom : 0.0f;
    t = glm::clamp(t, 0.0f, 1.0f);
    return projectbcs(sp0, sp1, sp2, {1.0f - t, 0.0f, t});
}

// --- Edge BC region ---
float va = d3 * d6 - d5 * d4;
if (va <= eps && (d4 - d3) >= -eps && (d5 - d6) >= -eps) {
    float denom = (d4 - d3) + (d5 - d6);
    float t = (fabs(denom) > eps) ? (d4 - d3) / denom : 0.0f;
    t = glm::clamp(t, 0.0f, 1.0f);
    return projectbcs(sp0, sp1, sp2, {0.0f, 1.0f - t, t});
}

// --- Face region ---
// If none of the above, it's inside the triangle face
vec3 n = glm::normalize(glm::cross(ab, ac));
vec3 q = p - glm::dot(p - a, n) * n;

// Standard barycentric computation
vec3 v0 = ab;
vec3 v1 = ac;
vec3 v2 = q - a;

float d00 = glm::dot(v0, v0);
float d01 = glm::dot(v0, v1);
float d11 = glm::dot(v1, v1);
float d20 = glm::dot(v2, v0);
float d21 = glm::dot(v2, v1);

float denomFace = d00 * d11 - d01 * d01;
if (fabs(denomFace) < eps) {
    // Degenerate face — fallback to vertex A
    return projectbcs(sp0, sp1, sp2, {1.0f, 0.0f, 0.0f});
}

float beta  = (d11 * d20 - d01 * d21) / denomFace;
float gamma = (d00 * d21 - d01 * d20) / denomFace;
float alpha = 1.0f - beta - gamma;

return projectbcs(sp0, sp1, sp2, {alpha, beta, gamma});

}