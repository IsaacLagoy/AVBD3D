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

// 6 test cases to watch for
// face to face (2, 2)
// edge to edge (1, 1)
// edge to face (1, 2)
// vertex to face (0, 2)
// vertex to edge (0, 1)
// vertex to vertex (0, 0)

struct affline {
    int dim = 0;
    bool u0 = false;
    bool u1 = false;
    bool u2 = false;

    affline() = default;
};

affline getAffine(const std::array<const SupportPoint*, 3>& sps, bool isA) {
    // rename data
    int sp0 = isA ? sps[0]->indexA : sps[0]->indexB;
    int sp1 = isA ? sps[1]->indexA : sps[1]->indexB;
    int sp2 = isA ? sps[2]->indexA : sps[2]->indexB;
    affline a;

    // find uniqueness
    a.u0 = sp0 != sp1 && sp0 != sp2;
    a.u1 = sp1 != sp0 && sp1 != sp2;
    a.u2 = sp2 != sp1 && sp2 != sp0;

    int numU = a.u0 + a.u1 + a.u2;

    // check for identical points
    if (numU == 0) { 
        a.dim = 0; 
        return a; 
    }

    // find model space locations of vertices
    const vec3& v0 = Mesh::uniqueVerts[sp0];
    const vec3& v1 = Mesh::uniqueVerts[sp1];
    const vec3& v2 = Mesh::uniqueVerts[sp2];

    // check for collinear vertices
    vec3 v01 = v1 - v0;
    vec3 v02 = v2 - v0;

    if (glm::length2(glm::cross(v01, v02)) < 1e-10f) { 
        a.dim = 1; 
        return a; 
    }
    
    // assumed to be coplanar non-colinear TODO check this
    a.dim = 2;
    return a;
}

vec3 closestPointOnLine(const vec3& u0, const vec3& u1, const vec3& v) {
    vec3 ab = u1 - u0;
    float t = glm::dot(v - u0, ab) / glm::dot(ab, ab);
    t = glm::clamp(t, 0.0f, 1.0f);
    return u0 + t * ab;
}

// may need to be barycentric
vec3 closestPointOnFace(const vec3& a, const vec3& b, const vec3& c, const vec3& p) {
    // shouldn't be collinear
    vec3 n = glm::cross(b - a, c - a);
    float t = (glm::dot(n, p) - glm::length(n)) / glm::dot(n, n);
    return p - t * n;
}

std::pair<vec3, vec3> closestPointBetweenLines(const vec3& u0, const vec3& u1, const vec3& v0, const vec3& v1) {
    // we are assuming that lines are not degenerate
    vec3 d1 = u1 - u0;
    vec3 d2 = v1 - v0;
    vec3 r  = u0 - v0;
    float a = glm::dot(d1, d1);
    float e = glm::dot(d2, d2);
    float f = glm::dot(d2, r);

    // general nondegenerate case
    float c = glm::dot(d1, r);
    float b = glm::dot(d1, d2);
    float denom = a * e - b * b;

    // if segments are not parallel, compute the closest point on L1 to L2 and clamp to segment S1. Else pick arbitrary s (here 0)
    float s;
    if (denom != 0) s = glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
    else s = 0.0f;

    // copmute point on L2 closest to S1(s)
    float t = (b * s + f) / e;

    if (t < 0.0f) {
        t = 0.0f;
        s = glm::clamp(-c / a, 0.0f, 1.0f);
    } else if (t > 1.0f) {
        t = 1.0f;
        s = glm::clamp((b - c) / a, 0.0f, 1.0f);
    }

    vec3 c1 = u0 + d1 * s;
    vec3 c2 = v0 + d2 * t;
    return { c1, c2 };
}

// these need to be projected to 2d

// line segment to triangle
void closestPointToFace(std::vector<vec3>& pts, const vec3& v0, const vec3& v1, const vec3& a, const vec3& b, const vec3& c) {
    
}

// check for no solution
void clipFace(std::vector<vec3>& pts, const vec3& a0, const vec3& b0, const vec3& c0, const vec3& a1, const vec3& b1, const vec3& c1) {

}

vec3 avgVecs(const std::vector<vec3>& pts) {
    if (pts.size() == 0) return vec3();

    vec3 v = vec3();
    for (const vec3& p : pts) v += p;
    return v / (float) pts.size();
}

std::pair<vec3, vec3> getContact(Polytope* polytope, Rigid* bodyA, Rigid* bodyB) {
    // determine affine relationships
    affline affA = getAffine(polytope->front().sps, true);
    affline affB = getAffine(polytope->front().sps, false);

    // rename data
    const Face& face = polytope->front();

    const SupportPoint& sp0 = *face.sps[0];
    const SupportPoint& sp1 = *face.sps[1];
    const SupportPoint& sp2 = *face.sps[2];

    const vec3& a0 = Mesh::uniqueVerts[sp0.indexA];
    const vec3& a1 = Mesh::uniqueVerts[sp1.indexA];
    const vec3& a2 = Mesh::uniqueVerts[sp2.indexA];
    
    const vec3& b0 = Mesh::uniqueVerts[sp0.indexB];
    const vec3& b1 = Mesh::uniqueVerts[sp1.indexB];
    const vec3& b2 = Mesh::uniqueVerts[sp2.indexB];

    // check vertex - vertex
    if (affA.dim == 0 && affB.dim == 0) return { a0, b0 };

    // check vertex - edge (b has at least 2 unique vertices)
    if (affA.dim == 0 && affB.dim == 1) return { a0, closestPointOnLine(affB.u0 ? b0 : b1, b2, a0) };
    if (affA.dim == 1 && affB.dim == 0) return { closestPointOnLine(affB.u0 ? a0 : a1, a2, b0), b0 };

    // check vertex - face
    if (affA.dim == 0 && affB.dim == 2) return { a0, closestPointOnFace(b0, b1, b2, a0) };
    if (affA.dim == 2 && affB.dim == 0) return { closestPointOnFace(a0, a1, a2, b0), b0 };

    // check edge - edge
    if (affA.dim == 1 && affB.dim == 1) return closestPointBetweenLines(affB.u0 ? a0 : a1, a2, affB.u0 ? b0 : b1, b2);

    std::vector<vec3> pts;

    // check edge - face
    if (affA.dim == 1 && affB.dim == 2) {
        closestPointToFace(pts, affA.u0 ? a0 : a1, a2, b0, b1, b2);
        vec3 p = avgVecs(pts);
        return { p, closestPointOnFace(b0, b1, b2, p) };
    };
    if (affA.dim == 2 && affB.dim == 1) {
        closestPointToFace(pts, affB.u0 ? b0 : b1, b2, a0, a1, a2);
        vec3 p = avgVecs(pts);
        return { closestPointOnFace(a0, a1, a2, p), p };
    };

    // check face - face
    clipFace(pts, a0, a1, a2, b0, b1, b2);

    // fallback to barycentric
    if (pts.size() == 0) return barycentric(polytope, bodyA, bodyB);

    vec3 p = avgVecs(pts);
    return { p, closestPointOnFace(b0, b1, b2, p) };
}