#include "collision.h"

// helper function

std::array<float, 3> barycentricsOnTriangle(const vec3& a, const vec3& b, const vec3& c, const vec3& p) {
    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 bc = c - b;

    // copmute parametric position s for projection p' of p on ab
    float snom = glm::dot(p - a, ab);
    float sdenom = glm::dot(p - b, a - b);

    // copmute parametric position t for projection p' of p on ac
    float tnom = glm::dot(p - a, ac);
    float tdenom = glm::dot(p - c, a - c);

    // vertex region early out
    if (snom <= 0.0f && tnom <= 0.0f) return { 1, 0, 0 };

    // compute parametric position u for projection p' of p on bc
    float unom = glm::dot(p - b, bc);
    float udenom = glm::dot(p - c, b - c);

    // other two vertex early outs
    if (sdenom <= 0.0f && unom <= 0.0f) return { 0, 1, 0 };
    if (tdenom <= 0.0f && udenom <= 0.0f) return { 0, 0, 1 };

    // p is outside (or on) ab if the triple scalar product [n pa pb] <= 0
    vec3 n = glm::cross(b - a, c - a);
    float vc = glm::dot(n, glm::cross(a - p, b - p));

    // if p outside ab and within feature region of ab
    if (vc <= 0.0f && snom >= 0.0f && sdenom >= 0.0f) {
        float t = snom / (snom + sdenom);
        return { 1 - t, t, 0 };
    }

    // p is outside (or on) bc if the triple scalar product  [n pb pc] <= 0
    float va = glm::dot(n, glm::cross(b - p, c - p));

    // if p outside bc and within feature region of bc
    if (va <= 0.0f && unom >= 0.0f && udenom >= 0.0f) {
        float t = unom / (unom + udenom);
        return { 0, 1 - t, t };
    }

    // p is outside (or on) ca if the triple scalar product [n pc pa] <= 0
    float vb = glm::dot(n, glm::cross(c - p, a - p));

    // if p outside ca and within feature region of ca
    if (vb <= 0.0f && tnom >= 0.0f && tdenom >= 0.0f) {
        float t =  tnom / (tnom + tdenom);
        return { t, 0, 1 - t };
    }

    // p must be inside face region.
    float u = va / (va + vb + vc);
    float v = vb / (va + vb + vc);
    float w = 1.0f - u - v;

    return { u, v, w };
}

vec3 closestPointOnTriangle(const vec3& a, const vec3& b, const vec3& c, const vec3& p) {
    // Check vertex regions
    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 ap = p - a;

    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a; // vertex region A

    vec3 bp = p - b;
    float d3 = glm::dot(ab, bp);
    float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b; // vertex B

    vec3 cp = p - c;
    float d5 = glm::dot(ab, cp);
    float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c; // vertex C

    // Check edge regions
    float vc = d1*d4 - d3*d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + v * ab; // edge AB
    }

    float vb = d5*d2 - d1*d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + w * ac; // edge AC
    }

    float va = d3*d6 - d5*d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b); // edge BC
    }

    // Inside face region
    float denom = 1.0f / (va + vb + vc); // sum of barycentric weights
    float v = vb * denom;
    float w = vc * denom;
    return a + ab*v + ac*w;
}


void fallbackContact(std::vector<vec3>& rAs, std::vector<vec3>& rBs, Polytope* polytope, Rigid* bodyA, Rigid* bodyB) {
    // vec renaming
    const vec3& a = polytope->front().sps[0]->mink;
    const vec3& b = polytope->front().sps[1]->mink;
    const vec3& c = polytope->front().sps[2]->mink;

    // find closest points on triangle
    vec3 p = projectPointOntoPlane(vec3(), polytope->front().normal, a);
    std::array<float, 3> bcs = barycentricsOnTriangle(a, b, c, p);

    // project each point
    vec3 rA = vec3();
    vec3 rB = vec3();

    for (int i = 0; i < 3; i++) {
        rA += bcs[i] * transform(polytope->front().sps[i]->indexA, bodyA);
        rB += bcs[i] * transform(polytope->front().sps[i]->indexB, bodyB);
    }

    rAs.push_back(rA);
    rBs.push_back(rB);
}

// 6 test cases to watch for
// face to face (2, 2)
// edge to edge (1, 1)
// edge to face (1, 2)
// vertex to face (0, 2)
// vertex to edge (0, 1)
// vertex to vertex (0, 0)

struct affine {
    int dim = 0;
    bool u0 = false;
    bool u1 = false;
    bool u2 = false;

    affine() = default;
};

// should only be 0, 1, or 3 unique

affine getAffine(const std::array<const SupportPoint*, 3>& sps, bool isA) {
    const float EPSILON = 1e-8f;

    // rename data
    int sp0 = isA ? sps[0]->indexA : sps[0]->indexB;
    int sp1 = isA ? sps[1]->indexA : sps[1]->indexB;
    int sp2 = isA ? sps[2]->indexA : sps[2]->indexB;
    affine a;

    // find model space locations of vertices
    const vec3& v0 = Mesh::uniqueVerts[sp0];
    const vec3& v1 = Mesh::uniqueVerts[sp1];
    const vec3& v2 = Mesh::uniqueVerts[sp2];

    // find uniqueness
    bool e01 = glm::length2(v0 - v1) < EPSILON;
    bool e12 = glm::length2(v1 - v2) < EPSILON;
    bool e20 = glm::length2(v2 - v0) < EPSILON;

    a.u0 = !e01 && !e20;
    a.u1 = !e01 && !e12;
    a.u2 = !e12 && !e20;

    int numU = a.u0 + a.u1 + a.u2;

    // check for identical points
    if (numU == 0) { 
        a.dim = 0; 
        return a; 
    }

    if (numU <= 2) {
        a.dim = 1;
        return a;
    }

    // check for collinear vertices
    vec3 v01 = v1 - v0;
    vec3 v02 = v2 - v0;
    vec2 cross = glm::cross(v01, v02);
    float mag2 = glm::length2(cross);
    float l01 = glm::length2(v01);
    float l02 = glm::length2(v02);
    float epsilon = std::max(l01, l02) * EPSILON;

    if (mag2 < epsilon) { 
        a.dim = 1; 
        return a; 
    }
    
    // assumed to be coplanar non-colinear TODO check this
    a.dim = 2;
    return a;
}

vec3 avgVecs(const std::vector<vec3>& pts) {
    if (pts.size() == 0) return vec3();

    vec3 v = vec3();
    for (const vec3& p : pts) v += p;
    return v / (float) pts.size();
}

int getContact(std::vector<vec3>& rAs, std::vector<vec3>& rBs, Polytope* polytope, Rigid* bodyA, Rigid* bodyB) {
    // determine affine relationships
    affine affA = getAffine(polytope->front().sps, true);
    affine affB = getAffine(polytope->front().sps, false);

    // rename data
    const Face& face = polytope->front();

    const SupportPoint& sp0 = *face.sps[0];
    const SupportPoint& sp1 = *face.sps[1];
    const SupportPoint& sp2 = *face.sps[2];

    const vec3& a0 = transform(Mesh::uniqueVerts[sp0.indexA], bodyA);
    const vec3& a1 = transform(Mesh::uniqueVerts[sp1.indexA], bodyA);
    const vec3& a2 = transform(Mesh::uniqueVerts[sp2.indexA], bodyA);
    
    const vec3& b0 = transform(Mesh::uniqueVerts[sp0.indexB], bodyB);
    const vec3& b1 = transform(Mesh::uniqueVerts[sp1.indexB], bodyB);
    const vec3& b2 = transform(Mesh::uniqueVerts[sp2.indexB], bodyB);

    // print("Collision");
    // print(affA.dim);
    // print(a0);
    // print(a1);
    // print(a2);
    // print(affB.dim);
    // print(b0);
    // print(b1);
    // print(b2);

    // check vertex - vertex
    if (affA.dim == 0 && affB.dim == 0) {
        rAs.push_back(a0);
        rBs.push_back(b0);
        return 1;
    }

    // check vertex - edge (b has at least 2 unique vertices)
    if (affA.dim == 0 && affB.dim == 1) {
        rAs.push_back(a0);
        rBs.push_back(closestPointOnSegmentToVertex(affB.u0 ? b0 : b1, b2, a0));
        return 2;
    }
    if (affA.dim == 1 && affB.dim == 0) {
        rAs.push_back(closestPointOnSegmentToVertex(affA.u0 ? a0 : a1, a2, b0));
        rBs.push_back(b0);
        return 2; 
    }

    // check vertex - face
    if (affA.dim == 0 && affB.dim == 2) {
        rAs.push_back(a0);
        rBs.push_back(closestPointOnTriangle(b0, b1, b2, a0));
        return 3; 
    }
    if (affA.dim == 2 && affB.dim == 0) {
        rAs.push_back(closestPointOnTriangle(a0, a1, a2, b0));
        rBs.push_back(b0);
        return 3; 
    }

    // check edge - edge
    if (affA.dim == 1 && affB.dim == 1) {
        std::pair<vec3, vec3> rs = closestPointBetweenSegments(affA.u0 ? a0 : a1, a2, affB.u0 ? b0 : b1, b2); 
        rAs.push_back(rs.first);
        rBs.push_back(rs.second);
        return 4;
    }

    std::vector<vec3> pts;

    // check edge - face
    if (affA.dim == 1 && affB.dim == 2) {
        closestPointsOnTriangleToSegment(pts, affA.u0 ? a0 : a1, a2, b0, b1, b2);
        if (pts.size() == 0) {
            fallbackContact(rAs, rBs, polytope, bodyA, bodyB);
            return 7;
        }

        // add all contact points
        rAs.insert(rAs.begin(), pts.begin(), pts.end());
        for (vec3 p : pts) rBs.push_back(closestPointOnTriangle(b0, b1, b2, p));
        return 5;
    };
    if (affA.dim == 2 && affB.dim == 1) {
        closestPointsOnTriangleToSegment(pts, affB.u0 ? b0 : b1, b2, a0, a1, a2);
        if (pts.size() == 0) {
            fallbackContact(rAs, rBs, polytope, bodyA, bodyB);
            return 7;
        }

        // add all contact points
        for (vec3 p : pts) rAs.push_back(closestPointOnTriangle(a0, a1, a2, p));
        rBs.insert(rBs.begin(), pts.begin(), pts.end());
        return 5;
    };

    // check face - face
    clipFace(pts, a0, a1, a2, b0, b1, b2);

    // fallback to barycentric
    if (pts.size() == 0) {
        fallbackContact(rAs, rBs, polytope, bodyA, bodyB);
        return 7;
    }

    rAs.insert(rAs.begin(), pts.begin(), pts.end());
    for (vec3 p : pts) rBs.push_back(closestPointOnTriangle(b0, b1, b2, p));
    return 6;
}