#include "collision.h"

// helper function
vec3 projectPointToPlane(const vec3& point, const vec3& normal, const vec3& planePoint) {
    vec3 diff = point - planePoint;
    float d = glm::dot(normal, diff);
    vec3 proj = d * normal;
    return point - proj;
}

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
    // assumes non-digenerate triangle
    vec3 n = glm::normalize(glm::cross(b - a, c - a));
    vec3 proj = projectPointToPlane(p, n, a);
    std::array<float, 3> bcs = barycentricsOnTriangle(a, b, c, proj);
    return bcs[0] * a + bcs[1] * b + bcs[2] * c;
}

void fallbackContact(std::vector<vec3>& rAs, std::vector<vec3>& rBs, Polytope* polytope, Rigid* bodyA, Rigid* bodyB) {
    // vec renaming
    const vec3& a = polytope->front().sps[0]->mink;
    const vec3& b = polytope->front().sps[1]->mink;
    const vec3& c = polytope->front().sps[2]->mink;

    // find closest points on triangle
    vec3 p = projectPointToPlane(vec3(), polytope->front().normal, a);
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

affine getAffine(const std::array<const SupportPoint*, 3>& sps, bool isA) {
    // rename data
    int sp0 = isA ? sps[0]->indexA : sps[0]->indexB;
    int sp1 = isA ? sps[1]->indexA : sps[1]->indexB;
    int sp2 = isA ? sps[2]->indexA : sps[2]->indexB;
    affine a;

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

    if (glm::length2(glm::cross(v01, v02)) < 1e-8f) { 
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

vec3 closestPointOnPlane(const vec3& a, const vec3& b, const vec3& c, const vec3& p) {
    vec3 n = glm::normalize(glm::cross(b - a, c - a));
    float t = glm::dot(n, p - a); // signed distance from p to plane
    return p - t * n;
}


std::pair<vec3, vec3> closestPointBetweenSegments(const vec3& p0, const vec3& p1,
                                                  const vec3& q0, const vec3& q1) {
    vec3 d1 = p1 - p0; // Direction of segment 1
    vec3 d2 = q1 - q0; // Direction of segment 2
    vec3 r  = p0 - q0;

    float a = glm::dot(d1, d1); // squared length of segment 1
    float e = glm::dot(d2, d2); // squared length of segment 2
    float f = glm::dot(d2, r);

    // Handle degenerate segments
    if (a <= 1e-8f && e <= 1e-8f) {
        // Both segments are points
        return {p0, q0};
    }
    if (a <= 1e-8f) {
        // First segment is a point
        float t = glm::clamp(f / e, 0.0f, 1.0f);
        return {p0, q0 + d2 * t};
    }
    if (e <= 1e-8f) {
        // Second segment is a point
        float s = glm::clamp(-glm::dot(d1, r) / a, 0.0f, 1.0f);
        return {p0 + d1 * s, q0};
    }

    float b = glm::dot(d1, d2);
    float c = glm::dot(d1, r);

    float denom = a*e - b*b;
    float s, t;

    // If not parallel, compute closest point on infinite lines
    if (denom != 0.0f) {
        s = glm::clamp((b*f - c*e) / denom, 0.0f, 1.0f);
    } else {
        // Parallel segments: pick s = 0 and clamp t later
        s = 0.0f;
    }

    t = (b*s + f) / e;

    // Clamp t to segment
    if (t < 0.0f) {
        t = 0.0f;
        s = glm::clamp(-c / a, 0.0f, 1.0f);
    } else if (t > 1.0f) {
        t = 1.0f;
        s = glm::clamp((b - c) / a, 0.0f, 1.0f);
    }

    vec3 c1 = p0 + d1 * s;
    vec3 c2 = q0 + d2 * t;

    return { c1, c2 };
}


// these need to be projected to 2d
void get2dBasis(vec3& u, vec3& v, const vec3& a, const vec3& b, const vec3& c) {
    // assume non-collinear
    vec3 normal = glm::normalize(glm::cross(b - a, c - a));
    u = glm::normalize(b - a);
    v = glm::normalize(glm::cross(normal, u));
}

vec2 project2d(const vec3& u, const vec3& v, const vec3& a, const vec3& p) {
    vec3 delta = p - a;
    return { glm::dot(delta, u), glm::dot(delta, v) };
}

vec3 project3d(const vec3& u, const vec3& v, const vec3& a, const vec2& p) {
    return a + p.x * u + p.y * v;
}

float cross(const vec2& r, const vec2& s) {
    return r.x * s.y - r.y * s.x;
}

bool isInsideEdge2d(const vec2& c0, const vec2& c1, const vec2& p) {
    return cross(c1 - c0, p - c0) >= 0;
}

void orderTriangle2d(std::vector<vec2>& vecs) {
    float signedArea = 0;
    for (int i = 0; i < vecs.size(); i++) {
        vec2 p0 = vecs[i];
        vec2 p1 = vecs[(i + 1) % vecs.size()];
        signedArea += (p0.x * p1.y - p1.x * p0.y);
    }
    if (signedArea < 0) {
        // Clockwise → reverse order
        std::reverse(vecs.begin(), vecs.end());
    }
}

std::optional<vec2> segmentIntersect2d(const vec2& p0, const vec2& p1, const vec2& q0, const vec2& q1) {
    vec2 r = p1 - p0;
    vec2 s = q1 - q0;

    float denom = cross(r, s);
    if (denom == 0) return std::nullopt;

    float t = cross(q0 - p0, s) / denom;
    float u = cross(q0 - p0, r) / denom;

    if (0 <= t && t <= 1 && 0 <= u && u <= 1) return p0 + t * r;
    return std::nullopt;
}

bool pointIsInTriangle2d(const vec2& p, const vec2& a, const vec2& b, const vec2& c) {
    // assume non degenerate triangle
    vec2 v0 = c - a;
    vec2 v1 = b - a;
    vec2 v2 = p - a;

    float dot00 = glm::dot(v0, v0);
    float dot01 = glm::dot(v0, v1);
    float dot02 = glm::dot(v0, v2);
    float dot11 = glm::dot(v1, v1);
    float dot12 = glm::dot(v1, v2);

    float denom = dot00 * dot11 - dot01 * dot01;

    // debug just in case
    if (denom == 0) throw std::runtime_error("pointIsInTriangle2d denom = 0");

    float u = (dot11 * dot02 - dot01 * dot12) / denom;
    float v = (dot00 * dot12 - dot01 * dot02) / denom;

    return u >= 0 && v >= 0 && u + v <= 1;
}

// line segment to triangle
void closestPointToFace(std::vector<vec3>& pts, const vec3& v0, const vec3& v1, const vec3& a, const vec3& b, const vec3& c) {
    vec3 u, v;
    get2dBasis(u, v, a, b, c);

    vec3 normal = glm::normalize(glm::cross(b - a, c - a));

    // project endpoints onto plane
    vec3 p0 = projectPointToPlane(v0, normal, a);
    vec3 p1 = projectPointToPlane(v1, normal, a);

    // convert points to 2d
    vec2 l0 = project2d(u, v, a, p0);
    vec2 l1 = project2d(u, v, a, p1);
    vec2 t0 = project2d(u, v, a, a);
    vec2 t1 = project2d(u, v, a, b);
    vec2 t2 = project2d(u, v, a, c);

    // perform line - triangle intersection 2d
    bool l0IsInside = pointIsInTriangle2d(l0, t0, t1, t2);
    bool l1IsInside = pointIsInTriangle2d(l1, t0, t1, t2);

    if (l0IsInside) pts.push_back(project3d(u, v, a, l0));
    if (l1IsInside) pts.push_back(project3d(u, v, a, l1));

    if (l0IsInside && l1IsInside) return;

    // check all edges
    std::optional<vec2> intersect;

    intersect = segmentIntersect2d(l0, l1, t0, t1);
    if (intersect.has_value()) pts.push_back(project3d(u, v, a, intersect.value()));

    intersect = segmentIntersect2d(l0, l1, t1, t2);
    if (intersect.has_value()) pts.push_back(project3d(u, v, a, intersect.value()));

    intersect = segmentIntersect2d(l0, l1, t2, t0);
    if (intersect.has_value()) pts.push_back(project3d(u, v, a, intersect.value()));
}

// check for no solution
void clipFace(std::vector<vec3>& pts, const vec3& a0, const vec3& b0, const vec3& c0, const vec3& a1, const vec3& b1, const vec3& c1) {
    vec3 u, v;
    get2dBasis(u, v, a1, b1, c1);

    vec3 normal = glm::normalize(glm::cross(b1 - a1, c1 - a1));

    // project endpoints onto plane
    vec3 p0 = projectPointToPlane(a0, normal, a1);
    vec3 p1 = projectPointToPlane(b0, normal, a1);
    vec3 p2 = projectPointToPlane(c0, normal, a1);

    // convert points to 2d
    std::vector<vec2> clipper = { project2d(u, v, a1, p0), project2d(u, v, a1, p1), project2d(u, v, a1, p2) };
    std::vector<vec2> subject = { project2d(u, v, a1, a1), project2d(u, v, a1, b1), project2d(u, v, a1, c1) };

    // order edges
    orderTriangle2d(clipper);
    orderTriangle2d(subject);

    // perform triangle - triangle intersection
    // sutherland-hodgeman
    std::vector<vec2> output = { subject[0], subject[1], subject[2] };
    std::vector<vec2> input;
    for (int c = 0; c < 3; c++) {
        input = output;
        output.clear();

        const vec2& e0 = clipper[c];
        const vec2& e1 = clipper[(c + 1) % 3];

        std::optional<vec2> tv;

        for (int i = 0; i < input.size(); i++) {
            const vec2& s = input[i];
            const vec2& p = input[(i + 1) % input.size()];

            if (isInsideEdge2d(e0, e1, p)) {
                if (!isInsideEdge2d(e0, e1, s)) {
                    tv = segmentIntersect2d(e0, e1, s, p);
                    if (tv.has_value()) output.push_back(tv.value());
                }
                output.push_back(p);
            } else if (isInsideEdge2d(e0, e1, s)) {
                tv = segmentIntersect2d(e0, e1, s, p);
                if (tv.has_value()) output.push_back(tv.value());
            }
        }
    }

    // convert intersections / inertior points back to 3d
    for (const vec2& o : output) pts.push_back(project3d(u, v, a1, o));
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

    // check vertex - vertex
    if (affA.dim == 0 && affB.dim == 0) {
        rAs.push_back(a0);
        rBs.push_back(b0);
        return 1;
    }

    // check vertex - edge (b has at least 2 unique vertices)
    if (affA.dim == 0 && affB.dim == 1) {
        rAs.push_back(a0);
        rBs.push_back(closestPointOnLine(affB.u0 ? b0 : b1, b2, a0));
        return 2;
    }
    if (affA.dim == 1 && affB.dim == 0) {
        rAs.push_back(closestPointOnLine(affA.u0 ? a0 : a1, a2, b0));
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
        closestPointToFace(pts, affA.u0 ? a0 : a1, a2, b0, b1, b2);
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
        closestPointToFace(pts, affB.u0 ? b0 : b1, b2, a0, a1, a2);
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