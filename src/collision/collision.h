#ifndef COLLISION_H
#define COLLISION_H

#include "solver.h"
#include "rigid.h"
#include "util/unorderedArray.h"
#include "mesh.h"
#include "linalg/linalg.h"
#include <cmath>
#include <optional>

#define DEBUG_PRINT_GJK false

// simplex
using Simplex = UnorderedArray<SupportPoint, 4>;
enum Index { A, B, C, D };

struct Compare {
    // Compare FacePtrs
    bool operator()(const Face& a, const Face& b) const {
        return a.distance < b.distance;
    }
};

// polytope
struct Polytope {
    std::unordered_map<SupportPoint, std::shared_ptr<SupportPoint>, SupportPointHash, SupportPointEqual> sps;
    std::set<Face, Compare> pq; // Min-heap based on face distance to origin
    vec3 vertTot; // used for tracking centroid when origin fails

    Polytope(const Simplex& simplex);
    ~Polytope();
    const SupportPoint* add(const SupportPoint& sp);
    void add(Face face);
    std::optional<Face> buildFace(const SupportPoint* pa, const SupportPoint* pb, const SupportPoint* pc, bool force=false);
    void erase(const Face& toErase);
    void erase(const std::vector<Face>& toErase);
    bool insert(const SupportPoint& spRef);
    const Face& front() const;
};

// function declarations
bool handleSimplex(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex0(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex1(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex2(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex3(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex4(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);

bool gjk(Rigid* bodyA, Rigid* bodyB, Simplex& simplex);
bool epa(Rigid* bodyA, Rigid* bodyB, Polytope* polytope);

int getContact(std::vector<vec3>& rAs, std::vector<vec3>& rBs, Polytope* polytope, Rigid* bodyA, Rigid* bodyB);
vec3 projectPointOntoPlane(const vec3& point, const vec3& normal, const vec3& planePoint);
vec3 closestPointOnSegmentToVertex(const vec3& u0, const vec3& u1, const vec3& v);
std::pair<vec3, vec3> closestPointBetweenSegments(const vec3& p0, const vec3& p1, const vec3& q0, const vec3& q1);
void closestPointsOnTriangleToSegment(std::vector<vec3>& pts, const vec3& v0, const vec3& v1, const vec3& a, const vec3& b, const vec3& c);
void clipFace(std::vector<vec3>& pts, const vec3& a0, const vec3& b0, const vec3& c0, const vec3& a1, const vec3& b1, const vec3& c1);

#endif