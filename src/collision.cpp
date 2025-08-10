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
    vec3 inv = glm::inverse(body->rotation) * dir;
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

float projectedDistance(const vec3& normal, const vec3& point) {
    // Assumes `normal` is already normalized
    return glm::dot(normal, point);
}

bool sameDirection(const vec3& v1, const vec3& v2) {
    return glm::dot(v1, v2) > 0;
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
    if (DEBUG_PRINT_GJK) print(bodyA->position);
    if (DEBUG_PRINT_GJK) print(polytope->front().normal);

    // compute contact information
    contacts[0].normal = -polytope->front().normal; // TODO ensure 
    contacts[0].depth = polytope->front().distance;

    std::pair<vec3, vec3> rs = barycentric(polytope, bodyA, bodyB);

    contacts[0].rA = rs.first;
    contacts[0].rB = rs.second;

    delete polytope;
    return 1;
}

// GJK
bool gjk(Rigid* bodyA, Rigid* bodyB, Simplex& simplex) {
    vec3 dir;
    for (unsigned short i = 0; i < 20; i++) {

        if (DEBUG_PRINT_GJK) print("Iteration");
        if (DEBUG_PRINT_GJK) print(i);
        if (DEBUG_PRINT_GJK) print((int) simplex.size());
        if (DEBUG_PRINT_GJK) print("dir before");
        if (DEBUG_PRINT_GJK) print(dir);

        // defines direction and determins if collision has happened
        bool detected = handleSimplex(simplex, bodyA, bodyB, dir); 

        if (DEBUG_PRINT_GJK) print("dir after");
        if (DEBUG_PRINT_GJK) print(dir);

        // return early if collision is found
        if (detected) return true;
        // add a new point to simplex
        simplex.add(getSupportPoint(bodyA, bodyB, dir));

        if (DEBUG_PRINT_GJK) print("mink point added:");
        if (DEBUG_PRINT_GJK) print(simplex[simplex.size() - 1].mink);
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
    dir = bodyB->position - bodyA->position;
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

// EPA
Polytope::Polytope(const Simplex& simplex) : sps(), pq(), vertTot(0) {
    // copy vertices from the simplex (Do not change to move, they will need to be saved in more efficient versions)
    for (int i = 0; i < 4; i++) add(simplex[i]);

    // capture iterators or pointers for stable access
    std::vector<const SupportPoint*> pts;
    pts.reserve(4);  // avoid reallocations
    for (const auto& [key, sptr] : sps) {
        pts.push_back(sptr.get());
        if (pts.size() == 4) break;
    }

    // add faces with correct combinations
    add(buildFace(pts[0], pts[1], pts[2]).value()); 
    add(buildFace(pts[0], pts[1], pts[3]).value()); 
    add(buildFace(pts[0], pts[2], pts[3]).value()); 
    add(buildFace(pts[1], pts[2], pts[3]).value()); 
}

Polytope::~Polytope() {}

// add support points and faces to the polytope
const SupportPoint* Polytope::add(const SupportPoint& sp) {
    vertTot += sp.mink;

    // Check if SupportPoint already exists
    auto it = sps.find(sp);
    if (it != sps.end()) {
        // Already present, return stored pointer
        return it->second.get();
    }

    // Not present, create new shared_ptr and insert
    auto spPtr = std::make_shared<SupportPoint>(sp);
    auto [insertedIt, inserted] = sps.emplace(*spPtr, spPtr);

    return insertedIt->second.get();
}

void Polytope::add(Face face) { pq.insert(face); }

// create new faces using existing points
std::optional<Face> Polytope::buildFace(const SupportPoint* pa, const SupportPoint* pb, const SupportPoint* pc) {

    const vec3& av = pa->mink;
    const vec3& bv = pb->mink;
    const vec3& cv = pc->mink;

    if (hasNaN(av)) throw std::runtime_error("av has nan");
    if (hasNaN(bv)) throw std::runtime_error("bv has nan");
    if (hasNaN(cv)) throw std::runtime_error("cv has nan");

    Face face = Face();

    // find normal and distance from plane to origin
    face.normal = glm::cross(bv - av, cv - av);
    if (glm::length2(face.normal) < 1e-8f) {
        return std::nullopt; // face is degenerate
    } else {
        face.normal = glm::normalize(face.normal);
    }

    // signed distance from origin to plane
    face.distance = projectedDistance(face.normal, av); // assumes plane passes through av
    
    // initialize winding order
    face.sps = { pa, pb, pc };

    // test if face is coplanar with the origin. if so, use the polytope center instead of origin to determine normal direction
    vec3 midpoint = vec3(0);
    if (std::abs(glm::dot(face.normal, av)) < 1e-6f) midpoint = vertTot / (float) sps.size();

    // check winding order
    if (!sameDirection(face.normal, av - midpoint)) {
        face.normal *= -1;
        face.sps = { pa, pc, pb }; // ensures vertices are ordered to face normal outward.
    }

    return face;
}

void Polytope::erase(const Face& toErase) { 
    auto it = pq.find(toErase); // fast O(log n) lookup using heterogeneous comparator
    if (it != pq.end()) pq.erase(it); // fast O(log n) erase
}
void Polytope::erase(const std::vector<Face>& toErase) { 
    for (const Face& face : toErase) erase(face);
}

// inserts a new support point into the polytope
bool Polytope::insert(const SupportPoint& spRef) {
    
    // check if point is already in cloud or if it is closer than the face's centroid
    auto it = sps.find(spRef);
    if (it != sps.end() || projectedDistance(front().normal, spRef.mink) - front().distance < 1e-6f) return true;

    // insert support point into the sps cloud
    const SupportPoint* sp = add(spRef);

    Edge edge;
    std::set<Edge> edges;
    // loop through every face
    for (auto it = pq.begin(); it != pq.end();) {
        const Face& face = *it;

        // if face is not facing the new point, continue
        if (!sameDirection(face.normal, sp->mink - face.sps[0]->mink)) {
            ++it;
            continue;
        }

        // add the face's edges to the horizon edges
        for (int i = 0; i < 3; i++) {
            face.overrideEdge(i, edge);

            // if reversed edge appears, remove it. We can do this if we have consistent winding order
            auto edgeIt = edges.find({ edge.second, edge.first });
            if (edgeIt != edges.end()) {
                edges.erase(edgeIt);
                continue;
            }
            edges.insert(edge);
        }

        // erase returns the next iterator safely
        it = pq.erase(it);
    }

    // add new faces from edges
    for (Edge edge : edges) {
        std::optional<Face> faceOpt = buildFace(edge.first, edge.second, sp);
        if (!faceOpt.has_value()) continue; // skip degenerate face
        add(faceOpt.value());
    }

    return false;
}

const Face& Polytope::front() const { return *pq.begin(); }

bool epa(Rigid* bodyA, Rigid* bodyB, Polytope* polytope) {
    bool done = false;

    while (!done) {
        SupportPoint sp = getSupportPoint(bodyA, bodyB, polytope->front().normal);
        done = polytope->insert(sp);
    }

    return false;
}