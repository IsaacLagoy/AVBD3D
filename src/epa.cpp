#include "collision.h"

float projectedDistance(const vec3& normal, const vec3& point) {
    // Assumes `normal` is already normalized
    return glm::dot(normal, point);
}

bool sameDirection(const vec3& v1, const vec3& v2) {
    return glm::dot(v1, v2) > 0;
}

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