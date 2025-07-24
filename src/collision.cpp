#include "solver.h"

// datatypes
struct SupportPoint {
    int indexA = 0;
    int indexB = 0;
    vec3 mink = vec3(); // position in Minkowski difference space

    // comparison operator for set
    bool operator<(const SupportPoint& other) const {
        if (indexA != other.indexA) return indexA < other.indexA;
        return indexB < other.indexB;
    }
};

using Simplex = UnorderedArray<SupportPoint, 4>;

enum Index { A, B, C, D };

// helper functions
vec3 transform(const vec3& vertex, Rigid* body) {
    vec4 four = vec4(vertex, 1.0f);
    return vec3(buildModelMatrix(body) * four);
}

vec3 transform(int index, Rigid* body) {
    vec3 vertex = Mesh::uniqueVerts[index];
    return transform(vertex, body);
}

int bestDot(Rigid* body, const vec3& dir) {
    // transform dir to model space
    vec3 inv = glm::inverse(body->rotation) * dir;
    return Mesh::bestDot(inv);
}

SupportPoint getSupportPoint(Rigid* bodyA, Rigid* bodyB, const vec3& dir) {
    int indexA = bestDot(bodyA, dir);
    int indexB = bestDot(bodyB, -dir);
    return { indexA, indexB, transform(indexA, bodyA) - transform(indexB, bodyB) };
}

// function declarations
bool handleSimplex(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool simplex0(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool simplex1(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool simplex2(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool simplex3(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool simplex4(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);

bool gjk(Rigid* bodyA, Rigid* bodyB, Simplex& simplex);
bool epa(Rigid* bodyA, Rigid* bodyB, const Simplex&);

// Main
int Manifold::collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts) {
    Simplex simplex = Simplex(); // can prolly go on the stack idk, there's only one rn
    bool collided = gjk(bodyA, bodyB, simplex);

    print("entering epa");

    epa(bodyA, bodyB, simplex);
    
    if (collided) {
        bodyA->color = vec4(1, 0, 0, 1);
        bodyB->color = vec4(1, 0, 0, 1);
    }

    return collided;
}

// GJK
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
    dir = bodyB->position - bodyA->position;
    return false;
}

bool simplex1(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    dir *= -1;
    return false;
}

bool simplex2(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir) {
    vec3 vecBA = simplex[A].mink - simplex[B].mink;
    vec3 vecBo = -simplex[B].mink;
    dir = glm::cross(glm::cross(vecBA, vecBo), vecBA);
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
        simplex.remove(index);
        vec3 io = -simplex[index].mink;
        dir = (glm::dot(normal, io) > 0) ? normal : -normal;
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

    vec3 DCB = cross(DC, DB);
    vec3 DBA = cross(DB, DA);
    vec3 DAC = cross(DA, DC);

    // Triangle faces: ABD, DCA, DBC
    if (simplex4check(DCB, Do, A, simplex, dir)) return false;
    if (simplex4check(DBA, Do, C, simplex, dir)) return false;
    if (simplex4check(DAC, Do, B, simplex, dir)) return false;

    return true;
}

// EPA
using Edge = std::pair<const SupportPoint*, const SupportPoint*>;

struct Face {
    std::array<const SupportPoint*, 3> sps;
    vec3 normal;
    float distance;

    bool operator==(const Face& other) const {
        return sps == other.sps;
    }

    // overrides edge reference with indexed edge from face
    void overrideEdge(int i, Edge& edge) {
        edge = { sps[i % 3], sps[(i + 1) % 3] }; 
    }
};

using FacePtr = std::unique_ptr<Face>;

struct Compare {
    using is_transparent = void; // enables heterogeneous lookup

    // Compare FacePtrs
    bool operator()(const FacePtr& a, const FacePtr& b) const {
        return a->distance > b->distance;
    }

    // Compare FacePtr with Face*
    bool operator()(const FacePtr& a, const Face* b) const {
        return a->distance > b->distance;
    }

    bool operator()(const Face* a, const FacePtr& b) const {
        return a->distance > b->distance;
    }
};


struct Polytope {
    std::set<SupportPoint> sps;

    // Min-heap based on face distance to origin
    std::set<FacePtr, Compare> pq;

    // used for tracking centroid when origin fails
    vec3 vertTot;

    Polytope(const Simplex& simplex) : sps(), pq(), vertTot(0) {
        // copy vertices from the simplex (Do not change to move, they will need to be saved in more efficient versions)
        for (int i = 0; i < 4; i++) add(simplex[i]);

        // capture iterators or pointers for stable access
        std::vector<const SupportPoint*> pts;
        for (const auto& sp : sps) {
            pts.push_back(&sp);
            if (pts.size() == 4) break;
        }

        print("vectorized faces");

        // add faces with correct combinations
        add(buildFace(pts[0], pts[1], pts[2]));
        add(buildFace(pts[0], pts[1], pts[3]));
        add(buildFace(pts[0], pts[2], pts[3]));
        add(buildFace(pts[1], pts[2], pts[3]));
    }

    ~Polytope() {
        // were using std::unique_ptr so memory is handled
    }

    // add support points and faces to the polytope
    const SupportPoint* add(SupportPoint sp) {
        vertTot += sp.mink;
        auto [it, inserted] = sps.insert(sp);
        return &(*it);
    }
    void add(FacePtr face) { pq.insert(std::move(face)); }

    // create new faces using existing points
    FacePtr buildFace(const SupportPoint* pa, const SupportPoint* pb, const SupportPoint* pc) {
        const vec3& av = pa->mink;
        const vec3& bv = pb->mink;
        const vec3& cv = pc->mink;

        print(1);

        FacePtr face = std::make_unique<Face>();

        print(2);

        vec3 centerFace = (av + bv + cv) / 3.0f;
        face->distance = glm::length2(centerFace); // distance can be squared since it is only used for positive comparisons
        face->normal = glm::cross(av - bv, av - cv);
        face->sps = { pa, pb, pc };

        print(3);

        // test if face is coplanar with the origin. if so, use the polytope center instead of origin to determine normal direction
        vec3 midpoint = vec3(0);
        if (std::abs(glm::dot(face->normal, av)) < 1e-6f) glm::vec3 midpoint = vertTot / (float) sps.size();

        print(4);

        // check winding order
        if (glm::dot(face->normal, centerFace - midpoint) < 0) {
            face->normal *= -1;
            face->sps = { pa, pc, pb }; // ensures vertices are ordered to face normal outward.
        }

        print(5);

        return face;
    }

     void visibleFaces(const SupportPoint* sp, std::vector<Face*>& visible) {
        for (const auto& face : pq) {
            // if face is looking away from point, continue
            if (glm::dot(face->normal, sp->mink) < 0) continue;
            visible.push_back(face.get());
        }
    }

     void horizonEdges(const std::vector<Face*>& visible, std::vector<Edge>& edges) {
        std::set<Edge> cloud;

        Edge edge;
        for (Face* face : visible) {
            for (int i = 0; i < 3; i++) { // loop through each edge of a face
                face->overrideEdge(i, edge);

                // remove edge if it already appears in cloud
                auto it = cloud.find({ edge.second, edge.first });
                if (it != cloud.end()) {
                    cloud.erase(it);
                    continue;
                }

                cloud.insert(edge);
            }
        }

        // return as a vector (maybe change to keep set)
        edges = std::vector<Edge>(cloud.begin(), cloud.end()); 
    }

    void erase(Face* toErase) { 
        auto it = pq.find(toErase); // fast O(log n) lookup using heterogeneous comparator
        if (it != pq.end()) pq.erase(it); // fast O(log n) erase
    }
    void erase(const std::vector<Face*>& toErase) { 
        for (Face* face : toErase) erase(face);
    }

    // inserts a new support point into the polytope
    bool insert(const SupportPoint& spRef) {
        // insert support point into the sps cloud
        const SupportPoint* sp = add(spRef);

        // check if point is already in cloud or if it is closer than the face's centroid
        auto it = sps.find(spRef);
        if (it == sps.end() || glm::length2(sp->mink) < front()->distance) return true;

        // discover which faces can "see" the new support point
        std::vector<Face*> visible; 
        visibleFaces(sp, visible);

        // generate horizon edges
        std::vector<Edge> edges;
        horizonEdges(visible, edges);

        // remove visible faces from polytope
        erase(visible);

        // add new faces from edges
        for (Edge edge : edges) add(buildFace(edge.first, edge.second, &spRef));

        return false;
    }

    Face* front() {
        if (pq.empty()) return nullptr;
        return pq.begin()->get();
    }
};

bool epa(Rigid* bodyA, Rigid* bodyB, const Simplex& simplex) {
    // generate polytope
    Polytope* polytope = new Polytope(simplex);

    print("polytope built");

    bool done = false;

    while (!done) {
        SupportPoint sp = getSupportPoint(bodyA, bodyB, vec3(1, 0, 0));

        print("found point");

        done = polytope->insert(sp);

        print("iteration complete");
    }

    print("exiting epa");

    delete polytope;

    return false;
}