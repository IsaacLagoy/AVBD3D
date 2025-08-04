#include "solver.h"

#define DEBUG_PRINT_GJK false

// support point
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

struct SupportPointHash {
    size_t operator()(const SupportPoint& sp) const {
        size_t h1 = std::hash<int>{}(sp.indexA);
        size_t h2 = std::hash<int>{}(sp.indexB);
        return h1 ^ (h2 << 1);  // combine hashes
    }
};

struct SupportPointEqual {
    bool operator()(const SupportPoint& a, const SupportPoint& b) const {
        return a.indexA == b.indexA && a.indexB == b.indexB;
    }
};

// simplex
using Simplex = UnorderedArray<SupportPoint, 4>;
enum Index { A, B, C, D };

// edge
using Edge = std::pair<const SupportPoint*, const SupportPoint*>;

// polytope face
struct Face {
    std::array<const SupportPoint*, 3> sps;
    vec3 normal;
    float distance;

    bool operator==(const Face& other) const {
        return sps == other.sps;
    }

    // overrides edge reference with indexed edge from face
    void overrideEdge(int i, Edge& edge) const {
        edge = { sps[i % 3], sps[(i + 1) % 3] }; 
    }
};

struct Compare {
    // Compare FacePtrs
    bool operator()(const Face& a, const Face& b) const {
        return a.distance > b.distance;
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
    Face buildFace(const SupportPoint* pa, const SupportPoint* pb, const SupportPoint* pc);
    void erase(const Face& toErase);
    void erase(const std::vector<Face>& toErase);
    bool insert(const SupportPoint& spRef);
    const Face& front() const;
};


// function declarations
vec3 transform(const vec3& vertex, Rigid* body);
vec3 transform(int index, Rigid* body);

bool handleSimplex(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex0(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex1(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex2(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex3(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex4(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);

bool gjk(Rigid* bodyA, Rigid* bodyB, Simplex& simplex);
bool epa(Rigid* bodyA, Rigid* bodyB, Polytope* polytope);

std::pair<vec3, vec3> barycentric(Polytope* polytope, Rigid* bodyA, Rigid* bodyB);