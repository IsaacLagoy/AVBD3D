#include "solver.h"

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
    std::optional<Face> buildFace(const SupportPoint* pa, const SupportPoint* pb, const SupportPoint* pc);
    void erase(const Face& toErase);
    void erase(const std::vector<Face>& toErase);
    bool insert(const SupportPoint& spRef);
    const Face& front() const;
};

// function declarations
SupportPoint getSupportPoint(Rigid* bodyA, Rigid* bodyB, const vec3& dir);
bool handleSimplex(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex0(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex1(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex2(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex3(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);
bool      simplex4(Simplex& simplex, Rigid* bodyA, Rigid* bodyB, vec3& dir);

bool gjk(Rigid* bodyA, Rigid* bodyB, Simplex& simplex);
bool epa(Rigid* bodyA, Rigid* bodyB, Polytope* polytope);

void barycentric(std::vector<vec3>& rAs, std::vector<vec3>& rBs, Polytope* polytope, Rigid* bodyA, Rigid* bodyB);
void getContact(std::vector<vec3>& rAs, std::vector<vec3>& rBs, Polytope* polytope, Rigid* bodyA, Rigid* bodyB);