#ifndef SOLVER_H
#define SOLVER_H

#pragma once

#include "includes.h"

#define MAX_ROWS 12           // Max scalar rows an individual constraint can have (3D contact = 3n)
#define PENALTY_MIN 1000.0f   // Minimum penalty parameter
#define PENALTY_MAX 1e9f      // Maximum penalty parameter
#define COLLISION_MARGIN 0.04f
#define STICK_THRESH 0.02f
#define SHOW_CONTACTS true

struct Rigid;
struct Force;
struct Manifold;
struct Solver;
struct Mesh;

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

    SupportPoint() = default;
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

struct StackFace {
    std::array<SupportPoint, 3> sps;
    vec3 normal;
    float distance;

    StackFace(const Face& face) : sps(), normal(face.normal), distance(face.distance) {
        for (int i = 0; i < 3; i++) sps[i] = *face.sps[i];
    }

    StackFace() = default;
};

// contains data for a single rigid body
struct Rigid {
    Solver* solver;
    Force* forces;
    Rigid* next;

    // position and rotation stored seperately since rotation is quaternion
    vec3 position; 
    quat rotation = quat(1, 0, 0, 0); 

    vec6 velocity = vec6(0); // linear 3, angular 3
    vec6 prevVelocity = vec6(0);

    vec3 initialPosition;
    quat initialRotation;
    vec3 inertialPosition;
    quat inertialRotation;

    vec3 scale;
    float mass;
    mat3x3 inertiaTensor;
    float friction;
    float radius;

    // visual attributes
    vec4 color;

    Rigid(Solver* solver, vec3 size, float density, float friction, vec3 position, quat rotation = quat(1, 0, 0, 0),
          vec6 velocity = vec6(), vec4 color = vec4(0.8, 0.8, 0.8, 0.5));
    ~Rigid();

    bool constrainedTo(Rigid* other) const;

    mat3x3 getInertiaTensor() const;
    mat6x6 getMassMatrix() const;

    vec3 deltaWInitial() const;
    vec3 deltaWInertial() const;

    // static
    static int globalID;
};

// Provides constraint parameters and common interface for all forces.
struct Force {
    Solver* solver;
    Rigid* bodyA;
    Rigid* bodyB;

    Force* nextA;
    Force* nextB;
    Force* next;

    std::vector<vec6> J; // Jacobian rows for bodyA (bodyB are -J)
    std::vector<mat6x6> H; // Hassian/approx for complaint constraints

    float C[MAX_ROWS]; // Constraint error per row;
    float fmin[MAX_ROWS]; // Lower force/impulse limits
    float fmax[MAX_ROWS]; // Upper force/impulse limits
    float stiffness[MAX_ROWS];
    float motor[MAX_ROWS];
    float fracture[MAX_ROWS];
    float penalty[MAX_ROWS];
    float lambda[MAX_ROWS]; // Accumulated impulses (warm-start)

    Force(Solver* solver, Rigid* bodyA, Rigid* bodyB);
    virtual ~Force();

    void disable();

    virtual int rows() const = 0; // # of scalar constraint equations
    virtual bool initialize() = 0; // called once when added to solver
    virtual void computeConstraint(float alpha) = 0; // C and limits per row
    virtual void computeDerivatives(Rigid* body) = 0; // J and H per body

    // static
    static int globalID;
};

// ball-and-socket joint
struct Joint : Force {
    vec3 rA, rB; // anchor offsets in local body space
    vec3 C0; // orientation error reference

    Joint(Solver* solver, Rigid* bodyA, Rigid* bodyB, vec3 rA, vec3 rB, vec3 stiffness = vec3(INFINITY));

    int rows() const override { return 3; }
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
};

// no-op force used to ignore collision between two bodies
struct IgnoreCollision : Force {
    IgnoreCollision(Solver* solver, Rigid* bodyA, Rigid* bodyB) : Force(solver, bodyA, bodyB) {}

    int rows() const override { return 0; }
    bool initialize() override { return true; }
    void computeConstraint(float alpha) override {}
    void computeDerivatives(Rigid* body) override {}
};

struct Manifold : Force {

    struct Contact {
        vec3 rA;
        vec3 rB;
        vec3 normal; // world space contact normal A -> B
        float depth; 
        vec3 t1;
        vec3 t2;
        vec6 JAn, JBn; // normal Jacobian rows
        vec6 JAt1, JBt1; // tangent Jacobian rows
        vec6 JAt2, JBt2; // tangent Jacobian rows in the other direction
        vec3 C0; // accumulated positional error (n, t1, t2)
        bool stick; // static vs dynamic friction
        StackFace face; // saves contact data
        int type;

        Contact() : rA(), rB(), normal(), depth(0.0), t1(), t2(), JAn(), JBn(), JAt1(), JBt1(), JAt2(), JBt2(), C0(), stick(true), face() {}

        // only considers face indices
        bool operator==(const Contact& rhs) {
            for (int i = 0; i < 3; i++) 
                if (face.sps[i].indexA != rhs.face.sps[i].indexA || face.sps[i].indexB != rhs.face.sps[i].indexB)
                    return false;
            return true;
        }
    }; 

    Contact contacts[4];
    int numContacts;

    // friction variables, later change to mus and mud
    float friction;

    Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB);

    int rows() const override { return numContacts * 3; }
    bool initialize() override;
    void computeConstraint(float alpha) override;
    void computeDerivatives(Rigid* body) override;
    bool isContactStillValid(const Contact& oldContact, Rigid* bodyA, Rigid* bodyB);

    static int collide(Rigid* bodyA, Rigid* bodyB, Contact* contacts);
};

struct Solver {
    vec3 gravity;
    int iterations;

    float alpha; 
    float beta;
    float gamma;

    Rigid* bodies;
    Force* forces;
    Mesh* meshes;

    Solver();
    ~Solver();

    Rigid* pick(vec3 at, vec3& local); // ray-pick helper

    void clear();
    void defaultParams();
    void step(float dt);
};

struct Mesh {
    // Cube vertices (position only)
    inline static const float verts[72] = {
        // Positions          // Normals
        // Front face (+Z)
        -0.5f, -0.5f,  0.5f,  
        0.5f, -0.5f,  0.5f,  
        0.5f,  0.5f,  0.5f,  
        -0.5f,  0.5f,  0.5f,  

        // Back face (-Z)
        -0.5f, -0.5f, -0.5f,  
        -0.5f,  0.5f, -0.5f,  
        0.5f,  0.5f, -0.5f,  
        0.5f, -0.5f, -0.5f,  

        // Left face (-X)
        -0.5f, -0.5f, -0.5f, 
        -0.5f, -0.5f,  0.5f, 
        -0.5f,  0.5f,  0.5f, 
        -0.5f,  0.5f, -0.5f, 

        // Right face (+X)
        0.5f, -0.5f, -0.5f,  
        0.5f,  0.5f, -0.5f,
        0.5f,  0.5f,  0.5f,
        0.5f, -0.5f,  0.5f,

        // Bottom face (-Y)
        -0.5f, -0.5f, -0.5f,  
        0.5f, -0.5f, -0.5f,
        0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,

        // Top face (+Y)
        -0.5f,  0.5f, -0.5f, 
        -0.5f,  0.5f,  0.5f, 
        0.5f,  0.5f,  0.5f, 
        0.5f,  0.5f, -0.5f,
    };

    inline static const float norms[72] {
        0, 0, 1,
        0, 0, 1,
        0, 0, 1,
        0, 0, 1,

        0, 0, -1,
        0, 0, -1,
        0, 0, -1,
        0, 0, -1,

        -1, 0, 0,
        -1, 0, 0,
        -1, 0, 0,
        -1, 0, 0,

        1, 0, 0,
        1, 0, 0,
        1, 0, 0,
        1, 0, 0,

        0, -1, 0,
        0, -1, 0,
        0, -1, 0,
        0, -1, 0,

        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
        0, 1, 0,
    };

    // Cube indices for drawing with glDrawElements
    inline static const unsigned int inds[36] = {
        // Front face
        0, 1, 2,
        2, 3, 0,

        // Back face
        4, 5, 6,
        6, 7, 4,

        // Left face
        8, 9, 10,
        10, 11, 8,

        // Right face
        12, 13, 14,
        14, 15, 12,

        // Bottom face
        16, 17, 18,
        18, 19, 16,

        // Top face
        20, 21, 22,
        22, 23, 20
    };

    inline static const int numUniqueVerts = 8;
    inline static const vec3 uniqueVerts[8] = {
        vec3(-0.5, -0.5, -0.5),
        vec3(-0.5, -0.5, 0.5),
        vec3(-0.5, 0.5, -0.5),
        vec3(-0.5, 0.5, 0.5),
        vec3(0.5, -0.5, -0.5),
        vec3(0.5, -0.5, 0.5),
        vec3(0.5, 0.5, -0.5),
        vec3(0.5, 0.5, 0.5)
    };

    static int bestDot(vec3 dir);
};

// helper functions
mat4x4 buildModelMatrix(const Rigid* b);
mat4x4 buildInverseModelMatrix(const Rigid* b);
mat4x4 buildModelMatrix(const vec3& pos, const vec3& sca, const quat& rot);
vec3 transform(const vec3& vertex, Rigid* body);
vec3 transform(int index, Rigid* body);
vec3 inverseTransform(const glm::vec3& worldPoint, Rigid* body);

vec3 rotateNScale(const vec3& vertex, Rigid* body);
vec3 rotateNScale(int index, Rigid* body);

mat6x6 diagonalLump(const mat6x6& mat);

SupportPoint getSupportPoint(Rigid* bodyA, Rigid* bodyB, const vec3& dir);

// linear algebra
vec6 solve(const mat6x6& lhs, const vec6& rhs);

#endif