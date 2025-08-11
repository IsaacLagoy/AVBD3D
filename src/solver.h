#ifndef SOLVER_H
#define SOLVER_H

#pragma once

#include "includes.h"

#define MAX_ROWS 12           // Max scalar rows an individual constraint can have (3D contact = 3n)
#define PENALTY_MIN 1000.0f   // Minimum penalty parameter
#define PENALTY_MAX 1e9f      // Maximum penalty parameter
#define COLLISION_MARGIN 0.02f
#define STICK_THRESH 0.02f
#define SHOW_CONTACTS true

struct Rigid;
struct Force;
struct Manifold;
struct Solver;
struct Mesh;

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
    vec6 initial;
    vec6 inertial;

    vec3 scale;
    float mass;
    mat3x3 invInertiaTensor;
    float friction;
    float radius;

    // visual attributes
    vec4 color;

    Rigid(Solver* solver, vec3 size, float density, float friction, vec3 position,
          vec6 velocity = vec6(), vec4 color = vec4(0.8, 0.8, 0.8, 1));
    ~Rigid();

    vec6 getConfiguration() const;
    void setConfiguration(const vec6& config);
    bool constrainedTo(Rigid* other) const;
    void draw();

    mat3x3 getInvInertiaTensor() const;
    mat6x6 getMassMatrix() const;
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
    virtual void draw() const {};
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
    void draw() const override;
};

// no-op force used to ignore collision between two bodies
struct IgnoreCollision : Force {
    IgnoreCollision(Solver* solver, Rigid* bodyA, Rigid* bodyB) : Force(solver, bodyA, bodyB) {}

    int rows() const override { return 0; }
    bool initialize() override { return true; }
    void computeConstraint(float alpha) override {}
    void computeDerivatives(Rigid* body) override {}
    void draw() const override {}
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
    void draw() const override;

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
    void draw();
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
vec3 transform(const vec3& vertex, Rigid* body);
vec3 transform(int index, Rigid* body);

vec3 rotate(const vec3& vertex, Rigid* body);
vec3 rotate(int index, Rigid* body);

// linear algebra
vec6 solve(const mat6x6& lhs, const vec6& rhs);

#endif