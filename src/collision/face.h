#ifndef FACE_H
#define FACE_H

#include "supportPoint.h"

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

#endif