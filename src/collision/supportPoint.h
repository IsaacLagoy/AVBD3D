#ifndef SUPPORTPOINT_H
#define SUPPORTPOINT_H

#include "util/includes.h"

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

using Edge = std::pair<const SupportPoint*, const SupportPoint*>;

#endif