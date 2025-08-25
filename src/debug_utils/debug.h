#ifndef DEBUG_H
#define DEBUG_H

#include "util/includes.h"

inline bool hasNaN(const vec3& v) {
    return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z);
}

#endif