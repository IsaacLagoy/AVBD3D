#include "linalg.h"

mat6x6 outer(const vec6& a, const vec6& b) {
    return { b * a[0], b * a[1], b * a[2], b * a[3], b * a[4], b * a[5] };
}

mat3x3 outer(const vec3& a, const vec3& b) {
    return { b * a[0], b * a[1], b * a[2] };
}