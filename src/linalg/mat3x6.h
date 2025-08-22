#ifndef MAT3X6_H
#define MAT3X6_H

#include "util/includes.h"
#include "vec6.h"

struct mat3x6 {
    vec6 rows[3];

    mat3x6() = default;
    mat3x6(const vec6& r1, const vec6& r2, const vec6& r3);

    // operators
    mat3x6& operator=(const mat3x6& rhs);
    vec6& operator[](int i);
    const vec6& operator[](int i) const;
    mat3x6 operator*(float rhs) const;

    // multiplication helper
    vec3 column(int i) const;
};

#endif