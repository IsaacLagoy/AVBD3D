#ifndef MAT6X3_H
#define MAT6X3_H

#include "mat3x6.h"
#include "mat6x6.h"

struct mat6x3 {
    vec3 rows[6];

    mat6x3() = default;
    mat6x3(const mat3x6& transpose);

    // operators
    vec3& operator[](int i);
    const vec3& operator[](int i) const;
    mat6x6 operator*(const mat3x6& rhs) const;
    mat6x3 operator*(float rhs) const;
};

mat6x3 transpose(const mat3x6& mat);

#endif