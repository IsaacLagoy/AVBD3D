#ifndef MAT6X6_H
#define MAT6X6_H

#include "util/includes.h"
#include "vec6.h"

struct mat6x6 {
    vec6 rows[6];

    mat6x6() = default;
    mat6x6(const mat3x3& tl, const mat3x3& tr, const mat3x3& bl, const mat3x3& br);
    mat6x6(const vec6& r1, const vec6& r2, const vec6& r3, const vec6& r4, const vec6& r5, const vec6& r6);
    ~mat6x6();

    // operators
    mat6x6& operator=(const mat6x6& rhs);
    vec6& operator[](int i);
    const vec6& operator[](int i) const;
    mat6x6 operator+(const mat6x6& rhs) const;
    mat6x6& operator+=(const mat6x6& rhs);
    mat6x6 operator*(float rhs) const;
    vec6 operator*(const vec6& rhs) const;

    mat6x6 operator/(float rhs) const;
    void addBottomRight(const mat3x3& mat);
};

#endif