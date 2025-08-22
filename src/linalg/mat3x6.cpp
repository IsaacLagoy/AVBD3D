#include "mat3x6.h"

mat3x6::mat3x6(const vec6& r1, const vec6& r2, const vec6& r3) {
    rows[0] = r1;
    rows[1] = r2;
    rows[2] = r3;
}

mat3x6& mat3x6::operator=(const mat3x6& rhs) {
    if (&rhs == this) return *this;
    for (int i = 0; i < 3; i++) (*this)[i] = rhs[i];
    return *this;
}

vec6& mat3x6::operator[](int i) {
    if (i < 0 || i > 2) throw std::runtime_error("mat3x6: index out of bounds.");
    return rows[i];
}

const vec6& mat3x6::operator[](int i) const {
    if (i < 0 || i > 2) throw std::runtime_error("mat3x6: index out of bounds.");
    return rows[i];
}

mat3x6 mat3x6::operator*(float rhs) const {
    mat3x6 mat;

    for (int i = 0; i < 3; i++) mat[i] = (*this)[i] * rhs;

    return mat;
}

vec3 mat3x6::column(int i) const {
    vec3 vec;
    for (int j = 0; j < 3; j++) vec[j] = (*this)[j][i];
    return vec;
}