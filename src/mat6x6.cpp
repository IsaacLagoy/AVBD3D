#include "includes.h"

// used for creating mass matrix
mat6x6::mat6x6(const mat3x3& tl, const mat3x3& tr, const mat3x3& bl, const mat3x3& br) {
    for (int i = 0; i < 3; i++) rows[i + 0] = vec6(tl[i][0], tl[i][1], tl[i][2], tr[i][0], tr[i][1], tr[i][2]);
    for (int i = 0; i < 3; i++) rows[i + 3] = vec6(bl[i][0], bl[i][1], bl[i][2], br[i][0], br[i][1], br[i][2]);
}

mat6x6::mat6x6(const vec6& r1, const vec6& r2, const vec6& r3, const vec6& r4, const vec6& r5, const vec6& r6) {
    rows[0] = r1;
    rows[1] = r2;
    rows[2] = r3;
    rows[3] = r4;
    rows[4] = r5;
    rows[5] = r6;
}

mat6x6::~mat6x6() {}

// copy operator
mat6x6& mat6x6::operator=(const mat6x6& rhs) {
    if (&rhs == this) return *this;
    for (int i = 0; i < 6; i++) rows[i] = rhs[i];
    return *this;
}

vec6& mat6x6::operator[](int i) {
    if (i < 0 || i > 5) throw std::runtime_error("mat6x6: index out of bounds.");
    return rows[i];
}

const vec6& mat6x6::operator[](int i) const {
    if (i < 0 || i > 5) throw std::runtime_error("mat6x6: index out of bounds.");
    return rows[i];
}

mat6x6 mat6x6::operator+(const mat6x6& rhs) const {
    mat6x6 mat = mat6x6();
    for (int i = 0; i < 6; i++) mat[i] = (*this)[i] + rhs[i];
    return mat;
}

mat6x6& mat6x6::operator+=(const mat6x6& rhs) {
    for (int i = 0; i < 6; i++) (*this)[i] += rhs[i];
    return *this;
}

mat6x6 mat6x6::operator*(float rhs) const {
    mat6x6 mat = mat6x6();
    for (int i = 0; i < 6; i++) mat[i] = (*this)[i] * rhs;
    return mat;
}

mat6x6 mat6x6::operator/(float rhs) const {
    mat6x6 mat = mat6x6();
    for (int i = 0; i < 6; i++) mat[i] = (*this)[i] / rhs;
    return mat;
}

vec6 mat6x6::operator*(const vec6& rhs) const {
    vec6 vec = vec6();
    // take the dot product of every row with rhs
    for (int i = 0; i < 6; i++) vec[i] = dot((*this)[i], rhs);
    return vec;
}

void mat6x6::addBottomRight(const mat3x3& mat) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            (*this)[i][j] += mat[i][j];
        }
    }
}