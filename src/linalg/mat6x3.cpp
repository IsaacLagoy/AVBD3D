#include "mat6x3.h"

mat6x3::mat6x3(const mat3x6& transpose) {
    for (int i = 0; i < 6; i++) rows[i] = vec3(transpose[0][i], transpose[1][i], transpose[2][i]);
}

vec3& mat6x3::operator[](int i) {
    if (i < 0 || i > 5) throw std::runtime_error("mat6x6: index out of bounds.");
    return rows[i];
}

const vec3& mat6x3::operator[](int i) const {
    if (i < 0 || i > 5) throw std::runtime_error("mat6x6: index out of bounds.");
    return rows[i];
}

mat6x3 mat6x3::operator*(float rhs) const {
    mat6x3 mat;
    for (int i = 0; i < 6; i++) mat[i] = (*this)[i] * rhs;
    return mat;
}

mat6x6 mat6x3::operator*(const mat3x6& rhs) const {
    mat6x6 mat;

    // loop through columns of rhs for locality
    for (int c = 0; c < 6; c++) { 
        vec3 column = rhs.column(c);
        for (int r = 0; r < 6; r++) mat[r][c] = glm::dot((*this)[r], column);
    }

    return mat;
}

mat6x3 transpose(const mat3x6& mat) {
    return mat6x3(mat); // baked in transpose constructor
}