#ifndef VEC6_H
#define VEC6_H

#include "util/includes.h"
#include "debug/debug.h"

// commonly used data structures
struct vec6 {
    glm::vec3 linear;
    glm::vec3 angular;

    vec6() = default;
    vec6(const vec3& lin, const vec3& ang) : linear(lin), angular(ang) {
        if (hasNaN(linear)) throw std::runtime_error("vec6(const vec3& lin, const vec3& ang) Linear component of copy has NaN");
        if (hasNaN(angular)) throw std::runtime_error("vec6(const vec3& lin, const vec3& ang) Angular component of copy has NaN");
    }
    vec6(const vec6& vec) : linear(vec.linear), angular(vec.angular) {
        if (hasNaN(linear)) throw std::runtime_error("vec6(const vec6& vec) Linear component of copy has NaN");
    }
    vec6(float x, float y, float z, float ax, float ay, float az) : linear(x, y, z), angular(ax, ay, az) {
        if (std::isnan(x)) throw std::runtime_error("vec6(float x, float y, float z, float ax, float ay, float az) x component of copy has NaN");
        if (std::isnan(y)) throw std::runtime_error("vec6(float x, float y, float z, float ax, float ay, float az) y component of copy has NaN");
        if (std::isnan(z)) throw std::runtime_error("vec6(float x, float y, float z, float ax, float ay, float az) z component of copy has NaN");
        if (std::isnan(ax)) throw std::runtime_error("vec6(float x, float y, float z, float ax, float ay, float az) ax component of copy has NaN");
        if (std::isnan(ay)) throw std::runtime_error("vec6(float x, float y, float z, float ax, float ay, float az) ay component of copy has NaN");
        if (std::isnan(az)) throw std::runtime_error("vec6(float x, float y, float z, float ax, float ay, float az) az component of copy has NaN");
    }
    vec6(float f) : linear(f), angular(f) {
        if (std::isnan(f)) throw std::runtime_error("vec6(float f) f component of copy has NaN");
    }

    // operators
    vec6& operator=(const vec6& vec);
    float& operator[](int i);
    const float& operator[](int i) const;
    vec6 operator+(const vec6& rhs) const;
    vec6 operator-(const vec6& rhs) const;
    vec6 operator*(float rhs) const;
    vec6 operator/(float rhs) const;
    vec6& operator+=(const vec6& rhs);
};

float dot(vec6 v1, vec6 v2);

#endif