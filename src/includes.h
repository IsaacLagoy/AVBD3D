#ifndef INCLUDES_H
#define INCLUDES_H

#pragma once

#include <iostream>
#include <set>
#include <memory>
#include <unordered_map>
#include <vector>
#include <array>
#include <optional>

// import glad and glfw
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// import glm
#include <glm/glm.hpp>

// look for future changes to glm experimental
#define GLM_ENABLE_EXPERIMENTAL 
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

// import assimp Jonah Stuff
#include <assimp/scene.h>
#include <stb/stb_image.h>

#define DEBUG_PRINT false
#define DEBUG_LINEAR_PRINT false

// shorthand names
using vec2 = glm::vec2;
using vec3 = glm::vec3;
using vec4 = glm::vec4;
using mat3x3 = glm::mat3x3;
using mat4x4 = glm::mat4x4;
using quat = glm::quat;

// debug functions
bool hasNaN(const vec3& v);

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

// only used for transpose
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

template <typename T, size_t N>
class UnorderedArray {
    std::array<T, N> data;
    size_t _size = 0;

    public:
    void add(const T& element) {
        if (_size == N) return;
        data[_size++] = element;
    }

    void remove(size_t index) {
        if (index >= _size) return;
        if (index < _size - 1) std::swap(data[index], data[_size - 1]);
        _size--;
    }

    T& operator[](size_t index) { return data[index]; }
    const T& operator[](size_t index) const { return data[index]; }

    void clear() { _size = 0; }
    size_t size() const { return _size; }
};

// random
float uniform();
float uniform(float min, float max);

// printing
void print(std::string str);
void print(char* str);
void print(int n);
void print(float f);
void print(const vec3& vec);
void print(const vec6& vec);
void print(const quat& quat);
void print(const mat6x6& mat);
void print(const mat3x3& mat);

// vec6
float dot(vec6 v1, vec6 v2);

// rotation conversions
vec3 logMapSO3(quat q);
quat expMapSO3(vec3 omega);

// matrix functions
mat3x3 skewSymmetricCrossProductMatrix(const vec3& vec);
mat6x3 transpose(const mat3x6& mat);
mat6x6 outer(const vec6& a, const vec6& b);
mat3x3 outer(const vec3& a, const vec3& b);

#endif