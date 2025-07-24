#ifndef INCLUDES_H
#define INCLUDES_H

#pragma once

#include <iostream>
#include <set>
#include <memory>
#include <unordered_map>

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

// shorthand names
using vec3 = glm::vec3;
using vec4 = glm::vec4;
using mat3x3 = glm::mat3x3;
using mat4x4 = glm::mat4x4;
using quat = glm::quat;

// commonly used data structures
struct vec6 {
    glm::vec3 linear;
    glm::vec3 angular;

    vec6() = default;
    vec6(const vec3& lin, const vec3 ang) : linear(lin), angular(ang) {}
    vec6(float f) : linear(f), angular(f) {}

    float& operator[](int i) {
        return i < 3 ? linear[i] : angular[i - 3];
    }

    const float& operator[](int i) const {
        return i < 3 ? linear[i] : angular[i - 3];
    }
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

float uniform();
float uniform(float min, float max);
void print(std::string str);
void print(char* str);
void print(int n);
void print(float f);

#endif