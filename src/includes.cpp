#include "includes.h"
#include <random>
#include <iomanip>

// debugging
bool hasNaN(const vec3& v) {
    return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z);
}

// random
float uniform(float min, float max) {
    static std::mt19937 rng(std::random_device{}());  // Seed the random engine once
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

float uniform() {
    static std::mt19937 rng(std::random_device{}());  // Seed the random engine once
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return dist(rng);
}


// printing
void print(std::string str) {
    std::cout << str << std::endl;
}

void print(char* str) {
    std::cout << str << std::endl;
}

void print(int n) {
    std::cout << n << std::endl;
}

void print(float f) {
    std::cout << f << std::endl;
}

void print(const vec3& vec) {
    std::cout << "<" << vec.x << "\t" << vec.y << "\t" << vec.z << ">" << std::endl;
}

void print(const quat& quat) {
    std::cout << "<" << quat.w << "\t" << quat.x << "\t" << quat.y << "\t" << quat.z << ">" << std::endl;
}

void print(const vec6& vec) {
    std::cout << "<";
    for (int i = 0; i < 5; i++) std::cout << vec[i] << "\t";
    std::cout << vec[5] << ">" << std::endl;
}

void print(const mat6x6& mat) {
    for (int i = 0; i < 6; i++) print(mat[i]);
}

void print(const mat3x3& mat) {
    for (int i = 0; i < 3; i++) print(mat[i]);
}

// vec6
float dot(vec6 v1, vec6 v2) {
    return glm::dot(v1.linear, v2.linear) + glm::dot(v1.angular, v2.angular);
}

// rotation mapping
vec3 logMapSO3(quat q) { // ensure q is normalized
    if (q.w < 0) q = -q; // find shortest path rotation

    float angle = 2.0f * acos(glm::clamp(q.w, -1.0f, 1.0f));
    float sinHalfAngle = sqrt(glm::max(0.0f, 1.0f - q.w * q.w)); // max included to avoid error

    vec3 axis = vec3(q.x, q.y, q.z);
    if (sinHalfAngle < 1e-7f || glm::length2(axis) < 1e-7f) return vec3(0); // no rotation
    axis = glm::normalize(axis);

    return axis * angle;
}

quat expMapSO3(vec3 omega) {
    float angle = glm::length(omega);
    if (angle < 1e-7f) return quat(1, 0, 0, 0); // no rotation

    vec3 axis = omega / angle;
    float halfAngle = 0.5f * angle;

    return glm::normalize(quat(cos(halfAngle), axis * sin(halfAngle)));
}

mat3x3 skewSymmetricCrossProductMatrix(const vec3& vec) {
    return mat3x3(
        0, -vec.z, vec.y, 
        vec.z, 0, -vec.x, 
        -vec.y, vec.x, 0
    );
}

mat6x3 transpose(const mat3x6& mat) {
    return mat6x3(mat); // baked in transpose constructor
}

mat6x6 outer(const vec6& a, const vec6& b) {
    return { b * a[0], b * a[1], b * a[2], b * a[3], b * a[4], b * a[5] };
}