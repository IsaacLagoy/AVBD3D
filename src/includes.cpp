#include "includes.h"
#include <random>

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
    std::cout << "<" << vec.x << ", " << vec.y << ", " << vec.z << ">" << std::endl;
}