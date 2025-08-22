#ifndef RANDOM_H
#define RANDOM_H

#include <random>

#include "includes.h"

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

#endif