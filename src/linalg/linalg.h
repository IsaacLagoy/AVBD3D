#ifndef LINALG_H
#define LINALG_H

#include "util/includes.h"
#include "mat6x6.h"

// matrix functions
mat6x6 outer(const vec6& a, const vec6& b);
mat3x3 outer(const vec3& a, const vec3& b);

#endif