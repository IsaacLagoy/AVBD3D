#ifndef RIGID_H
#define RIGID_H

#include "solver.h"

mat4x4 buildModelMatrix(const Rigid* b);
mat4x4 buildInverseModelMatrix(const Rigid* b);
mat4x4 buildModelMatrix(const vec3& pos, const vec3& sca, const quat& rot);
vec3 transform(const vec3& vertex, Rigid* body);
vec3 transform(int index, Rigid* body);
vec3 inverseTransform(const glm::vec3& worldPoint, Rigid* body);

#endif