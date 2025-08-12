#include "solver.h"

mat4x4 buildModelMatrix(const Rigid* b) {
    mat4x4 translation = glm::translate(mat4x4(1), b->position);
    mat4x4 scaling = glm::scale(mat4x4(1), b->scale);
    mat4x4 rotate = mat4x4(b->rotation);

    return translation * rotate * scaling;
}

mat4x4 buildModelMatrix(const vec3& pos, const vec3& sca, const quat& rot) {
    mat4x4 translation = glm::translate(mat4x4(1), pos);
    mat4x4 scaling = glm::scale(mat4x4(1), sca);
    mat4x4 rotate = mat4x4(rot);

    return translation * rotate * scaling;
}