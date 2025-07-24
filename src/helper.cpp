#include "solver.h"

mat4x4 buildModelMatrix(const Rigid* b) {
    mat4x4 translation = glm::translate(mat4x4(1), b->position);
    mat4x4 scaling = glm::scale(mat4x4(1), b->scale);
    mat4x4 rotate = mat4x4(b->rotation);

    return translation * rotate * scaling;
}