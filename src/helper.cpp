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

glm::mat4 buildInverseModelMatrix(const Rigid* b) {
    glm::mat4 invTranslation = glm::translate(glm::mat4(1), -b->position);
    glm::mat4 invRotation = glm::mat4(glm::conjugate(b->rotation));
    glm::mat4 invScaling = glm::scale(glm::mat4(1), 1.0f / b->scale);
    
    return invRotation * invScaling * invTranslation;
}

glm::vec3 inverseTransform(const glm::vec3& worldPoint, Rigid* body) {
    // Undo translation
    glm::vec3 p = worldPoint - body->position;

    // Undo rotation
    glm::quat invRot = glm::conjugate(glm::normalize(body->rotation));
    p = invRot * p;

    // Undo non-uniform scale
    p /= body->scale; // component-wise division

    return p;
}


mat6x6 diagonalLump(const mat6x6& mat) {
    mat6x6 nMat = mat6x6();
    for (int c = 0; c < 6; c++) {
        float sum = 0;
        for (int r = 0; r < 6; r++) sum += mat[r][c] * mat[r][c];
        nMat[c][c] = sqrt(sum);
    }
    return nMat;
}