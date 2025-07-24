#include "solver.h"

int Mesh::bestDot(vec3 dir) {
    bool four = dir.x >= 0;
    bool two = dir.y >= 0;
    bool one = dir.z >= 0;

    return 4 * four + 2 * two + 1 * one;
}