#include "solver.h"

mat6x6 diagonalLump(const mat6x6& mat) {
    mat6x6 nMat = mat6x6();
    for (int c = 0; c < 6; c++) {
        float sum = 0;
        for (int r = 0; r < 6; r++) sum += mat[r][c] * mat[r][c];
        nMat[c][c] = sqrt(sum);
    }
    return nMat;
}