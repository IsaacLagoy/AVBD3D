#include "solver.h"

vec6 solve(const mat6x6& lhs, const vec6& rhs) {
    mat6x6 L = mat6x6();
    vec6 D = vec6();

    print("lhs");
    for (int i = 0; i < 6; i++) print(lhs[i]);
    print("rhs");
    print(rhs);


    // compute LDL^T decomposition
    for (int i = 0; i < 6; i++) {
        // compute D[i]
        float sum = 0.0f;
        for (int j = 0; j < i; j++) sum += L[i][j] * L[i][j] * D[j];
        D[i] = lhs[i][i] - sum;

        // compute L[j][i] for j > i
        for (int j = i + 1; j < 6; j++) {
            float s = lhs[j][i];
            for (int k = 0; k < i; k++) s -= L[j][k] * L[i][k] * D[k];
            L[j][i] = s / D[i];
        }
    }

    print("D");
    print(D);

    // forward substitution: solve Ly = b
    vec6 y;
    for (int i = 0; i < 6; i++) {
        float sum = 0.0f;
        for (int j = 0; j < i; j++) sum += L[i][j] * y[j];
        y[i] = rhs[i] - sum;
    }

    print("y");
    print(y);

    // diagonal solve: solve Dz = y
    vec6 z;
    for (int i = 0; i < 6; i++) z[i] = y[i] / D[i];

    print("z");
    print(z);

    // backward substitution: solve L^T x = z
    vec6 x;
    for (int i = 5; i >= 0; i--) {
        float sum = 0.0f;
        for (int j = i + 1; j < 6; j++) sum += L[j][i] * x[j];
        x[i] = z[i] - sum;
    }

    print("x");
    print(x);

    return x;
}