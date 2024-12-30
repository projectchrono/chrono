// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/fea/ChPolarDecomposition.h"

namespace chrono {
namespace fea {

// compute the one-norm of a 3x3 matrix (row-major)
double PolarDecomposition::oneNorm(const double* A) {
    double norm = 0.0;
    for (int i = 0; i < 3; i++) {
        double columnAbsSum = fabs(A[i + 0]) + fabs(A[i + 3]) + fabs(A[i + 6]);
        if (columnAbsSum > norm)
            norm = columnAbsSum;
    }
    return norm;
}

// compute the inf-norm of a 3x3 matrix (row-major)
double PolarDecomposition::infNorm(const double* A) {
    double norm = 0.0;
    for (int i = 0; i < 3; i++) {
        double rowSum = fabs(A[3 * i + 0]) + fabs(A[3 * i + 1]) + fabs(A[3 * i + 2]);
        if (rowSum > norm)
            norm = rowSum;
    }
    return norm;
}

// Input: M (3x3 mtx)
// Output: Q (3x3 rotation mtx), S (3x3 symmetric mtx)
double PolarDecomposition::Compute(const double* M, double* Q, double* S, double tolerance) {
    double Mk[9];
    double Ek[9];
    double det, M_oneNorm, M_infNorm, E_oneNorm;

    // Mk = M^T
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Mk[3 * i + j] = M[3 * j + i];

    M_oneNorm = oneNorm(Mk);
    M_infNorm = infNorm(Mk);

    do {
        double MadjTk[9];

        // row 2 x row 3
        crossProduct(&(Mk[3]), &(Mk[6]), &(MadjTk[0]));
        // row 3 x row 1
        crossProduct(&(Mk[6]), &(Mk[0]), &(MadjTk[3]));
        // row 1 x row 2
        crossProduct(&(Mk[0]), &(Mk[3]), &(MadjTk[6]));

        det = Mk[0] * MadjTk[0] + Mk[1] * MadjTk[1] + Mk[2] * MadjTk[2];
        if (det == 0.0) {
            printf("Warning (polarDecomposition) : zero determinant encountered.\n");
            break;
        }

        double MadjT_one = oneNorm(MadjTk);
        double MadjT_inf = infNorm(MadjTk);

        double gamma = std::sqrt(std::sqrt((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm)) / std::fabs(det));
        double g1 = gamma * 0.5;
        double g2 = 0.5 / (gamma * det);

        for (int i = 0; i < 9; i++) {
            Ek[i] = Mk[i];
            Mk[i] = g1 * Mk[i] + g2 * MadjTk[i];
            Ek[i] -= Mk[i];
        }

        E_oneNorm = oneNorm(Ek);
        M_oneNorm = oneNorm(Mk);
        M_infNorm = infNorm(Mk);
    } while (E_oneNorm > M_oneNorm * tolerance);

    // Q = Mk^T
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Q[3 * i + j] = Mk[3 * j + i];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            S[3 * i + j] = 0;
            for (int k = 0; k < 3; k++)
                S[3 * i + j] += Mk[3 * i + k] * M[3 * k + j];
        }

    // S must be symmetric; enforce the symmetry
    for (int i = 0; i < 3; i++)
        for (int j = i; j < 3; j++)
            S[3 * i + j] = S[3 * j + i] = 0.5 * (S[3 * i + j] + S[3 * j + i]);

    return (det);
}

}  // end namespace fea
}  // end namespace chrono
