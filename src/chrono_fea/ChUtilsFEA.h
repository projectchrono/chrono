// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Set of utility functions for FEA module.
//
// =============================================================================

#ifndef CH_UTILS_FEA
#define CH_UTILS_FEA

#include <cmath>

#include "chrono/core/ChMatrixNM.h"

namespace chrono {
namespace fea {

/// In-place LU factorization.
/// Return false if the matrix is (close to) singular
template <int N>
bool LU_factor(ChMatrixNM<double, N, N>& A,  ///< [input/output] matrix to be factorized
               ChMatrixNM<int, N, 1>& INDX,  ///< [output] vector of pivots
               bool& pivoting                ///< [output] true if pivoting was required; false otherwise
               ) {
    double AAMAX;
    double SUM;
    double DUM;
    int IMAX;
    double TINY = 1.0e-20;
    ChMatrixNM<double, N, 1> VV;

    for (int I = 0; I < N; I++) {
        AAMAX = 0.0;
        for (int J = 0; J < N; J++) {
            if ((std::abs(A(I, J))) > AAMAX) {
                AAMAX = std::abs(A(I, J));
            }
        }
        if (AAMAX == 0.0) {
            return false;  // singular matrix
        }
        VV(I) = 1.0 / AAMAX;
    }

    pivoting = false;

    for (int J = 0; J < N; J++) {
        for (int I = 0; I < J; I++) {
            SUM = A(I, J);
            for (int K = 0; K < I; K++) {
                SUM -= A(I, K) * A(K, J);
            }
            A(I, J) = SUM;
        }

        AAMAX = 0.0;
        for (int I = J; I < N; I++) {
            SUM = A(I, J);
            for (int K = 0; K < J; K++) {
                SUM -= A(I, K) * A(K, J);
            }
            A(I, J) = SUM;
            if ((DUM = VV(I) * std::abs(SUM)) >= AAMAX) {
                IMAX = I;
                AAMAX = DUM;
            }
        }

        if (J != IMAX) {
            for (int K = 0; K < N; K++) {
                DUM = A(IMAX, K);
                A(IMAX, K) = A(J, K);
                A(J, K) = DUM;
            }
            pivoting = true;
            VV(IMAX) = VV(J);
        }

        INDX(J) = IMAX;
        if (A(J, J) == 0.0) {
            A(J, J) = TINY;
        }
        if (J != N - 1) {
            DUM = 1.0 / A(J, J);
            for (int I = J + 1; I < N; I++) {
                A(I, J) *= DUM;
            }
        }
    }

    return true;
}

/// LU linear system solution (back substitution)
template <int N>
void LU_solve(const ChMatrixNM<double, N, N>& A,  ///< [input] LU factorized matrix
              const ChMatrixNM<int, N, 1>& INDX,  ///< [output] vector of pivots
              ChMatrixNM<double, N, 1>& B         ///< [input/output] on entry, the RHS; on return, the solution vector
              ) {
    int II = 0;
    int LL;
    double SUM;
    for (int I = 0; I < N; I++) {
        LL = INDX(I);
        SUM = B(LL);
        B(LL) = B(I);
        if (II != 0) {
            for (int J = II - 1; J < I; J++) {
                SUM -= A(I, J) * B(J);
            }
        } else if (SUM != 0.0) {
            II = I + 1;
        }
        B(I) = SUM;
    }

    for (int I = N - 1; I >= 0; I--) {
        SUM = B(I);
        for (int J = I + 1; J < N; J++) {
            SUM -= A(I, J) * B(J);
        }
        B(I) = SUM / A(I, I);
    }
}

}  // end of namespace fea
}  // end of namespace chrono

#endif