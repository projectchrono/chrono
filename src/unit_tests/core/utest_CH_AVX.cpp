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
// Authors: Milad Rakhsha, Radu Serban
// =============================================================================
//
// Unit test for MatrMultiplyAVX and MatrMultiplyTAVX.
//
// =============================================================================

#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChLog.h"

using namespace chrono;

void FillRand(ChMatrix<double>& matra) {
    int A_Nrow = matra.GetRows();
    int A_NCol = matra.GetColumns();

    for (int rowA = 0; rowA < A_Nrow; rowA++) {
        for (int colA = 0; colA < A_NCol; colA++) {
            matra.SetElement(rowA, colA, double(rand() % 100) / 100);
        }
    }
}

// Check multiplication A*B of random matrices A (MxN) and B (NxK)
bool CheckMatMult(int M, int N, int K, double tolerance) {
    GetLog() << "(" << M << "x" << N << ") * (" << N << "x" << K << ")   ... ";

    ChMatrixDynamic<double> A(M, N);
    ChMatrixDynamic<double> B(N, K);
    A.FillRandom(10, -10);
    B.FillRandom(10, -10);

    ChMatrixDynamic<double> ref(M, K);
    ref.MatrMultiply(A, B);

    ChMatrixDynamic<double> avx(M, K);
    avx.MatrMultiplyAVX(A, B);

    if (avx.Equals(ref, tolerance)) {
        GetLog() << "OK\n";
        return true;
    }

    GetLog() << "FAILED\n";
    GetLog() << "\n(A*B)_ref";
    ref.StreamOUT(GetLog());
    GetLog() << "\n(A*B)_avx";
    avx.StreamOUT(GetLog());
    GetLog() << "\n(A*B)_avx - (A*B)_ref";
    (avx - ref).StreamOUT(GetLog());

    return false;
}

// Check multiplication A*B' of random matrices A (MxN) and B (KxN)
bool CheckMatMultT(int M, int N, int K, double tolerance) {
    GetLog() << "(" << M << "x" << N << ") * (" << K << "x" << N << ")^T   ... ";

    ChMatrixDynamic<double> A(M, N);
    ChMatrixDynamic<double> B(K, N);
    A.FillRandom(10, -10);
    B.FillRandom(10, -10);

    ChMatrixDynamic<double> ref(M, K);
    ref.MatrMultiplyT(A, B);

    ChMatrixDynamic<double> avx(M, K);
    avx.MatrMultiplyTAVX(A, B);

    if (avx.Equals(ref, tolerance)) {
        GetLog() << "OK\n";
        return true;
    }

    GetLog() << "FAILED\n";
    GetLog() << "\n(A*B')_ref";
    ref.StreamOUT(GetLog());
    GetLog() << "\n(A*B')_avx";
    avx.StreamOUT(GetLog());
    GetLog() << "\n(A*B')_avx - (A*B')_ref";
    (avx - ref).StreamOUT(GetLog());

    return false;
}

int main(int argc, char* argv[]) {
    // Print differences between standard and AVX-based multiplicaitons
    bool printMul = true;

    // Tolerance for comparing matrices
    double tolerance = 1e-12;

    // Result of unit tests
    bool passed = true;

    // Initialize seed for rand()
    srand(static_cast<unsigned int>(time(nullptr)));

    GetLog() << "\n-----------------MatrMultiply---------------------- \n";

    passed &= CheckMatMult(20, 8, 24, tolerance);
    passed &= CheckMatMult(21, 8, 24, tolerance);
    passed &= CheckMatMult(22, 8, 24, tolerance);
    passed &= CheckMatMult(23, 8, 24, tolerance);

    passed &= CheckMatMult(20, 9, 24, tolerance);
    passed &= CheckMatMult(21, 9, 24, tolerance);
    passed &= CheckMatMult(22, 9, 24, tolerance);
    passed &= CheckMatMult(23, 9, 24, tolerance);

    passed &= CheckMatMult(20, 10, 24, tolerance);
    passed &= CheckMatMult(21, 10, 24, tolerance);
    passed &= CheckMatMult(22, 10, 24, tolerance);
    passed &= CheckMatMult(23, 10, 24, tolerance);

    passed &= CheckMatMult(20, 11, 24, tolerance);
    passed &= CheckMatMult(21, 11, 24, tolerance);
    passed &= CheckMatMult(22, 11, 24, tolerance);
    passed &= CheckMatMult(23, 11, 24, tolerance);

    GetLog() << "\n-----------------MatrMultiplyT---------------------- \n";

    passed &= CheckMatMultT(20, 8, 24, tolerance);
    passed &= CheckMatMultT(21, 8, 24, tolerance);
    passed &= CheckMatMultT(22, 8, 24, tolerance);
    passed &= CheckMatMultT(23, 8, 24, tolerance);

    passed &= CheckMatMultT(20, 9, 24, tolerance);
    passed &= CheckMatMultT(21, 9, 24, tolerance);
    passed &= CheckMatMultT(22, 9, 24, tolerance);
    passed &= CheckMatMultT(23, 9, 24, tolerance);

    passed &= CheckMatMultT(20, 10, 24, tolerance);
    passed &= CheckMatMultT(21, 10, 24, tolerance);
    passed &= CheckMatMultT(22, 10, 24, tolerance);
    passed &= CheckMatMultT(23, 10, 24, tolerance);

    passed &= CheckMatMultT(20, 11, 24, tolerance);
    passed &= CheckMatMultT(21, 11, 24, tolerance);
    passed &= CheckMatMultT(22, 11, 24, tolerance);
    passed &= CheckMatMultT(23, 11, 24, tolerance);

    // Return 0 if all tests passed.
    return !passed;
}
