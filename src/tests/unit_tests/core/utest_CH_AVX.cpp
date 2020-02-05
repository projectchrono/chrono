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
// Authors: Milad Rakhsha, Radu Serban
// =============================================================================
//
// Unit test for MatrMultiplyAVX and MatrMultiplyTAVX.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChMatrixDynamic.h"

using namespace chrono;

class MatMult : public ::testing::Test, public ::testing::WithParamInterface<std::tuple<int, int, int>> {
  public:
    MatMult() : verbose(false), tolerance(1e-12) {
        M = std::get<0>(GetParam());
        N = std::get<1>(GetParam());
        K = std::get<2>(GetParam());
    }

  protected:
    bool verbose;
    double tolerance;
    int M;
    int N;
    int K;
};

TEST_P(MatMult, AB) {
    ChMatrixDynamic<double> A(M, N);
    ChMatrixDynamic<double> B(N, K);
    A.FillRandom(10, -10);
    B.FillRandom(10, -10);

    ChMatrixDynamic<double> ref(M, K);
    ref.MatrMultiply(A, B);

    ChMatrixDynamic<double> avx(M, K);
    avx.MatrMultiplyAVX(A, B);

    if (!avx.Equals(ref, tolerance)) {
        if (verbose) {
            GetLog() << "(" << M << "x" << N << ") * (" << N << "x" << K << ")\n";
            GetLog() << "\n(A*B)_ref";
            ref.StreamOUT(GetLog());
            GetLog() << "\n(A*B)_avx";
            avx.StreamOUT(GetLog());
            GetLog() << "\n(A*B)_avx - (A*B)_ref";
            (avx - ref).StreamOUT(GetLog());
        }
        FAIL();
    }
}

TEST_P(MatMult, ABt) {
    ChMatrixDynamic<double> A(M, N);
    ChMatrixDynamic<double> B(K, N);
    A.FillRandom(10, -10);
    B.FillRandom(10, -10);

    ChMatrixDynamic<double> ref(M, K);
    ref.MatrMultiplyT(A, B);

    ChMatrixDynamic<double> avx(M, K);
    avx.MatrMultiplyTAVX(A, B);

    if (!avx.Equals(ref, tolerance)) {
        if (verbose) {
            GetLog() << "(" << M << "x" << N << ") * (" << K << "x" << N << ")^T\n";
            GetLog() << "\n(A*B')_ref";
            ref.StreamOUT(GetLog());
            GetLog() << "\n(A*B')_avx";
            avx.StreamOUT(GetLog());
            GetLog() << "\n(A*B')_avx - (A*B')_ref";
            (avx - ref).StreamOUT(GetLog());
        }
        FAIL();
    }
}

INSTANTIATE_TEST_CASE_P(AVX,
                        MatMult,
                        ::testing::Combine(::testing::Values(20, 21, 22, 23),
                                           ::testing::Values(8, 9, 10, 11),
                                           ::testing::Values(24)));
