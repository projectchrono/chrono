//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
// =============================================================================
// Authors: Milad Rakhsha, Radu Serban
// =============================================================================
// This test evaluates the effect of size of matrices for MatrMultiplyAVX.
// You can perform this test and see how beneficial MatrMultiplyAVX can be
// on your machine on different settings
// =============================================================================

#include "chrono/core/ChTransform.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChTimer.h"
using namespace chrono;
void FillRand(ChMatrix<double>& matra) {
    int A_Nrow = matra.GetRows();
    int A_NCol = matra.GetColumns();

    for (int rowA = 0; rowA < A_Nrow; rowA++) {
        for (int colA = 0; colA < A_NCol; colA++) {
            matra.SetElement(rowA, colA, rand() % 100);
        }
    }
}

int main(int argc, char* argv[]) {
    // If you want to pollute the cache line to see if this affects the speed up or not
    bool polluteCache = true;
    // This is the number of repetition of the operation. Remember that timing only one operation is not accurate
    int ITERATION = 10000;
    ChTimer<double> timer;
    srand(time(NULL));
    bool MatrMultiply = false;
    bool MatrMultiplyT = true;

    if (MatrMultiply) {
        int MAX_A_i = 10;
        int MAX_A_j = 32;
        int MAX_B_j = 32;
        int MAX_REPEAT = 4;
        std::ofstream outFile;
        outFile.open("./AVX_MatrMultiply_Analysis.txt", std::ios::app);

        for (int A_i = 1; A_i <= MAX_A_i; A_i++) {
            for (int A_j = 1; A_j <= MAX_A_j; A_j++) {
                GetLog() << "A_i--"
                         << "A_j--"
                         << "B_j--"
                         << "SpeedUp"
                         << "\n";
                GetLog() << "---------------------\n";
                for (int B_j = 1; B_j <= MAX_B_j; B_j++) {
                    double AVE_SPEEDUP = 0;
                    for (int num_repeat = 0; num_repeat < MAX_REPEAT; num_repeat++) {
                        int A_row = A_i;
                        int A_col = A_j;
                        int B_row = A_col;
                        int B_col = B_j;
                        ChMatrixDynamic<double> A(A_row, A_col);
                        ChMatrixDynamic<double> B(B_row, B_col);  // For Multiplication
                        ChMatrixDynamic<double> C(A_row, A_col);  // For add/sub
                        ChMatrixDynamic<double> AmulB(A_row, B_col);
                        ChMatrixDynamic<double> AmulB_ref(A_row, B_col);

                        timer.reset();
                        timer.start();
                        for (int j = 0; j < ITERATION; j++) {
                            if (polluteCache) {
                                timer.stop();
                                FillRand(A);
                                FillRand(B);
                                timer.start();
                            }
                            AmulB.MatrMultiply(A, B);
                        }
                        timer.stop();

                        double tempTime = timer();
                        timer.reset();
                        timer.start();
                        for (int j = 0; j < ITERATION; j++) {
                            if (polluteCache) {
                                timer.stop();
                                FillRand(A);
                                FillRand(B);
                                timer.start();
                            }
                            AmulB.MatrMultiplyAVX(A, B);
                        }
                        timer.stop();
                        double AVXTime = timer();
                        AVE_SPEEDUP += tempTime / AVXTime;
                    }
                    outFile << A_i << " " << A_j << " " << B_j << " " << AVE_SPEEDUP / MAX_REPEAT << "\n";
                    GetLog() << A_i << " " << A_j << " " << B_j << " " << AVE_SPEEDUP / MAX_REPEAT << "\n";
                }
            }
        }

        outFile.close();
    }

    if (MatrMultiplyT) {
        int MAX_A_i = 10;
        int MAX_A_j = 32;
        int MAX_B_i = 32;
        int MAX_REPEAT = 4;
        std::ofstream outFile;
        outFile.open("./AVX_MatrMultiplyT_Analysis.txt", std::ios::app);

        for (int A_i = 1; A_i <= MAX_A_i; A_i++) {
            for (int A_j = 1; A_j <= MAX_A_j; A_j++) {
                GetLog() << "A_i--"
                         << "A_j--"
                         << "B_i--"
                         << "SpeedUp"
                         << "\n";
                GetLog() << "---------------------\n";
                for (int B_i = 1; B_i <= MAX_B_i; B_i++) {
                    double AVE_SPEEDUP = 0;
                    for (int num_repeat = 0; num_repeat < MAX_REPEAT; num_repeat++) {
                        int A_row = A_i;
                        int A_col = A_j;
                        int B_row = B_i;
                        int B_col = A_col;
                        ChMatrixDynamic<double> A(A_row, A_col);
                        ChMatrixDynamic<double> B(B_row, B_col);  // For Multiplication
                        ChMatrixDynamic<double> C(A_row, A_col);  // For add/sub
                        ChMatrixDynamic<double> AmulB(A_row, B_row);
                        ChMatrixDynamic<double> AmulB_ref(A_row, B_row);

                        timer.reset();
                        timer.start();
                        for (int j = 0; j < ITERATION; j++) {
                            if (polluteCache) {
                                timer.stop();
                                FillRand(A);
                                FillRand(B);
                                timer.start();
                            }
                            AmulB.MatrMultiplyT(A, B);
                        }
                        timer.stop();

                        double tempTime = timer();
                        timer.reset();
                        timer.start();
                        for (int j = 0; j < ITERATION; j++) {
                            if (polluteCache) {
                                timer.stop();
                                FillRand(A);
                                FillRand(B);
                                timer.start();
                            }
                            AmulB.MatrMultiplyTAVX(A, B);
                        }
                        timer.stop();
                        double AVXTime = timer();
                        AVE_SPEEDUP += tempTime / AVXTime;
                    }
                    outFile << A_i << " " << A_j << " " << B_i << " " << AVE_SPEEDUP / MAX_REPEAT << "\n";
                    GetLog() << A_i << " " << A_j << " " << B_i << " " << AVE_SPEEDUP / MAX_REPEAT << "\n";
                }
            }
        }

        outFile.close();
    }

    return 0;
}
